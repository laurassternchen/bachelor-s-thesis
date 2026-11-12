#!/usr/bin/env python3
"""Sets the known start pose, sends one of the three fixed goals, and
records the outcome -- including rejection, which never appears on the
action status topic the SafetyMonitor itself watches (correction 2).

Every config starts from the same pose (x_pose:=0 y_pose:=0 -- every sim
launch file's own default, so no override is needed on the command line)
so AMCL's particle filter converges to the same place this script
publishes to /initialpose.

Usage:
    ros2 run safety_monitor send_goal.py --ros-args \\
        -p config_id:=base -p run_id:=run0 -p goal:=goal1
"""
import json
import math
import os
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

GOALS = {
    'goal1': (0.0, 4.2, 0.0),
    'goal2': (-6.0, -3.0, 0.0),
    'goal3': (-9.0, 0.0, 0.0),
}

# Matches RViz's own "2D Pose Estimate" tool, so AMCL sees a familiar
# covariance shape for the initial pose.
INITIALPOSE_COVARIANCE = [0.0] * 36
INITIALPOSE_COVARIANCE[0] = 0.25
INITIALPOSE_COVARIANCE[7] = 0.25
INITIALPOSE_COVARIANCE[35] = 0.06853892326654787

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_UNKNOWN: 'unknown',
}


def yaw_to_quat(yaw_rad):
    half = yaw_rad / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


class SendGoal(Node):

    def _spin_sleep(self, duration_s, step_s=0.05):
        """Spins this node while waiting, instead of blocking with
        time.sleep(). A raw sleep stalls the whole process without ever
        invoking rclpy callbacks, so tf2_ros.TransformListener's /tf
        subscription (and anything else callback-driven) receives nothing
        during the wait -- this was silently starving the TF buffer during
        every pose-reset settle period, intermittently making a later
        lookup_transform fail outright (no data at all) rather than just
        reading something stale."""
        deadline = time.time() + duration_s
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return
            rclpy.spin_once(self, timeout_sec=min(step_s, remaining))

    def __init__(self):
        super().__init__('send_goal')
        p = self.declare_parameter
        self.config_id = p('config_id', 'base').value
        self.run_id = p('run_id', 'run0').value
        self.bags_dir = p('bags_dir', 'bags').value
        self.goal_name = p('goal', 'goal1').value
        self.start_x = p('start_x', 0.0).value
        self.start_y = p('start_y', 0.0).value
        self.start_yaw = p('start_yaw', 0.0).value
        self.warn_distance_m = p('warn_distance_m', 0.5).value
        self.max_goal_s = p('max_goal_s', 120.0).value
        # False for every goal in a circuit except its first: lets goal2/goal3
        # continue from wherever the previous goal in the same circuit left
        # the robot, instead of snapping AMCL's belief back to the start pose
        # while the robot physically stays put.
        self.reset_pose = p('reset_pose', True).value
        self.world_name = p('world_name', 'my_room').value
        self.robot_entity_name = p('robot_entity_name', 'turtlebot3_waffle').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.monitor_params_client = self.create_client(
            SetParameters, '/safety_monitor_node/set_parameters')
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

    def _sync_monitor_run_id(self):
        """Pushes this run's config_id/run_id into an already-running
        safety_monitor_node, so one long-lived monitor session can cover
        many goal-sends without a restart between them -- each one's
        monitor_summary.json still lands under its own run_id."""
        # 2.0s used to be enough, but this is the first thing a freshly
        # started node does -- DDS discovery of the monitor's service can
        # itself take a few seconds (reproduced directly: a brand-new
        # /initialpose publisher needed ~3s to match AMCL's subscriber in
        # this same environment), so a short timeout here intermittently
        # reports "unavailable" even though the monitor is running fine.
        # Returns False if the push never landed. The caller must then not
        # send the goal: the monitor would open this run's window under the
        # previous run_id and overwrite that run's monitor_summary.json.
        request = SetParameters.Request()
        for name, value in (('config_id', self.config_id), ('run_id', self.run_id)):
            request.parameters.append(Parameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)))
        for attempt in range(1, 4):
            if not self.monitor_params_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(
                    f'safety_monitor_node/set_parameters unavailable (attempt {attempt}/3)')
                continue
            future = self.monitor_params_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            response = future.result()
            if response is not None and all(r.successful for r in response.results):
                return True
            self.get_logger().warn(f'run_id sync to safety_monitor_node failed (attempt {attempt}/3)')
        return False

    def _teleport_to_start(self):
        """Moves the robot's actual simulated pose in Gazebo back to the
        start pose. Needed because /initialpose only resets AMCL's belief,
        not the physics engine's own idea of where the robot is -- without
        this, starting a new circuit by publishing /initialpose at (0, 0)
        would tell AMCL to believe it's at the origin while the robot is
        physically wherever the previous circuit's last goal left it,
        which breaks localization (the laser scan won't match the
        asserted pose)."""
        qx, qy, qz, qw = yaw_to_quat(self.start_yaw)
        req = (f'name: "{self.robot_entity_name}" '
               f'position: {{x: {self.start_x}, y: {self.start_y}, z: 0.0}} '
               f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}')
        try:
            subprocess.run(
                ['gz', 'service', '-s', f'/world/{self.world_name}/set_pose',
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000', '--req', req],
                check=True, capture_output=True, text=True, timeout=5.0)
            self.get_logger().info(
                f'Teleported {self.robot_entity_name} to '
                f'({self.start_x}, {self.start_y})')
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired,
                FileNotFoundError) as e:
            self.get_logger().error(
                f'Teleport to start pose failed ({e}) -- /initialpose is about '
                f'to mismatch the robot\'s real position')
        self._spin_sleep(0.5)  # let physics settle before anything reads the new pose

    def _warn_if_not_at_start(self):
        dist = self._measure_distance_from_start()
        if dist is not None and dist > self.warn_distance_m:
            self.get_logger().warn(
                f'Robot is {dist:.2f} m from the expected start pose '
                f'({self.start_x}, {self.start_y}) -- /initialpose may not match reality')

    def _reset_pose_once(self):
        self._teleport_to_start()
        self._publish_initialpose()
        self._clear_costmaps()

    def _measure_distance_from_start(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None  # no pose yet
        dx = t.transform.translation.x - self.start_x
        dy = t.transform.translation.y - self.start_y
        return math.hypot(dx, dy)

    def _verify_reset(self, max_attempts=3, tolerance_m=0.3):
        """Confirms the robot actually converged near the start pose after
        teleport + /initialpose, retrying the whole reset if not. Needed
        because the Gazebo teleport never touches the diff-drive plugin's
        own odometry integrator (pure dead-reckoning from real wheel
        rotation since the sim started) -- every circuit that drives out
        and gets teleported back, rather than actually driving back,
        leaves odom's drift from true position a little larger than
        before. That drift never resets itself, so the single /initialpose
        correction AMCL has to compute grows over the course of a long
        campaign, and can occasionally fail to converge. Confirmed by
        reading a bad run's bag directly: odom->base_footprint read
        (-18, -2) at window-open, nowhere near the start pose."""
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                self.get_logger().warn(
                    f'Pose reset attempt {attempt}/{max_attempts}: retrying '
                    f'teleport + /initialpose + costmap clear')
                self._reset_pose_once()
            dist = self._measure_distance_from_start()
            if dist is not None and dist <= tolerance_m:
                self.get_logger().info(
                    f'Pose reset confirmed within {dist:.2f}m (attempt {attempt})')
                return True, dist
        self.get_logger().error(
            f'Pose reset did NOT converge within {tolerance_m}m of the start pose '
            f'after {max_attempts} attempts -- this run is likely starting from '
            f'the wrong physical position')
        return False, dist

    def _wait_for_initialpose_subscriber(self, timeout_s=5.0):
        """This node's /initialpose publisher is brand new every run (a
        fresh short-lived process, not a long-running node) -- publishing
        before DDS discovery has actually matched it with AMCL's
        subscriber silently drops the message, since there is nobody
        connected yet to receive it. Confirmed by reproducing with
        `ros2 topic pub`, which showed several seconds of "waiting for
        subscription" even with the sim and AMCL already fully up."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.initialpose_pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn(
            f'No subscriber on /initialpose after {timeout_s}s -- the pose '
            f'reset below may be silently dropped')
        return False

    def _publish_initialpose(self):
        self._warn_if_not_at_start()
        self._wait_for_initialpose_subscriber()
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.start_x
        msg.pose.pose.position.y = self.start_y
        qx, qy, qz, qw = yaw_to_quat(self.start_yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = INITIALPOSE_COVARIANCE
        self.initialpose_pub.publish(msg)
        self.get_logger().info(f'Published /initialpose at ({self.start_x}, {self.start_y})')
        self._spin_sleep(1.0)  # let AMCL's particle filter converge before navigating

    def _clear_costmaps(self):
        """Nav2's costmaps build up obstacle/inflation data incrementally
        from sensor readings and have no idea the robot just teleported --
        right after a reset, the costmap at the robot's new footprint can
        still hold stale lethal/inflated cells from wherever it just was,
        which makes the planner reject the start pose as "in collision"
        and abort the goal almost instantly, before ever planning. Clearing
        both costmaps right after the pose reset avoids that."""
        for name, client in (('global', self.clear_global_costmap_client),
                              ('local', self.clear_local_costmap_client)):
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(
                    f'{name}_costmap clear service unavailable -- the next goal '
                    f'may abort immediately due to stale costmap data')
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is None:
                self.get_logger().warn(f'Failed to clear {name} costmap')

    def _outcome_path(self):
        out_dir = os.path.join(self.bags_dir, self.config_id, self.run_id)
        os.makedirs(out_dir, exist_ok=True)
        return os.path.join(out_dir, 'outcome.json')

    def _write_outcome(self, outcome):
        with open(self._outcome_path(), 'w') as f:
            json.dump(outcome, f, indent=2)

    def run(self):
        if not self._sync_monitor_run_id():
            self.get_logger().error(
                f'Could not push run_id={self.run_id} to safety_monitor_node -- '
                'not sending the goal, so no other run\'s summary gets overwritten')
            self._write_outcome({
                'config_id': self.config_id, 'run_id': self.run_id, 'goal': self.goal_name,
                'accepted': None, 'terminal_status': None,
                'error_msg': 'monitor run_id sync failed; goal not sent',
                'pose_reset_ok': None, 'pose_reset_distance_m': None,
            })
            return
        pose_reset_ok, pose_reset_distance_m = None, None
        if self.reset_pose:
            self._reset_pose_once()
            pose_reset_ok, pose_reset_distance_m = self._verify_reset()
        else:
            self.get_logger().info(
                'reset_pose:=false -- continuing from wherever the previous '
                'goal in this circuit left the robot, no teleport/initialpose')

        x, y, yaw = GOALS[self.goal_name]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        outcome = {
            'config_id': self.config_id, 'run_id': self.run_id,
            'goal': self.goal_name, 'goal_x': x, 'goal_y': y,
            'accepted': None, 'terminal_status': None, 'error_msg': None,
            # None when reset_pose:=false (continuing a circuit, no reset
            # attempted). False means _verify_reset's retries were
            # exhausted -- this run started from an uncontrolled position
            # and should likely be excluded from analysis.
            'pose_reset_ok': pose_reset_ok,
            'pose_reset_distance_m': pose_reset_distance_m,
        }

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            outcome['accepted'] = False
            outcome['error_msg'] = 'action server unavailable'
            self._write_outcome(outcome)
            self.get_logger().error('navigate_to_pose action server unavailable')
            return

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        outcome['accepted'] = bool(goal_handle and goal_handle.accepted)
        self._write_outcome(outcome)  # written now so a rejection is never lost

        if not outcome['accepted']:
            self.get_logger().error(f'Goal "{self.goal_name}" was rejected')
            return

        self.get_logger().info(f'Goal "{self.goal_name}" accepted, waiting for result')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.max_goal_s)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            outcome['terminal_status'] = 'timed_out'
            outcome['error_msg'] = f'no result within {self.max_goal_s}s, canceled'
        else:
            result = result_future.result()
            outcome['terminal_status'] = STATUS_NAMES.get(result.status, 'unknown')
            outcome['error_msg'] = result.result.error_msg
        self._write_outcome(outcome)
        self.get_logger().info(f'Goal "{self.goal_name}" finished: {outcome["terminal_status"]}')


def main():
    rclpy.init()
    node = SendGoal()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
