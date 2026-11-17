#!/usr/bin/env python3
"""Runs a fixed sequence of nav goals repeatedly and flags abnormal behavior.

Cycles through GOALS (a mix of free/speed-zone/keepout-zone targets) for
`num_trials` total goal-sends, without restarting the simulation between
trials. Publishes one diagnostic_msgs/DiagnosticStatus per trial on
/experiment_diagnostics -- view it in Lichtblick's Diagnostics panel to see
anomalies (WARN/ERROR) highlighted automatically, instead of combing through
logs by hand.

Run with the main simulation already up and localized:
    ros2 run my_robot_sim experiment_runner.py
"""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

REQUIRED_NODES = ['pose_adapter', 'zone_detector', 'dynamic_risk_constrainer']

# (name, x, y, yaw_deg, zone_type, expect_success)
GOALS = [
    ('free_1',    5.0, -5.0,   0.0, 'free',    True),
    ('speed_1',   3.0,  4.0,   0.0, 'speed',   True),
    ('free_2',    0.0, -8.0,   0.0, 'free',    True),
    ('keepout_1', -2.0, 6.0,   0.0, 'keepout', False),
]


def yaw_to_quat(deg):
    half = math.radians(deg) / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


class ExperimentRunner(Node):

    def __init__(self):
        super().__init__('experiment_runner')
        self.num_trials = self.declare_parameter('num_trials', 20).value
        self.pose_jump_m = self.declare_parameter('pose_jump_threshold_m', 0.4).value
        self.pose_jump_window_s = self.declare_parameter('pose_jump_window_s', 1.0).value
        self.max_goal_s = self.declare_parameter('max_goal_duration_s', 60.0).value

        self.diag_pub = self.create_publisher(DiagnosticArray, '/experiment_diagnostics', 10)
        self.create_subscription(PoseStamped, '/pose_in', self._on_pose, 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._last_pose = None   # (stamp_sec, x, y)
        self._jumps = []         # jumps seen during the current trial

    def _on_pose(self, msg: PoseStamped):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        x, y = msg.pose.position.x, msg.pose.position.y
        if self._last_pose is not None:
            pt, px, py = self._last_pose
            dt = t - pt
            if 0 < dt <= self.pose_jump_window_s:
                dist = math.hypot(x - px, y - py)
                if dist > self.pose_jump_m:
                    self._jumps.append((t, dist, dt))
        self._last_pose = (t, x, y)

    def _nodes_alive(self):
        names = {n for n, _ in self.get_node_names_and_namespaces()}
        missing = [n for n in REQUIRED_NODES if n not in names]
        return missing

    def _publish_trial(self, trial_idx, goal_name, level, message, values):
        status = DiagnosticStatus()
        status.level = level
        status.name = f'experiment_runner: trial {trial_idx} ({goal_name})'
        status.message = message
        status.hardware_id = f'my_robot_sim/trial_{trial_idx}'
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self.diag_pub.publish(arr)
        self.get_logger().info(f'[trial {trial_idx}] {goal_name}: {message}')

    def _send_goal(self, name, x, y, yaw_deg):
        qx, qy, qz, qw = yaw_to_quat(yaw_deg)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            return None, 'action server unavailable', 0

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None, 'goal rejected', 0

        result_future = goal_handle.get_result_async()
        start = time.monotonic()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.max_goal_s)
        elapsed = time.monotonic() - start

        if not result_future.done():
            goal_handle.cancel_goal_async()
            return GoalStatus.STATUS_ABORTED, 'timed out, canceled', elapsed

        result = result_future.result()
        return result.status, result.result.error_msg, elapsed

    def run(self):
        self.get_logger().info(f'Starting experiment: {self.num_trials} trials, '
                                f'{len(GOALS)} goals in rotation')

        # Grace period: DDS discovery can take a few seconds after a fresh
        # launch, so don't fail on the very first instant -- retry briefly
        # before concluding a node is genuinely missing.
        missing = self._nodes_alive()
        retry_deadline = time.monotonic() + 10.0
        while missing and time.monotonic() < retry_deadline:
            time.sleep(1.0)
            missing = self._nodes_alive()
        if missing:
            self._publish_trial(0, 'startup', DiagnosticStatus.ERROR,
                                 f'missing nodes after 10s grace period: {missing}', {})
            return

        ok = warn = error = 0
        for i in range(1, self.num_trials + 1):
            name, x, y, yaw, zone_type, expect_success = GOALS[(i - 1) % len(GOALS)]

            missing = self._nodes_alive()
            if missing:
                # One quick re-check to rule out a momentary discovery
                # blip -- deliberately NOT a long grace period here, since
                # this check exists to catch real node deaths mid-campaign.
                time.sleep(1.0)
                missing = self._nodes_alive()
            if missing:
                self._publish_trial(i, name, DiagnosticStatus.ERROR,
                                     f'ABORTING CAMPAIGN: missing nodes: {missing}', {})
                self.get_logger().error('Stopping -- required nodes died mid-campaign.')
                break

            self._jumps = []
            status, error_msg, elapsed = self._send_goal(name, x, y, yaw)
            succeeded = (status == GoalStatus.STATUS_SUCCEEDED)

            values = {
                'zone_type': zone_type,
                'expected_success': expect_success,
                'actual_succeeded': succeeded,
                'duration_s': f'{elapsed:.1f}',
                'pose_jumps': len(self._jumps),
                'error_msg': error_msg,
            }

            if succeeded != expect_success:
                level, msg = DiagnosticStatus.ERROR, (
                    f'UNEXPECTED RESULT: expected '
                    f'{"success" if expect_success else "failure"}, got '
                    f'{"success" if succeeded else "failure"}')
                error += 1
            elif self._jumps:
                level, msg = DiagnosticStatus.WARN, (
                    f'pose jump(s) detected during trial: {self._jumps}')
                warn += 1
            elif elapsed > self.max_goal_s * 0.8:
                level, msg = DiagnosticStatus.WARN, f'slow trial ({elapsed:.1f}s)'
                warn += 1
            else:
                level, msg = DiagnosticStatus.OK, 'nominal'
                ok += 1

            self._publish_trial(i, name, level, msg, values)

        self._publish_trial(
            self.num_trials, 'SUMMARY',
            DiagnosticStatus.OK if error == 0 else DiagnosticStatus.ERROR,
            f'{ok} ok, {warn} warn, {error} error (of {self.num_trials} trials)',
            {'ok': ok, 'warn': warn, 'error': error})


def main():
    rclpy.init()
    node = ExperimentRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
