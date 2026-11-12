#!/usr/bin/env python3
"""Passive SafetyMonitor: subscribe-only, publishes /spi_log (+ a status log).

Observes the same handful of standard topics regardless of which of the
four Nav2 configurations is under test (base / native_keepout /
native_speed / external) and classifies zones from its own copy of
zone_map.pgm, so it adds nothing to -- and depends on nothing specific
to -- the system under test (RQ1). SPI evaluation is scoped to the
window between a goal being accepted and reaching a terminal status;
missing pose during that window is tracked separately as "no_pose" time,
never counted as a violation.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from safety_monitor_msgs.msg import SafetyEvent

from .safety_case import SafetyCase
from .spis import SPI1Overspeed, SPI2Keepout, SPI3Clearance, SPI4Intervention
from .zone_mask import ZoneMask

# Written into monitor_summary.json so analyze_runs.py can tell new runs
# (stamps = first/last violating sample) from older ones (end = publish time).
EVENT_STAMP_CONVENTION = 'first_last_violating_sample'

TERMINAL_STATUSES = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
}

# A goal server can transition ACCEPTED -> EXECUTING fast enough that a
# separate subscriber never observes a status message showing plain
# ACCEPTED -- only EXECUTING ever arrives. Any of these three counts as
# "there is now a tracked, not-yet-finished goal" for window-opening
# purposes.
ACTIVE_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


class SafetyMonitorNode(Node):

    @property
    def config_id(self):
        return self.get_parameter('config_id').value

    @property
    def run_id(self):
        return self.get_parameter('run_id').value

    def __init__(self):
        super().__init__('safety_monitor_node')
        p = self.declare_parameter

        # Not cached: a long-lived monitor session covers many goal-sends,
        # and send_goal.py pushes a fresh config_id/run_id via this node's
        # own /set_parameters service right before each one, so these must
        # always be read live (see the config_id/run_id properties below).
        p('config_id', 'base')
        p('run_id', 'run0')
        self.bags_dir = p('bags_dir', 'bags').value
        zone_map_yaml = p('zone_map_yaml', '').value
        self.global_frame = p('global_frame', 'map').value
        self.robot_frame = p('robot_frame', 'base_footprint').value
        self.pose_source = p('pose_source', 'tf').value
        gt_pose_topic = p('gt_pose_topic', '/ground_truth_pose').value
        tick_hz = p('tick_hz', 20.0).value
        self.clear_after_s = p('clear_after_s', 0.3).value
        safety_case_yaml = p('safety_case_yaml', '').value

        if not zone_map_yaml:
            raise RuntimeError('the zone_map_yaml parameter is required')
        with open(zone_map_yaml) as f:
            image_name = yaml.safe_load(f)['image']
        zone_map_pgm = os.path.join(os.path.dirname(zone_map_yaml), image_name)
        self.zone_mask = ZoneMask.from_files(zone_map_pgm, zone_map_yaml)

        # Thresholds and SPI -> goal mapping come only from the safety case.
        if not safety_case_yaml:
            raise RuntimeError('the safety_case_yaml parameter is required')
        case = SafetyCase.from_file(safety_case_yaml)
        cl = self.clear_after_s
        self.spi1 = SPI1Overspeed(case.claim_id('SPI-1'), case.threshold('SPI-1'), cl)
        self.spi2 = SPI2Keepout(case.claim_id('SPI-2'), cl, threshold=case.threshold('SPI-2'))
        self.spi3 = SPI3Clearance(case.claim_id('SPI-3'), case.threshold('SPI-3'), cl)
        self.spi4 = SPI4Intervention(case.claim_id('SPI-4'), case.threshold('SPI-4'), cl)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_gt_pose = None

        self.latest_speed_mps = 0.0
        self.latest_cmd_vel_smoothed = None

        self.window_active = False
        self.active_goal_id = None
        self.window_config_id = self.config_id
        self.window_run_id = self.run_id
        self.window_start_time = None
        self.no_pose_time = 0.0
        self.last_valid_pos = None
        self.path_length = 0.0
        self.tick_period = 1.0 / tick_hz
        self.spi_event_counts = {}
        # Wall-clock processing time of each check cycle, per window.
        self.tick_proc_s = []
        self.spi4_proc_s = []

        self.event_pub = self.create_publisher(SafetyEvent, '/spi_log', 10)
        self.status_pub = self.create_publisher(DiagnosticStatus, '/spi_status', 10)

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self._on_cmd_vel_smoothed, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self._on_goal_status, 10)
        if self.pose_source == 'gt':
            self.create_subscription(PoseStamped, gt_pose_topic, self._on_gt_pose, 10)

        self.create_timer(self.tick_period, self._tick)

        self.get_logger().info(
            f'safety_monitor_node ready: config_id={self.config_id} run_id={self.run_id} '
            f'pose_source={self.pose_source}')

    # -- pose -----------------------------------------------------------
    def _on_gt_pose(self, msg):
        self.latest_gt_pose = msg

    def _current_position(self):
        if self.pose_source == 'gt':
            if self.latest_gt_pose is None:
                return None
            pos = self.latest_gt_pose.pose.position
            return (pos.x, pos.y)
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    # -- subscriptions ----------------------------------------------------
    def _on_odom(self, msg):
        self.latest_speed_mps = msg.twist.twist.linear.x

    def _on_cmd_vel_smoothed(self, msg):
        self.latest_cmd_vel_smoothed = msg

    def _on_cmd_vel(self, msg):
        if not self.window_active or self.latest_cmd_vel_smoothed is None:
            return
        started = time.perf_counter()
        pos = self._current_position()
        if pos is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        ev = self.spi4.update(t, pos[0], pos[1],
                              self.latest_cmd_vel_smoothed.linear.x, msg.linear.x)
        if ev is not None:
            self._publish_event(self.spi4, ev)
        self.spi4_proc_s.append(time.perf_counter() - started)

    def _tick(self):
        if not self.window_active:
            return
        started = time.perf_counter()
        pos = self._current_position()
        if pos is None:
            self.no_pose_time += self.tick_period
            self.tick_proc_s.append(time.perf_counter() - started)
            return
        x, y = pos
        if self.last_valid_pos is not None:
            dx, dy = x - self.last_valid_pos[0], y - self.last_valid_pos[1]
            self.path_length += (dx ** 2 + dy ** 2) ** 0.5
        self.last_valid_pos = (x, y)

        t = self.get_clock().now().nanoseconds * 1e-9
        for spi in (self.spi1, self.spi2, self.spi3):
            if spi is self.spi1:
                ev = spi.update(t, x, y, self.latest_speed_mps, self.zone_mask)
            else:
                ev = spi.update(t, x, y, self.zone_mask)
            if ev is not None:
                self._publish_event(spi, ev)
        self._publish_status()
        self.tick_proc_s.append(time.perf_counter() - started)

    def _publish_status(self):
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = 'safety_monitor'
        status.message = 'window active' if self.window_active else 'idle'
        status.hardware_id = f'{self.config_id}/{self.run_id}'
        status.values = [
            KeyValue(key='no_pose_time_s', value=f'{self.no_pose_time:.2f}'),
            KeyValue(key='path_length_m', value=f'{self.path_length:.2f}'),
            KeyValue(key='spi_event_counts', value=str(self.spi_event_counts)),
        ]
        self.status_pub.publish(status)

    # -- goal window ------------------------------------------------------
    def _on_goal_status(self, msg):
        # Two passes, not one: a single status message can contain both the
        # active goal's terminal entry and the next goal's accepted/executing
        # entry (the array's order between them is not guaranteed). Closing
        # first, against whichever goal_id was active *before* this message,
        # means a new window can never overwrite active_goal_id ahead of the
        # old window's close being recognised -- a one-pass loop could open
        # the new window first and then fail to match the old goal's close.
        for status in msg.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)
            if (status.status in TERMINAL_STATUSES and goal_id == self.active_goal_id
                    and self.window_active):
                self._end_window(status.status)
        for status in msg.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)
            if status.status in ACTIVE_STATUSES and goal_id != self.active_goal_id:
                self._start_window(goal_id)

    def _start_window(self, goal_id):
        self.active_goal_id = goal_id
        self.window_active = True
        # Snapshotted once, here, rather than read live at write-time: a
        # window must stay anchored to the run it started under, even if
        # send_goal.py's /set_parameters push for the *next* run arrives
        # (a separate channel from this goal's own terminal-status message)
        # before this window actually closes -- otherwise the previous
        # run's summary gets written under the new run_id and is silently
        # overwritten once that new run's own window closes.
        self.window_config_id = self.config_id
        self.window_run_id = self.run_id
        self.window_start_time = self.get_clock().now().nanoseconds * 1e-9
        self.no_pose_time = 0.0
        self.path_length = 0.0
        self.last_valid_pos = None
        # Pre-populated with all four so a SPI that never fires still shows
        # 0 explicitly, instead of being silently absent from the summary.
        self.spi_event_counts = {spi.spi_id: 0 for spi in (self.spi1, self.spi2, self.spi3, self.spi4)}
        self.tick_proc_s = []
        self.spi4_proc_s = []
        self.get_logger().info('Goal accepted: SPI evaluation window open')

    def _end_window(self, terminal_status):
        now = self.get_clock().now().nanoseconds * 1e-9
        for spi in (self.spi1, self.spi2, self.spi3, self.spi4):
            ev = spi.finalize(now)
            if ev is not None:
                self._publish_event(spi, ev)
        self.window_active = False
        self._write_summary(terminal_status, now)
        self.get_logger().info('Goal reached a terminal status: SPI evaluation window closed')

    # -- output -------------------------------------------------------------
    @staticmethod
    def _stamp(t_s):
        sec = math.floor(t_s)
        return TimeMsg(sec=int(sec), nanosec=min(int(round((t_s - sec) * 1e9)), 999_999_999))

    def _publish_event(self, spi, ev):
        # The event spans its first to last violating sample, not the
        # (>= clear_after_s later) moment the debouncer released it.
        msg = SafetyEvent()
        msg.header.stamp = self._stamp(ev['last_violating_stamp'])
        msg.header.frame_id = self.global_frame
        msg.start_stamp = self._stamp(ev['start_stamp'])
        msg.spi_id = spi.spi_id
        msg.claim_id = spi.claim_id
        msg.zone_id = ev.get('zone_id', '')
        msg.zone_type = ev.get('zone_type', '')
        msg.peak_value = float(ev['peak_value'])
        msg.threshold = float(spi.threshold)
        msg.severity = (SafetyEvent.SEVERITY_INFO if spi.spi_id == 'SPI-4'
                        else SafetyEvent.SEVERITY_VIOLATION)
        msg.config_id = self.window_config_id
        msg.run_id = self.window_run_id
        px, py = ev['peak_position']
        msg.position = Point(x=float(px), y=float(py), z=0.0)
        self.event_pub.publish(msg)

        self.spi_event_counts[spi.spi_id] = self.spi_event_counts.get(spi.spi_id, 0) + 1
        self.get_logger().info(
            f'{spi.spi_id} event: peak={msg.peak_value:.3f} (thr={msg.threshold:.3f}) '
            f'zone={msg.zone_id}')

    def _write_summary(self, terminal_status, end_time):
        out_dir = os.path.join(self.bags_dir, self.window_config_id, self.window_run_id)
        os.makedirs(out_dir, exist_ok=True)
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }.get(terminal_status, 'unknown')
        summary = {
            'config_id': self.window_config_id,
            'run_id': self.window_run_id,
            'terminal_status': status_name,
            'window_start_time': self.window_start_time,  # sim-time seconds, matches bag stamps
            'window_end_time': end_time,
            'duration_s': end_time - self.window_start_time if self.window_start_time else None,
            'path_length_m': self.path_length,
            'no_pose_time_s': self.no_pose_time,
            'spi_event_counts': self.spi_event_counts,
            'event_stamp_convention': EVENT_STAMP_CONVENTION,
            'clear_after_s': self.clear_after_s,
            'processing_time_ms': {
                'tick': self._timing_stats(self.tick_proc_s),
                'spi4_callback': self._timing_stats(self.spi4_proc_s),
            },
        }
        path = os.path.join(out_dir, 'monitor_summary.json')
        with open(path, 'w') as f:
            json.dump(summary, f, indent=2)
        self.get_logger().info(f'Wrote run summary to {path}')

    @staticmethod
    def _timing_stats(samples_s):
        if not samples_s:
            return {'n': 0, 'mean': None, 'p95': None, 'max': None}
        ms = np.asarray(samples_s) * 1e3
        return {'n': int(ms.size), 'mean': float(ms.mean()),
                'p95': float(np.percentile(ms, 95)), 'max': float(ms.max())}

    def destroy_node(self):
        # Fallback only: the goal-result path above is the primary trigger.
        if self.window_active:
            self._end_window(GoalStatus.STATUS_UNKNOWN)
        super().destroy_node()


def main():
    rclpy.init()
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
