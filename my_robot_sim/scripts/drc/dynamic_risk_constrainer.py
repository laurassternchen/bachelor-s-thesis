#!/usr/bin/env python3
"""Clamps speed and acceleration while the robot is inside a detected speed zone.

Sits in the command chain: cmd_vel_smoothed -> [this node] -> cmd_vel_safe.
Outside a zone commands pass through unchanged.

NOTE on the message type: check `ros2 topic info /cmd_vel_smoothed`.
If it is geometry_msgs/msg/Twist (not TwistStamped), set STAMPED = False.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

STAMPED = False
CmdMsg = TwistStamped if STAMPED else Twist

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DynamicRiskConstrainer(Node):

    def __init__(self):
        super().__init__('dynamic_risk_constrainer')
        p = self.declare_parameter
        self.v_max = p('zone_max_linear_vel', 0.2).value
        self.w_max = p('zone_max_angular_vel', 2.0).value
        self.accel = p('zone_max_accel', [1.25, 1.6]).value    # [linear, angular]
        self.decel = p('zone_max_decel', [1.25, 1.6]).value    # magnitudes
        self.pose_timeout = p('pose_timeout', 0.5).value

        self.grid = None
        self.pose = None
        self.last = [0.0, 0.0]      # last published [v, w]
        self.last_time = None
        self.in_zone = None

        self.create_subscription(OccupancyGrid, '/detected_zones', self.on_grid, LATCHED)
        self.create_subscription(PoseStamped, '/pose_in', self.on_pose, 10)
        self.create_subscription(CmdMsg, 'cmd_vel_smoothed', self.on_cmd, 10)
        self.pub = self.create_publisher(CmdMsg, 'cmd_vel_safe', 10)
        self.zone_pub = self.create_publisher(Bool, '/constrainer/in_zone', LATCHED)

    def on_grid(self, msg):
        self.grid = msg

    def on_pose(self, msg):
        self.pose = msg

    def limits_apply(self):
        """True if the robot is in a zone, or if we cannot tell (fail-safe)."""
        if self.grid is None or self.pose is None:
            self.get_logger().warn('No zone grid or pose yet: applying zone limits',
                                   throttle_duration_sec=5.0)
            return True
        age = (self.get_clock().now() - Time.from_msg(self.pose.header.stamp)).nanoseconds * 1e-9
        if age > self.pose_timeout:
            self.get_logger().warn(f'Pose is {age:.2f}s old: applying zone limits',
                                   throttle_duration_sec=5.0)
            return True
        info = self.grid.info
        col = int((self.pose.pose.position.x - info.origin.position.x) / info.resolution)
        row = int((self.pose.pose.position.y - info.origin.position.y) / info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height):
            return True                                   # outside the map: conservative
        return self.grid.data[row * info.width + col] != 0

    @staticmethod
    def rate_limit(target, prev, accel, decel, dt):
        rate = accel if abs(target) > abs(prev) else decel
        return clamp(target, prev - rate * dt, prev + rate * dt)

    def on_cmd(self, msg):
        twist = msg.twist if STAMPED else msg
        now = self.get_clock().now()
        dt = 0.05 if self.last_time is None else (now - self.last_time).nanoseconds * 1e-9
        dt = clamp(dt, 0.001, 0.2)      # guards against pauses and clock jumps
        self.last_time = now

        limited = self.limits_apply()
        v, w = twist.linear.x, twist.angular.z
        if limited:
            # Design decision: on zone entry above the limit we ramp down at
            # zone_max_decel instead of stepping, so the robot is briefly above
            # the limit inside the zone. A lookahead would remove this.
            v = self.rate_limit(clamp(v, -self.v_max, self.v_max),
                                self.last[0], self.accel[0], self.decel[0], dt)
            w = self.rate_limit(clamp(w, -self.w_max, self.w_max),
                                self.last[1], self.accel[1], self.decel[1], dt)
        self.last = [v, w]              # tracked outside the zone too: no jump at entry
        twist.linear.x, twist.angular.z = v, w
        self.pub.publish(msg)

        if limited != self.in_zone:
            self.in_zone = limited
            self.zone_pub.publish(Bool(data=limited))
            self.get_logger().info(f'{"Entering" if limited else "Leaving"} zone at v={v:.2f} m/s')


def main():
    rclpy.init()
    node = DynamicRiskConstrainer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()