#!/usr/bin/env python3
"""Publishes the robot pose in the map frame on /pose_in.

Keeps zone_detector and dynamic_risk_constrainer independent of where the
pose comes from. On another platform only this node has to be replaced.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


class PoseAdapter(Node):

    def __init__(self):
        super().__init__('pose_adapter')
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_footprint').value
        rate = self.declare_parameter('rate_hz', 20.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/pose_in', 10)
        self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        try:
            # Time() = latest available transform
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except TransformException as e:
            # Publish nothing rather than a stale pose; consumers check the stamp.
            self.get_logger().warn(f'No transform yet: {e}', throttle_duration_sec=5.0)
            return
        msg = PoseStamped()
        msg.header = t.header                      # frame_id = map, stamp = transform time
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.orientation = t.transform.rotation
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PoseAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
