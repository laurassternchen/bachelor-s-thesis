#!/usr/bin/env python3
"""Simulated zone perception.

Reads the hidden ground-truth zone map (/zone_truth) and reveals a zone cell
only once it is within detection_range of the robot. Revealed cells are
remembered. Outputs:
  /zone_obstacles  PointCloud2  detected keepout cells (for costmaps + collision_monitor)
  /detected_zones  OccupancyGrid  all detected zones (for the constrainer)

Zone map convention: 0 = free, 1..99 = speed zone, 100 = keepout.
Assumes the zone map origin has no rotation.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

# map_server publishes once; without TRANSIENT_LOCAL the message is never received
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class ZoneDetector(Node):

    def __init__(self):
        super().__init__('zone_detector')
        self.detection_range = self.declare_parameter('detection_range', 3.0).value
        self.keepout_min = self.declare_parameter('keepout_min_value', 100).value
        self.point_height = self.declare_parameter('point_height', 0.3).value
        rate = self.declare_parameter('rate_hz', 10.0).value

        self.info = None     # grid metadata
        self.truth = None    # 2D array: true zone value per cell
        self.known = None    # 2D bool array: cells detected so far (the memory)
        self.cx = self.cy = None   # map coordinates of every cell centre
        self.pose = None

        self.create_subscription(OccupancyGrid, '/zone_truth', self.on_truth, LATCHED)
        self.create_subscription(PoseStamped, '/pose_in', self.on_pose, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/detected_zones', LATCHED)
        self.cloud_pub = self.create_publisher(PointCloud2, '/zone_obstacles', 10)
        self.create_timer(1.0 / rate, self.tick)

    def on_truth(self, msg):
        self.info = msg.info
        h, w, res = msg.info.height, msg.info.width, msg.info.resolution
        self.truth = np.array(msg.data, dtype=np.int16).reshape(h, w)   # row 0 = bottom
        self.known = np.zeros((h, w), dtype=bool)
        xs = msg.info.origin.position.x + (np.arange(w) + 0.5) * res
        ys = msg.info.origin.position.y + (np.arange(h) + 0.5) * res
        self.cx, self.cy = np.meshgrid(xs, ys)
        self.get_logger().info(f'Zone truth received: {w}x{h} cells, '
                               f'{int((self.truth > 0).sum())} zone cells')
        self.publish_grid()

    def on_pose(self, msg):
        self.pose = msg

    def tick(self):
        if self.truth is None or self.pose is None:
            return
        x, y = self.pose.pose.position.x, self.pose.pose.position.y
        in_range = (self.cx - x) ** 2 + (self.cy - y) ** 2 <= self.detection_range ** 2
        newly = in_range & (self.truth > 0) & ~self.known
        if newly.any():
            self.known |= newly
            self.get_logger().info(f'Detected {int(newly.sum())} new zone cells')
            self.publish_grid()
        # Published every tick: the costmaps re-mark these cells on each update,
        # because laser clearing rays pass straight through a zone.
        self.publish_cloud()

    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.info
        msg.data = np.where(self.known, self.truth, 0).astype(np.int8).flatten().tolist()
        self.grid_pub.publish(msg)

    def publish_cloud(self):
        rows, cols = np.nonzero(self.known & (self.truth >= self.keepout_min))
        pts = np.column_stack((self.cx[rows, cols], self.cy[rows, cols],
                               np.full(rows.size, self.point_height))).astype(np.float32)
        # Stamp with the pose time so TF for this stamp is guaranteed to exist.
        header = Header(frame_id='map', stamp=self.pose.header.stamp)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))


def main():
    rclpy.init()
    node = ZoneDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
