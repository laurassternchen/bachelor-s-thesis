#!/usr/bin/env python3
"""Dynamically caps velocity_smoother's accel/decel while inside a speed-limited zone.

Nav2 has no built-in costmap filter for zone-based acceleration limits (only
speed, via SpeedFilter). This node fills that gap by watching /speed_limit
(already published by SpeedFilter) and reacting to it: a nonzero value means
the robot is inside the zone, 0.0 means no restriction applies (the same
convention SpeedFilter itself uses).
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from nav2_msgs.msg import SpeedLimit


class ZoneAccelLimiter(Node):

    def __init__(self):
        super().__init__('zone_accel_limiter')

        self.declare_parameter('default_max_accel', [2.5, 0.0, 3.2])
        self.declare_parameter('default_max_decel', [-2.5, 0.0, -3.2])
        self.declare_parameter('zone_max_accel', [1.25, 0.0, 1.6])
        self.declare_parameter('zone_max_decel', [-1.25, 0.0, -1.6])
        self.declare_parameter('velocity_smoother_node', 'velocity_smoother')

        self.default_max_accel = self.get_parameter('default_max_accel').value
        self.default_max_decel = self.get_parameter('default_max_decel').value
        self.zone_max_accel = self.get_parameter('zone_max_accel').value
        self.zone_max_decel = self.get_parameter('zone_max_decel').value
        vs_node = self.get_parameter('velocity_smoother_node').value

        self.in_zone = False

        self.client = self.create_client(
            SetParameters, f'/{vs_node}/set_parameters'
        )

        self.create_subscription(
            SpeedLimit, '/speed_limit', self.speed_limit_callback, 10
        )

        self.get_logger().info(
            f'zone_accel_limiter ready, watching /speed_limit, '
            f'targeting /{vs_node}/set_parameters'
        )

    def speed_limit_callback(self, msg: SpeedLimit):
        now_in_zone = msg.speed_limit != 0.0
        if now_in_zone == self.in_zone:
            return

        self.in_zone = now_in_zone
        accel = self.zone_max_accel if now_in_zone else self.default_max_accel
        decel = self.zone_max_decel if now_in_zone else self.default_max_decel

        self.get_logger().info(
            f'{"Entering" if now_in_zone else "Exiting"} zone -> '
            f'max_accel={accel}, max_decel={decel}'
        )
        self._set_params(accel, decel)

    def _set_params(self, accel, decel):
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                'velocity_smoother set_parameters service unavailable'
            )
            return

        request = SetParameters.Request()
        for name, value in (('max_accel', accel), ('max_decel', decel)):
            request.parameters.append(
                Parameter(
                    name=name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                        double_array_value=value,
                    ),
                )
            )
        self.client.call_async(request)


def main():
    rclpy.init()
    node = ZoneAccelLimiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
