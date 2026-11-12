"""Launches the passive SafetyMonitor alongside any of the four sim configs.

Never included by my_robot_sim's own launch files -- run it as a second,
independent `ros2 launch` alongside sim_baseline.launch.py / sim_keepout
.launch.py / sim_speed.launch.py / drc/sim.launch.py, so the monitor stays
provably separate from the system under test.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    monitor_share = get_package_share_directory('safety_monitor')
    robot_sim_share = get_package_share_directory('my_robot_sim')

    default_params_file = os.path.join(monitor_share, 'config', 'safety_monitor.yaml')
    default_zone_map_yaml = os.path.join(robot_sim_share, 'maps', 'zone_map.yaml')
    default_safety_case_yaml = os.path.join(monitor_share, 'config', 'safety_case.yaml')

    config_id = LaunchConfiguration('config_id')
    run_id = LaunchConfiguration('run_id')
    params_file = LaunchConfiguration('params_file')
    zone_map_yaml = LaunchConfiguration('zone_map_yaml')
    safety_case_yaml = LaunchConfiguration('safety_case_yaml')
    bags_dir = LaunchConfiguration('bags_dir')
    pose_source = LaunchConfiguration('pose_source')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spi_log_topic = LaunchConfiguration('spi_log_topic')
    spi_status_topic = LaunchConfiguration('spi_status_topic')

    declare_args = [
        DeclareLaunchArgument('config_id', default_value='base'),
        DeclareLaunchArgument('run_id', default_value='run0'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('zone_map_yaml', default_value=default_zone_map_yaml),
        DeclareLaunchArgument('safety_case_yaml', default_value=default_safety_case_yaml),
        DeclareLaunchArgument('bags_dir', default_value='bags'),
        # "gt" is parameter-ready in safety_monitor_node but not wired up to
        # a ros_gz_bridge yet -- see README's limitations section.
        DeclareLaunchArgument('pose_source', default_value='tf'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Remapped for offline replay (analysis/replay_bags.py) so the monitor's
        # output never collides with the /spi_log already stored in the bag.
        DeclareLaunchArgument('spi_log_topic', default_value='/spi_log'),
        DeclareLaunchArgument('spi_status_topic', default_value='/spi_status'),
    ]

    safety_monitor_node = Node(
        package='safety_monitor',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[
            params_file,
            {
                'config_id': config_id,
                'run_id': run_id,
                'zone_map_yaml': zone_map_yaml,
                'safety_case_yaml': safety_case_yaml,
                'bags_dir': bags_dir,
                'pose_source': pose_source,
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            ('/spi_log', spi_log_topic),
            ('/spi_status', spi_status_topic),
        ],
    )

    return LaunchDescription(declare_args + [safety_monitor_node])
