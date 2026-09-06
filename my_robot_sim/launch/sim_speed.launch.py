import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ---------------------------------------------------------
    # Package directories
    # ---------------------------------------------------------

    package_dir = get_package_share_directory('my_robot_sim')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # ---------------------------------------------------------
    # Map and Gazebo world
    # ---------------------------------------------------------

    map_file = os.path.join(
        package_dir,
        'maps',
        'map.yaml'
    )

    world_file = os.path.join(
        package_dir,
        'worlds',
        'world.sdf'
    )

    # ---------------------------------------------------------
    # Nav2 parameters (speed-limit zone)
    # ---------------------------------------------------------

    params_file = os.path.join(
        package_dir,
        'config',
        'nav2_params_speed.yaml'
    )

    # ---------------------------------------------------------
    # Robot starting position
    # ---------------------------------------------------------

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial X position of the robot'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial Y position of the robot'
    )

    declare_yaw = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial orientation of the robot'
    )

    # ---------------------------------------------------------
    # Nav2 + Gazebo + TurtleBot3
    # ---------------------------------------------------------

    nav2_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_dir,
                'launch',
                'tb3_simulation_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'world': world_file,
            'params_file': params_file,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
            'headless': 'False',
            'use_rviz': 'True',
            'use_simulator': 'True',
            'use_robot_state_pub': 'True',
        }.items()
    )

    # ---------------------------------------------------------
    # Speed filter mask + info server
    # ---------------------------------------------------------

    speed_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='speed_mask_server',
        output='screen',
        parameters=[
            {
                # Reuses the keepout experiment's mask directly (same y=3
                # boundary, by design) instead of a duplicate copy, so the
                # two zones can never silently drift apart.
                'yaml_filename': os.path.join(
                    package_dir,
                    'maps',
                    'keepout_mask.yaml'
                ),
                'use_sim_time': True,
            }
        ],
        remappings=[
            ('/map', '/speed_mask'),
            ('/map_metadata', '/speed_mask_metadata'),
        ],
    )
    
#Relative speed limit
    '''speed_costmap_filter_info_server = Node(
         package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='speed_costmap_filter_info_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'type': 1,
                'filter_info_topic': '/speed_filter_info',
                'mask_topic': '/speed_mask',
                'base': 0.0,
                'multiplier': 0.5, # 50% of normal speed of turtlebot
            }
        ],
    )'''
    
    speed_costmap_filter_info_server = Node(
         package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='speed_costmap_filter_info_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'type': 2,
                'filter_info_topic': '/speed_filter_info',
                'mask_topic': '/speed_mask',
                'base': 0.0,
                'multiplier': 0.002,
            }
        ],
    )


    speed_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_speed',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'speed_mask_server',
                    'speed_costmap_filter_info_server',
                ],
            }
        ],
    )

    # ---------------------------------------------------------
    # Zone-based acceleration limiting (custom node — Nav2 has
    # no built-in filter for this, only speed)
    # ---------------------------------------------------------

    zone_accel_limiter = Node(
        package='my_robot_sim',
        executable='zone_accel_limiter.py',
        name='zone_accel_limiter',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'default_max_accel': [2.5, 0.0, 3.2],
                'default_max_decel': [-2.5, 0.0, -3.2],
                'zone_max_accel': [1.25, 0.0, 1.6],
                'zone_max_decel': [-1.25, 0.0, -1.6],
                'velocity_smoother_node': 'velocity_smoother',
            }
        ],
    )

    # ---------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------

    return LaunchDescription([
        declare_x_pose,
        declare_y_pose,
        declare_yaw,
        nav2_simulation,
        speed_mask_server,
        speed_costmap_filter_info_server,
        speed_lifecycle_manager,
        zone_accel_limiter,
    ])
