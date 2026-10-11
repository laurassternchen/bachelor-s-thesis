import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_dir = get_package_share_directory('my_robot_sim')
    nav2_dir = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(package_dir, 'maps', 'map.yaml')
    world_file = os.path.join(package_dir, 'worlds', 'world.sdf')
    params_file = os.path.join(package_dir, 'config', 'drc', 'nav2_params.yaml')
    zone_map_file = os.path.join(package_dir, 'maps', 'zone_map.yaml')

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    declare_headless = DeclareLaunchArgument('headless', default_value='False')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='True')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

    nav2_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'world': world_file,
            'params_file': params_file,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
            'headless': headless,
            'use_rviz': use_rviz,
            'use_simulator': 'True',
            'use_robot_state_pub': 'True',
        }.items()
    )

    # Hidden ground truth: Nav2 never subscribes to /zone_truth.
    zone_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='zone_map_server',
        output='screen',
        parameters=[{'yaml_filename': zone_map_file, 'use_sim_time': True}],
        remappings=[('/map', '/zone_truth'),
                    ('/map_metadata', '/zone_truth_metadata')],
    )

    zone_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_zones',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'autostart': True,
                     'node_names': ['zone_map_server']}],
    )

    pose_adapter = Node(
        package='my_robot_sim',
        executable='pose_adapter.py',
        name='pose_adapter',
        output='screen',
        parameters=[{'use_sim_time': True, 'rate_hz': 20.0}],
    )

    zone_detector = Node(
        package='my_robot_sim',
        executable='zone_detector.py',
        name='zone_detector',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'detection_range': 3.0,
                     'keepout_min_value': 100,
                     'point_height': 0.3}],
    )

    dynamic_risk_constrainer = Node(
        package='my_robot_sim',
        executable='dynamic_risk_constrainer.py',
        name='dynamic_risk_constrainer',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'zone_max_linear_vel': 0.2,
                     'zone_max_angular_vel': 2.0,
                     'zone_max_accel': [1.25, 1.6],
                     'zone_max_decel': [1.25, 1.6],
                     'pose_timeout': 0.5}],
    )

    return LaunchDescription([
        declare_x_pose,
        declare_y_pose,
        declare_yaw,
        declare_headless,
        declare_use_rviz,
        nav2_simulation,
        zone_map_server,
        zone_lifecycle_manager,
        pose_adapter,
        zone_detector,
        dynamic_risk_constrainer,
    ])