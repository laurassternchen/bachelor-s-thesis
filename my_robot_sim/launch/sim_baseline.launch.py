import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    # Nav2 parameters (baseline — no costmap filters)
    # ---------------------------------------------------------

    params_file = os.path.join(
        package_dir,
        'config',
        'nav2_params_baseline.yaml'
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

    declare_headless = DeclareLaunchArgument('headless', default_value='False')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='True')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

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
            'headless': headless,
            'use_rviz': use_rviz,
            'use_simulator': 'True',
            'use_robot_state_pub': 'True',
        }.items()
    )

    # ---------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------

    return LaunchDescription([
        declare_x_pose,
        declare_y_pose,
        declare_yaw,
        declare_headless,
        declare_use_rviz,
        nav2_simulation,
    ])
