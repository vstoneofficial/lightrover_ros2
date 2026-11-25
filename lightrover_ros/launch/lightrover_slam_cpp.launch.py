"""SLAM と C++ 版の制御ノードを起動する launch.

操作用のノードは含まないため、別の端末で好きな操作方法を起動する。
  ゲームパッド (Python): ros2 launch lightrover_ros joy_input.launch.py
  ゲームパッド (C++)   : ros2 run joy joy_node と ros2 run lightrover_ros rover_gamepad_cpp
  キーボード           : ros2 run lightrover_ros rover_keyboard
  マウス               : ros2 run lightrover_ros rover_mouse
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('lightrover_ros')
    navigation_share_dir = get_package_share_directory('lightrover_navigation')
    slam_launch = os.path.join(share_dir, 'launch', 'lightrover_slam.launch.py')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'X2.yaml'),
        description='Path to the ROS2 parameters file to use.',
    )
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            navigation_share_dir,
            'config',
            'mapper_params_online_sync.yaml',
        ),
        description='Path to the slam_toolbox parameter file to use.',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz for SLAM',
    )
    launch_base_arg = DeclareLaunchArgument(
        'launch_base',
        default_value='true',
        description='Launch C++ base nodes for odometry and motor control',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items(),
    )

    launch_base = LaunchConfiguration('launch_base')
    base_nodes = [
        Node(
            package='lightrover_ros',
            executable='i2c_controller_cpp',
            output='screen',
            condition=IfCondition(launch_base),
        ),
        Node(
            package='lightrover_ros',
            executable='odom_manager_cpp',
            output='screen',
            condition=IfCondition(launch_base),
        ),
        Node(
            package='lightrover_ros',
            executable='pos_controller_cpp',
            output='screen',
            condition=IfCondition(launch_base),
        ),
    ]

    return LaunchDescription([
        params_file_arg,
        slam_params_file_arg,
        launch_rviz_arg,
        launch_base_arg,
        slam,
        *base_nodes,
    ])
