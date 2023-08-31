import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lightrover_ros'),
                'launch',
                'ydlidar_x2_launch.py'
            ])
        ]),
    )

    return LaunchDescription([  
        launch_ydlidar,
        Node(package='lightrover_ros', executable='i2c_controller', output='screen'),
        Node(package='lightrover_ros', executable='odom_manager', output='screen'),
        Node(package='lightrover_ros', executable='pos_controller', output='screen'),
    ])
