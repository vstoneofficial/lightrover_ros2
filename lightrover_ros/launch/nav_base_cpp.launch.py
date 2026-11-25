from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lightrover_ros'),
                'launch',
                'ydlidar_x2_launch.py',
            ])
        ]),
    )

    return LaunchDescription([
        launch_ydlidar,
        Node(package='lightrover_ros', executable='i2c_controller_cpp', output='screen'),
        Node(package='lightrover_ros', executable='odom_manager_cpp', output='screen'),
        Node(package='lightrover_ros', executable='pos_controller_cpp', output='screen'),
    ])
