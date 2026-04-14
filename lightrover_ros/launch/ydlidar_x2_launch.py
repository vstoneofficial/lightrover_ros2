from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    lightrover_ros_share_dir = get_package_share_directory('lightrover_ros')
    parameter_file = LaunchConfiguration('lidar_params_file')

    params_declare = DeclareLaunchArgument('lidar_params_file',
        default_value=os.path.join(lightrover_ros_share_dir, 'params', 'X2.yaml'),
        description='Path to the LiDAR params YAML file'
    )

    driver_node = Node(package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/'
    )

    tf2_laser = Node(package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=[
            '--x', '-0.042',
            '--y', '0',
            '--z', '0.1094',
            '--yaw', '-1.5708',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame',
        ],
        output='screen'
    )

    tf2_base = Node(package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link',
        ],
        output='screen'
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_laser,
        tf2_base
    ])
