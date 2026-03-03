from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    lightrover_ros_share_dir = get_package_share_directory('lightrover_ros')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
        default_value=os.path.join(lightrover_ros_share_dir, 'params', 'X2.yaml'),
        description='Path to the LiDAR params YAML file'
    )

    driver_node = LifecycleNode(package='ydlidar_ros2_driver',
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
        arguments=['-0.042','0','0.1094','-1.5708','0','0','base_link','laser_frame'],
        output='screen'
    )

    tf2_base = Node(package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_footprint', 'base_link'],
        output='screen'
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_laser,
        tf2_base
    ])
