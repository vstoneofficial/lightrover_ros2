from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os

def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    rviz_config_file = os.path.join(share_dir, 'config','ydlidar.rviz')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ydlidar_ros2_driver_node'

    static_tf_base_to_footprint = Node(package='tf2_ros',
                                       executable='static_transform_publisher',
                                       output='screen',
                                       arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'])

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'X2.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    
    driver_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                                )
                                
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',output='screen',
                    arguments=['-0.042', '0', '0.1094', '-1.5708', '0','0','base_link','laser_frame'],
                    )
    
    slam_node = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node',
        output='screen',
        parameters=[
            get_package_share_directory(
                'lightrover_ros')
            + '/configuration_files/mapper_params_offline.yaml'
        ],
    )
    
    rviz2_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=[
                        '-d',
                        get_package_share_directory('lightrover_ros')
                            + '/config/gmapping.rviz'],
                    )
    
    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
        static_tf_base_to_footprint,
        slam_node,
        rviz2_node,
    ])
