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
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    description_package_path = get_package_share_path('lightrover_description')
    default_model_path = description_package_path / 'urdf/lightrover_urdf.xacro'
    default_rviz_config_path = description_package_path / 'rviz/lightrover.rviz'
    
    launch_ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lightrover_ros'),
                'launch',
                'ydlidar_x2_launch.py'
            ])
        ]),
    )
    
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    

    return LaunchDescription([  
        launch_ydlidar,
        model_arg,
        robot_state_publisher_node,
        Node(package='lightrover_ros', executable='i2c_controller', output='screen'),
        Node(package='lightrover_ros', executable='odom_manager', output='screen'),
        Node(package='lightrover_ros', executable='pos_controller', output='screen'),
    ])
