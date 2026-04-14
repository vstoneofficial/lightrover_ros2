from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    share_dir = get_package_share_directory('lightrover_ros')
    slam_launch = os.path.join(share_dir, 'launch', 'lightrover_slam.launch.py')
    joycon_launch = os.path.join(share_dir, 'launch', 'pos_joycon.launch.py')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'X2.yaml'),
        description='Path to the ROS2 parameters file to use.',
    )
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('lightrover_navigation'),
            'config',
            'mapper_params_online_sync.yaml',
        ),
        description='Path to the slam_toolbox parameter file to use.',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Whether to launch RViz for SLAM',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items(),
    )

    joycon = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joycon_launch),
    )

    return LaunchDescription([
        params_file_arg,
        slam_params_file_arg,
        launch_rviz_arg,
        slam,
        joycon,
    ])
