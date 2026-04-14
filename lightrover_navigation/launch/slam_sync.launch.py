from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node


from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rviz_config_path = get_package_share_path('lightrover_navigation') / 'rviz/slam.rviz'
    base_launch_path = get_package_share_path('lightrover_ros') / 'launch/nav_base.launch.py'

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(rviz_config_path),
                                    description='Absolute path to rviz config file')
    launch_rviz_arg = DeclareLaunchArgument(
        name='launch_rviz',
        default_value='true',
        description='Whether to launch RViz for SLAM',
    )
    launch_base_arg = DeclareLaunchArgument(
        name='launch_base',
        default_value='true',
        description='Whether to launch base nodes required for lidar and odometry',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'QT_X11_NO_MITSHM': '1',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
        },
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    launch_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lightrover_navigation'),
                    'launch',
                    'online_sync_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'false',
            }.items()
        )

    launch_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(base_launch_path)),
        condition=IfCondition(LaunchConfiguration('launch_base')),
    )

    return LaunchDescription([
        rviz_arg,
        launch_rviz_arg,
        launch_base_arg,
        rviz_node,
        launch_base,
        launch_slam,
    ])
