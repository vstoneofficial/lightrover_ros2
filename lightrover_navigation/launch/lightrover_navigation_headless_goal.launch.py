import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('lightrover_navigation')
    navigation_launch = os.path.join(share_dir, 'launch', 'lightrover_navigation.launch.py')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_base = LaunchConfiguration('launch_base')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    initial_pose_wait_sec = LaunchConfiguration('initial_pose_wait_sec')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    goal_wait_sec = LaunchConfiguration('goal_wait_sec')
    server_wait_sec = LaunchConfiguration('server_wait_sec')
    result_wait_sec = LaunchConfiguration('result_wait_sec')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'launch_base': launch_base,
            'launch_rviz': 'false',
            'set_initial_pose': 'true',
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
            'initial_pose_wait_sec': initial_pose_wait_sec,
        }.items(),
    )

    delayed_goal = TimerAction(
        period=goal_wait_sec,
        actions=[
            Node(
                package='lightrover_ros',
                executable='send_nav_goal',
                name='send_nav_goal',
                output='screen',
                parameters=[{
                    'frame_id': 'map',
                    'x': goal_x,
                    'y': goal_y,
                    'yaw': goal_yaw,
                    'server_wait_sec': server_wait_sec,
                    'result_wait_sec': result_wait_sec,
                    'use_sim_time': use_sim_time,
                }],
            )
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(share_dir, 'maps', 'test.yaml'),
            description='Full path to map file to load',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share_dir, 'config', 'nav2_params.yaml'),
            description='Full path to Nav2 parameter file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'launch_base',
            default_value='true',
            description='Launch robot base nodes',
        ),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Initial pose x in map frame',
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Initial pose y in map frame',
        ),
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Initial pose yaw in radians',
        ),
        DeclareLaunchArgument(
            'initial_pose_wait_sec',
            default_value='5.0',
            description='Seconds to wait before publishing the initial pose',
        ),
        DeclareLaunchArgument(
            'goal_x',
            default_value='0.5',
            description='Goal x in map frame',
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='0.0',
            description='Goal y in map frame',
        ),
        DeclareLaunchArgument(
            'goal_yaw',
            default_value='0.0',
            description='Goal yaw in radians',
        ),
        DeclareLaunchArgument(
            'goal_wait_sec',
            default_value='12.0',
            description='Seconds to wait before sending the first goal',
        ),
        DeclareLaunchArgument(
            'server_wait_sec',
            default_value='30.0',
            description='Seconds send_nav_goal waits for NavigateToPose server',
        ),
        DeclareLaunchArgument(
            'result_wait_sec',
            default_value='40.0',
            description='Seconds send_nav_goal waits for navigation result',
        ),
        navigation,
        delayed_goal,
    ])
