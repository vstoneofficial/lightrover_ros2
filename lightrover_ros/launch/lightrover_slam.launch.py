from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name

from lifecycle_msgs.msg import Transition
import os


def generate_launch_description():
    share_dir = get_package_share_directory('lightrover_ros')
    navigation_share_dir = get_package_share_directory('lightrover_navigation')
    parameter_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    launch_rviz = LaunchConfiguration('launch_rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'X2.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )
    launch_rviz_declare = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz for SLAM',
    )
    slam_params_declare = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            navigation_share_dir,
            'config',
            'mapper_params_online_sync.yaml',
        ),
        description='Path to the slam_toolbox parameter file to use.',
    )

    # --- Static TFs ---
    static_tf_base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
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
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
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
    )

    # --- YDLIDAR driver (通常ノードとして起動するのが正しい) ---
    driver_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
    )

    # --- slam_toolbox (LifecycleNode) ---
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='/', 
        output='screen',
        parameters=[
            slam_params_file,
        ],
    )

    # 起動したら configure を送る
    slam_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_node_name('slam_toolbox'),
                                transition_id=Transition.TRANSITION_CONFIGURE,
                            )
                        )
                    ],
                )
            ],
        )
    )

    # inactive になったら activate を送る
    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_node_name('slam_toolbox'),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share_dir, 'config', 'gmapping.rviz')],
        output='screen',
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'QT_X11_NO_MITSHM': '1',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
        },
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        params_declare,
        launch_rviz_declare,
        slam_params_declare,
        driver_node,
        tf2_node,
        static_tf_base_to_footprint,
        slam_node,
        slam_configure,
        slam_activate,
        rviz2_node,
    ])
