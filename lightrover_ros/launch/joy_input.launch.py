import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='lightrover_ros',
            executable='rover_gamepad',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            output='screen',
            parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.1}],
        ),
    ])
