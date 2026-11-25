import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='lightrover_ros', executable='i2c_controller_cpp', output='screen'),
        launch_ros.actions.Node(
            package='lightrover_ros', executable='odom_manager_cpp', output='screen'),
        launch_ros.actions.Node(
            package='lightrover_ros', executable='pos_controller_cpp', output='screen'),
        launch_ros.actions.Node(
            package='lightrover_ros', executable='rover_gamepad_cpp', output='screen'),
        launch_ros.actions.Node(
            package='joy', executable='joy_node', output='screen',
            parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.1}])
    ])
