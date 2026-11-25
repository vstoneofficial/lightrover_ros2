import launch
import launch_ros.actions


# pos_joycon.launch.py のゲームパッドの代わりに、マウスでライトローバーを操作する
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='lightrover_ros', executable='i2c_controller', output='screen'),
        launch_ros.actions.Node(
            package='lightrover_ros', executable='odom_manager', output='screen'),
        launch_ros.actions.Node(
            package='lightrover_ros', executable='pos_controller', output='screen'),
        launch_ros.actions.Node(
            package='lightrover_ros', executable='rover_mouse', output='screen'),
    ])
