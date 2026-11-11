import launch
import launch_ros.actions


# pos_joycon.launch.py のゲームパッドの代わりに、キーボードでライトローバーを操作する
# キー入力は xterm で開いた別ウィンドウで受け付ける。
# xterm が使えない場合 (SSH 接続など) は nav_base.launch.py などで制御ノードを起動し、
# 別の端末で ros2 run lightrover_ros rover_keyboard を実行する。
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='lightrover_ros', executable='i2c_controller', output='screen'),
        launch_ros.actions.Node(
            package='lightrover_ros', executable='odom_manager', output='screen'),
        launch_ros.actions.Node(
            package='lightrover_ros', executable='pos_controller', output='screen'),
        launch_ros.actions.Node(
            package='lightrover_ros', executable='rover_keyboard', output='screen',
            prefix='xterm -fa Monospace -fs 12 -e'),
    ])
