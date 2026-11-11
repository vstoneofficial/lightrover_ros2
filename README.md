# lightrover_ros2

ライトローバーのROS2用のサンプルパッケージです。  
詳細な使用方法は以下のリンク先をご参照ください。  
[ライトローバーWebDoc](https://vstoneofficial.github.io/lightrover_webdoc/)

## 手動操作

ゲームパッドに加えて、マウス・キーボードでもライトローバーを操作できます。

| 操作方法 | 制御ノードとまとめて起動 | 操作ノードのみ起動 |
| --- | --- | --- |
| ゲームパッド | `ros2 launch lightrover_ros pos_joycon.launch.py` | `ros2 launch lightrover_ros joy_input.launch.py` |
| マウス | `ros2 launch lightrover_ros pos_mouse.launch.py` | `ros2 run lightrover_ros rover_mouse` |
| キーボード | `ros2 launch lightrover_ros pos_keyboard.launch.py` | `ros2 run lightrover_ros rover_keyboard` |

- マウス: 表示されたウィンドウ上で左ボタンを押している間だけ走行し、離すと停止します。ウィンドウ中央より上で前進、下で後退、左右で旋回し、中央から離れるほど速くなります。
- キーボード: `w`/`↑` 前進、`s`/`↓` 後退、`a`/`←` 左旋回、`d`/`→` 右旋回、スペースで停止。キーを押している間だけ走行し、離すとすぐに停止します (1回だけ押したときは約0.7秒走行します)。
  - `pos_keyboard.launch.py` はキー入力用のウィンドウを xterm で開きます。SSH 接続などで xterm が使えない場合は、制御ノードを起動したうえで別の端末から `ros2 run lightrover_ros rover_keyboard` を実行してください。
- 最高速度は `linear_scale`、`angular_scale` で変更できます。既定値はマウスが 0.2 m/s・1.2 rad/s、キーボードが 0.1 m/s・0.8 rad/s です。
  例: `ros2 run lightrover_ros rover_keyboard --ros-args -p linear_scale:=0.2`

マウス・キーボード操作には以下のパッケージが必要です。

```
sudo apt install python3-tk xterm
```
