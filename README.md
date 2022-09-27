# lightrover_ros2

ライトローバーのROS2用のサンプルパッケージ（仮）です。  
現在開発中のためROS1では実装されている機能が一部未実装かつ、詳細なドキュメントについても現在準備中です。  
また、将来的に大幅な改変が行われる可能性があります。  
使用の際には以上の点にご注意ください。  

## 使用方法
### Ubuntu MATE 20.04のセットアップ
1. 下記のリンクのUbuntu MATEのセットアップ、VNCの設定までを一通り行う。
https://vstoneofficial.github.io/lightrover_webdoc/setup/softwareSetupUbuntu/
<br>

2. Ubuntuの更新を行う。
```
sudo apt update
sudo apt upgrade
```

### ROS2 foxyのインストール
1. 下記のコマンドを実行する。
```
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

2. ros-foxy-desktopをインストールする
```
sudo apt update
sudo apt install ros-foxy-desktop
```

3. その他環境を構築する
```
sudo apt install -y python3-pip
pip3 install -U argcomplete
```

4. colconのインストール
```
sudo apt install python3-colcon-common-extensions
```

5. 新たにターミナルを開き下記コマンドでtalkerサンプルを実行する
```
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker
```

6. 新たにターミナルを開き下記コマンドでlistenerサンプルを実行する
```
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_py listener
```

listenerノードがtalkerから発信されるトピックをサブスクライブできればOK

7. ROS2用のワークスペースの作製
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
. install/setup.bash
```

### ライトローバーのros2用パッケージをセットアップ
1. light_rover_ros2をインストール
```
cd ~/ros2_ws/src
git clone https://github.com/vstoneofficial/lightrover_ros2/lightrover_interface
git clone https://github.com/vstoneofficial/lightrover_ros2/lightrover_ros
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build
```

### YDLidarのセットアップ
1. 必要パッケージのインストール
```
sudo apt install cmake pkg-config
```

2. YDLidar-SDKをインストールする
```
cd ~/
git clone https://github.com/YDLIDAR/YDLidar-SDK
mkdir -p ~/YDLidar-SDK/build
cd YDLidar-SDK/build
cmake ..
make
sudo make install
```

3. YDLiDAR ROS2ドライバをクローンする
```
cd ~/ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver
```

4. パラメータを編集

```
sudo nano ~/ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml  
```

~~~
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    ignore_array: ""
    baudrate: 115200
    lidar_type: 1
    device_type: 0
    sample_rate: 3
    abnormal_check_count: 4
    resolution_fixed: true
    reversion: false
    inverted: true
    auto_reconnect: true
    isSingleChannel: true
    intensity: false
    support_motor_dtr: true
    angle_max: 180.0
    angle_min: -180.0
    range_max: 8.0
    range_min: 0.1
    frequency: 10.0
    invalid_range_is_inf: false
~~~

5. YDLiDAR ROS2をインストールする
```
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
colcon build
. install/setup.bash
```

6. Rviz2に表示する
```
cd ~/
sudo chmod 777 /dev/ttyUSB0
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py
```

### ゲームパッドによる走行
※DhualShock4（以降DS4と表記）を使用する場合の手順です。

1. 必要パッケージのインストール
```
sudo pip install ds4drv transforms3d
sudo apt install ros-foxy-joy* ros-foxy-tf2-tools ros-foxy-tf-transformations
```

2. DS4の接続
```
sudo ds4drv
```

3. ローンチファイルの実行
新しい端末を開き下記コマンドを実行
```
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
. install/setup.bash
ros2 launch lightrover_ros pos_joycon.launch.py
```

### SLAMの実行
1. 必要パッケージのインストール
```
sudo apt install ros-foxy-slam-toolbox
```

2. DS4の接続
```
sudo ds4drv
```

3. ゲームパッド走行サンプルの実行
新しい端末を開き下記コマンドを実行
```
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
. install/setup.bash
ros2 launch lightrover_ros pos_joycon.launch.py
```

4. SLAM用ローンチファイルの実行
新しい端末を開き下記コマンドを実行
```
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
. install/setup.bash
sudo chmod 777 /dev/ttyUSB0
ros2 launch lightrover_ros lightrover_mapping.launch.py
```