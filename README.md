## 1. Installation

### 1.0 Install ROS-based dependencies:
```bash
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-velodyne
sudo apt install ros-humble-velodyne-gazebo-plugins
sudo apt install ros-humble-velodyne-description
```

### 1.1 Clone and install all dependencies:
    
```bash
sudo apt install -y python3-rosdep
rosdep update

cd <your_ws>/src
git clone git@github.com:Kazuhi-Kawashima/go2_nav_humble.git
cd <your_ws>
rosdep install --from-paths src --ignore-src -r -y
```

### 1.2 Build your workspace:
```bash
cd <your_ws>
colcon build
source <your_ws>/install/setup.bash
```

### 1.3 Run mapping:
    
```bash
ros2 launch go2_config gazebo_velodyne.launch.py
```

### 1.4 Run navigation:
```bash
ros2 launch go2_config gazebo_velodyne_nav.launch.py
```

## 2. Autoware Universe + Gazebo (Go2 + VLP-16) 検証メモ

このリポジトリは Nav2 を使った Gazebo ナビゲーション向け構成です。
Autoware Universe を Gazebo 上で検証する場合は、別途 Autoware Universe の
ワークスペースを用意し、Go2 の Gazebo モデルと VLP-16 の出力を
Autoware が要求するトピック/TF に合わせて接続する必要があります。

### 2.1 Gazebo 上の Go2 + VLP-16 セットアップ
- Gazebo 側は `gazebo_velodyne.launch.py` / `gazebo_velodyne_nav.launch.py` により
  VLP-16 を含む Go2 モデルが起動します。
- VLP-16 は `/velodyne_points` を出力し、`velodyne` フレームで publish されます。
  Autoware Universe 側の LiDAR 入力トピックへリマップまたはブリッジが必要です。

### 2.2 Autoware Universe 側で必要になるもの（概要）
- 車両モデル（URDF/vehicle_info）
- 3D pointcloud（VLP-16 点群）
- TF ツリー（`base_link` / `odom` / `map` など）
- 地図（Lanelet2 + PCD）

### 2.3 具体的に必要な接続例（概念）
- Gazebo Go2: `/velodyne_points` -> Autoware: `sensor/pointcloud` などへリマップ
- TF: `base_link` / `base_footprint` の整合を取る（Autoware の車両フレームに合わせる）
- Odometry: `/odom` を Autoware のオドメトリ入力へブリッジ

### 2.4 次に追加すると良いファイル
- Autoware Universe 起動用の launch
- Go2 から Autoware への topic/TF ブリッジ設定
- VLP-16 のフレーム名・トピック名を合わせたリマップ設定

### 2.5 Autoware 向けブリッジノード（go2_autoware_bridge）
Gazebo 上の Go2 出力を Autoware Universe の標準トピックに合わせて
転送する簡易ブリッジノードを追加しました。

#### 2.5.1 ビルド
```bash
cd <your_ws>
colcon build --packages-select go2_autoware_bridge
source <your_ws>/install/setup.bash
```

#### 2.5.2 起動
```bash
ros2 launch go2_autoware_bridge go2_autoware_bridge.launch.py
```

#### 2.5.3 主なパラメータ
- `input_pointcloud_topic` (default: `/velodyne_points`)
- `output_pointcloud_topic` (default: `/sensing/lidar/top/pointcloud`)
- `pointcloud_frame_id` (default: `velodyne`)
- `input_odom_topic` (default: `/odom`)
- `output_odom_topic` (default: `/localization/kinematic_state`)
- `odom_frame_id` (default: `odom`)
- `base_frame_id` (default: `base_link`)

### 2.6 Autoware 制御出力 -> Go2 cmd_vel ブリッジ
Autoware Universe の制御出力（Twist/TwistStamped）を Go2 の `cmd_vel` に
流し込みたい場合、`go2_cmd_vel_bridge_node` を利用できます。

#### 2.6.1 起動
```bash
ros2 run go2_autoware_bridge go2_cmd_vel_bridge_node
```

#### 2.6.2 主なパラメータ
- `input_twist_topic` (default: `/control/command/twist`)
- `input_type` (default: `twist_stamped`, `twist` を指定可能)
- `output_cmd_vel_topic` (default: `/cmd_vel`)

### 2.7 Autoware 起動込みの統合 launch
Autoware Universe を同時に起動したい場合は `autoware_go2.launch.py` を使います。
Autoware の launch パッケージとファイル名は引数で指定できます。

#### 2.7.1 起動例
```bash
ros2 launch go2_autoware_bridge autoware_go2.launch.py \\
  enable_autoware:=true \\
  autoware_launch_package:=autoware_launch \\
  autoware_launch_file:=autoware.launch.xml
```
