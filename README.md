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
