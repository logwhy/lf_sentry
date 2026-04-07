# teleop_gimbal_keyboard

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. Introduction

通过键盘控制，转换为云台的位置姿态话题 [pb_rm_interfaces/msg/GimbalCmd](https://github.com/SMBU-PolarBear-Robotics-Team/pb_rm_interfaces/blob/main/msg/GimbalCmd.msg) 发布。

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 2.2 Create Workspace

```bash
sudo apt install git-lfs
sudo pip install vcstool2
```

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/teleop_gimbal_keyboard.git
```

```bash
vcs import --recursive < dependencies.repos
```

### 2.3 Build

```bash
colcon build --symlink-install --cmake-args --packages-up-to teleop_gimbal_keyboard
```

### 2.4 Running

```bash
ros2 run teleop_gimbal_keyboard teleop_gimbal_keyboard --ros-args -r __ns:=/red_standard_robot1
```

Choose your robot namespace by changing the `__ns:=/red_standard_robot1` argument.
