# HIWIN Robot

[![License - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

ROS2 stack for HIWIN robots.

# Contents
Branch naming follows the ROS distribution they are compatible with. -devel branches may be unstable. Releases are made from the distribution branches (humble, iron).

# System Requirements
### TBD.

# Install From Source
### 1.Install requirements:
```bash
sudo apt install -y \
ros-humble-ament-cmake \
ros-humble-ament-cmake-clang-format \
ros-humble-angles \
ros-humble-ros2-controllers \
ros-humble-ros2-control \
ros-humble-ros2-control-test-assets \
ros-humble-controller-manager \
ros-humble-control-msgs \
ros-humble-control-toolbox \
ros-humble-generate-parameter-library \
ros-humble-joint-state-publisher \
ros-humble-joint-state-publisher-gui \
ros-humble-moveit \
ros-humble-pinocchio \
ros-humble-realtime-tools \
ros-humble-xacro \
ros-humble-hardware-interface \
ros-humble-ros-gz \
```

### 2.Create a ROS 2 workspace:
```bash
mkdir -p ~/colcon_ws/src
```
### 3.Clone repo and build packages:
```bash
source /opt/ros/humble/setup.bash

cd ~/colcon_ws

git clone https://github.com/HIWINCorporation/hiwin_ros2.git src/hiwin_ros2

colcon build --symlink-install

source install/setup.sh
```

# Usage
### Mock hardware
The package can simulate hardware with the ``ros2_control``MockSystem. For more details see [ros2_control](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html) documentation for more details.
```bash
ros2 launch hiwin_ra6_moveit_config ra6_moveit.launch.py ra_type:=ra610_1869
```
