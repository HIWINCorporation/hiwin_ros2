# HIWIN Robot ROS2

[![License - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

This repository provides the ROS2 stack for **HIWIN robots**, allowing integration with the ROS2 ecosystem for easy robot control and simulation.

## Contents
This repository follows branch naming aligned with ROS 2 distributions.
- Stable branches: `humble`, `iron`
- Development branches: `*-devel` (may be unstable)

## Features ##
- **Integration with `ros2_control`:** Direct hardware interface via ROS 2 control for precise control and monitoring.
- **MoveIt 2 Integration:** Enables motion planning, trajectory execution, and manipulation tasks.
- **Robot Drivers:** Built on top of the [hiwin_robot_client_library](https://github.com/HIWINCorporation/hiwin_robot_client_library) (currently under development) to support position control for HIWIN robots.

:warning: **Known Limitations**:warning:
The **hiwin_robot_client_library** is still under development. As a result:
1. The library cannot handle rapid command sequences effectively.
2. Execution times for physical robot movements are significantly longer than planned durations in trajectory commands.

## Packages in the Repository:
- `hiwin_driver` - Provides hardware interfaces for communication with HIWIN robots, including the implementation of dedicated controllers.
- `hiwin_ra6_moveit_config` - MoveIt 2 configuration package for the RA6 series of HIWIN robots. Includes tools for integration with MoveIt 2 for motion planning and control.
- `hiwin_rs4_moveit_config` - MoveIt 2 configuration package for the RS4 series of HIWIN robots. Includes tools for integration with MoveIt 2 for motion planning and control.

## General Requirements
- **Operating System:** Ubuntu 22.04 LTS
- **ROS 2 version:** Humble Hawksbill

## Getting Started
1. **Install ros2 packages**
Follow the steps outlined in the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).
2. **Source the ROS 2 Environment**
```bash
source /opt/ros/humble/setup.bash
```
3. **Create a ROS 2 Workspace**
```bash
mkdir -p ~/colcon_ws/src
```
4. **Clone the Repository and Build**
```bash
cd ~/colcon_ws

git clone https://github.com/HIWINCorporation/hiwin_ros2.git src/hiwin_ros2

colcon build --symlink-install

source install/setup.sh
```

## Usage
### :warning: **SAFETY FIRST**:warning:
*It is strongly recommended to test your code in simulation before using it on physical hardware.*

### Simulated hardware
To test the robot in a simulated environment:
```bash
ros2 launch hiwin_ra6_moveit_config ra6_moveit.launch.py ra_type:=ra605_710 use_fake_hardware:=true
```

### Real Robot Control
To connect to and control a physical robot:
```bash
ros2 launch hiwin_ra6_moveit_config ra6_moveit.launch.py ra_type:=ra605_710 use_fake_hardware:=false robot_ip:=<robot ip>
```
### **HRSS Offline Simulation**  
The **HIWIN Robot System Software (HRSS)** provides tools to control basic robot functions.  
For offline simulation:
1. Download [HRSS Offline](https://www.hiwinsupport.com/download_center.aspx?pid=MAR).
2. Launch the robot simulation:
```bash
ros2 launch hiwin_ra6_moveit_config ra6_moveit.launch.py ra_type:=ra605_710 use_fake_hardware:=false robot_ip:=<workstation ip>
```
