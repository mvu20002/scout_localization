# Agilex Scout V2 Localization Library

A library with examples demonstrating the localization of the Agilex Scout V2 robot on ROS. It utilizes sensor fusion of wheel odometry, visual odometry, GPS, and IMU measurements for enhanced accuracy.

## Prerequisites
- Ensure the following repository is cloned under `../scout_localization/src` path and set up in your workspace:
  [clearpathrobotics/cpr_gazebo](https://github.com/clearpathrobotics/cpr_gazebo)

Your file tree should look like this:
```plaintext
.
├── README.md
└── src
    ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
    ├── cpr_gazebo
    ├── intro_to_robot_autonomy
    ├── mono-slam
    ├── scout_odometry
    └── ugv_sim

6 directories, 2 files
```

## Installation

Follow the steps below to set up the workspace:

### Clone and Build
```bash
git clone https://github.com/mvu20002/scout_localization.git
cd scout_localization
catkin_make
```

### Source the Workspace
```bash
echo "source ~/scout_localization/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Examples

### 1. IMU + Wheel Odometry + GPS
To launch localization using IMU, wheel odometry, and GPS:
```bash
roslaunch intro_to_robot_autonomy term_project.launch
```
When you execute the command above, Gazebo and `rqt_multiplot` will open, allowing you to track the localization results in real time.

---
If you encounter any issues, feel free to check your setup and ensure all prerequisites are correctly installed.
