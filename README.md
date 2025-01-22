# Agilex Scout V2 Localization Library

This repository provides examples and tools for localizing the Agilex Scout V2 robot in a ROS environment. It demonstrates sensor fusion of wheel odometry, visual odometry, GPS, and IMU measurements to achieve enhanced localization accuracy.

---

## Prerequisites

Ensure the following dependencies are installed before proceeding:

### System Dependency
Install the required library for configuration management:
```bash
sudo apt-get install libconfig++-dev
```


## Installation

Follow these steps to set up and configure the workspace:

### Clone and Build
```bash
git clone https://github.com/mvu20002/scout_localization.git
cd scout_localization
catkin_make
```

### Source the Workspace
Add the workspace setup script to your shell configuration for convenience:
```bash
echo "source ~/scout_localization/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Additional Repository
Clone the required repository into your workspace under the `src` directory:
- [Clearpath Robotics CPR Gazebo](https://github.com/clearpathrobotics/cpr_gazebo)

---

### Verify Directory Structure
Ensure your workspace structure matches the following layout:
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

---

## Examples

### 1. IMU + Wheel Odometry + GPS
To launch localization using IMU, wheel odometry, and GPS:
```bash
roslaunch intro_to_robot_autonomy term_project.launch
```
When you execute the command above, Gazebo and `rqt_multiplot` will open, allowing you to track the localization results in real time.

---

If you encounter any issues, feel free to check your setup and ensure all prerequisites are correctly installed.
