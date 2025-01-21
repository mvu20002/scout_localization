# scout_localization
A library with examples demonstrating the localization of the Agilex Scout V2 robot on ROS. It utilizes sensor fusion of wheel odometry, visual odometry, GPS, and IMU measurements for enhanced accuracy.
# Agilex Scout V2 Localization Library

This repository provides examples for localizing the Agilex Scout V2 robot on ROS. It demonstrates sensor fusion using wheel odometry, visual odometry, GPS, and IMU measurements for accurate localization.

## Prerequisites
- Ensure the following repository is cloned and set up in your workspace:
  [clearpathrobotics/cpr_gazebo](https://github.com/clearpathrobotics/cpr_gazebo)

## Installation

Follow the steps below to set up the workspace:

```bash
git clone https://github.com/mvu20002/scout_localization.git
cd scout_localization
catkin_make
```

## Examples

### 1. IMU + Wheel Odometry
To launch localization using IMU and wheel odometry:
```bash
roslaunch intro_to_robot_autonomy term_project.launch
```

### 2. IMU + Wheel Odometry + GPS
To launch localization using IMU, wheel odometry, and GPS:
```bash
roslaunch intro_to_robot_autonomy term_project_gps.launch
```
