# MyBot ROS 2 Project

## Bot Image

![photo](https://github.com/user-attachments/assets/20d03f3e-8307-42ca-9634-eef9c0c97995)


> > This repository contains the complete ROS 2 workspace for **MyBot**, a differential drive mobile robot built for autonomous navigation and SLAM.

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)

## 🎥 Demo Video

**[Click here to see the robot in action!](https://youtu.be/SLRAsSv7wkM)**

## ✨ Features

* **Full Robot Bringup**: Launch files to bring the robot hardware, sensors, and software stack online.
* **SLAM**: Real-time map building using `slam_toolbox`.
* **Localization**: EKF-based sensor fusion for accurate odometry using `robot_localization`.
* **Web-Based Tuning**: The motor controller (ESP32) hosts a webpage for real-time PID tuning without re-flashing.
* **Navigation**: Full autonomous navigation with Nav2.
* **Simulation**: Gazebo simulation model and launch files for testing.

## 🛠️ Technology Stack

### Hardware
* **Chassis**: `Custom 3D Printed`
* **Compute (On-board)**: ` Raspberry Pi 4 (4GB)`
* **Microcontroller**: `ESP32 (Used as the low-level motor controller and encoder interface)`
* **Motors**: `DC Motors with Encoders`
* **Motor Driver**: `L298N`
* **Sensors (IMU)**: ` MPU-6050 / BNO085`
* **Sensors (LiDAR)**: `RPLidar A1 `
* **Power**: `[e.g., 12V LiPo Battery + 5V Buck Converter]`

### Software
* **OS**: `Ubuntu 22.04`
* **ROS 2**: `ROS 2 Humble Hawksbill`
* **Primary Languages**: `Python` and `C++`
* **Firmware**: The ESP32 motor driver runs custom firmware that hosts a **WiFi web server**, allowing for on-the-fly PID tuning of the motor encoder feedback loop from any browser.
* **Key ROS 2 Packages**:
    * `slam_toolbox` (for SLAM)
    * `robot_localization` (for EKF)
    * `nav2_stack` (for Navigation)
    * `ros2_control` (for hardware interfacing)

## 📦 Workspace Structure

This workspace is organized into several custom ROS 2 packages:

| Package | Description |
| :--- | :--- |
| `mybot_bringup` | Contains all top-level launch files for starting the robot. |
| `mybot_description` | Holds the robot's URDF (Unified Robot Description Format) files. |
| `mybot_firmware` | Code for the microcontroller and the `ros2_control` hardware interface. |
| `mybot_controller` | `ros2_control` controller configurations and custom controller code. |
| `mybot_localization` | EKF configuration files for `robot_localization`. |
| `mybot_mapping` | Configuration and launch files for `slam_toolbox`. |
| `mybot_planning` | Nav2 stack configuration files (planners, controllers, BT). |
| `mybot_msgs` | Custom ROS 2 message and service definitions. |
| `mybot_utils` | Contains utility scripts and nodes used by other packages. |
| `mybot_calibration` | Scripts for sensor calibration (e.g., IMU, camera). |
| `mybot_motion` | High-level motion logic (e.g., "go to goal"). |

## 🚀 Installation & Setup

### 1. Prerequisites
* A computer with Ubuntu 22.04 installed.
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) (Desktop Install).
* [Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) and [colcon](https://colcon.readthedocs.io/en/released/user/installation.html).

### 2. Clone & Build
```bash
# 1. Create a new ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone this repository
git clone https://github.com/sudhanshur555/Indoor_navigation_bot.git

# 3. Go back to the workspace root
cd ~/ros2_ws

# 4. Install dependencies from ROS packages
rosdep install --from-paths src -y -i

# 5. Build the workspace
colcon build

# 6. Source the workspace
source install/setup.bash

# 7. Launch the project on real hardware 
ros2 launch my_bot_bringup real_robot.launch.py use_slam:=true use_sim_time:=false
