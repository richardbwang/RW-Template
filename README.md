# RW-Template

Badges below (Downloads and Stars) update automatically every 5-15 minutes

[<img src="https://img.shields.io/github/downloads/richardbwang/RW-Template/total?style=for-the-badge">](https://github.com/richardbwang/RW-Template/releases)

[<img src="https://img.shields.io/github/stars/richardbwang/RW-Template?style=for-the-badge">](https://github.com/richardbwang/RW-Template/stargazers)

## 📦 Project Description

Welcome to RW-Template! This project is an advanced **VEX V5 autonomous robotics template** focused on precise, algorithm-driven motion planning and control. It is designed for teams who want robust, adaptable autonomous routines with support for a wide range of sensor and drive configurations.

---

## 🚀 Features

### ✅ Universal PID Class
- Modular, reusable PID controller for all motion and heading tasks.

### 📍 Odometry
- Tracks robot position using drivetrain encoders and/or tracking wheels for accurate field navigation.

### 🧭 IMU Support
- Integrates inertial sensor (IMU) for heading correction and absolute orientation.

### 🔁 Tracking Wheel Flexibility
- Works with or without tracking wheels.
- Supports all tracking wheel configurations.
- Fall-back support using only drivetrain encoders and IMU for basic pose estimation.

### 🎯 Turn to Face Point/Heading
- Algorithms to turn the robot to face any field point or specific heading.

### 🔄 Swing to Face Heading
- Swing turn routines for efficient, single-side turning to a point or heading.

### 🏹 Move to Point
- Navigate to a specific coordinate using position and heading correction.
- Works consistently with or without tracking wheels.

### 📐 Curve Path Movement (`curveCircle`)
- Executes smooth, circular/curved paths to a target point.
- Fully compatible with robots **not using tracking wheels or full odometry**.
- Uses heading + distance PID control to maintain curvature.

### 🏹 Move to Pose via Boomerang
- Advanced “boomerang” algorithm for smooth, curved motion to a target pose (position + heading).

### ⛓️ Motion Chaining
- Seamlessly links multiple motion commands for fluid, uninterrupted autonomous routines.

### 💡 Easy Updates
- `custom/` folder isolates personal changes from core logic  
- future updates to core code are seamless
- copy/paste portability across projects or seasons  

---

## 🛠️ Installation/Update Guide

Follow these steps to set up the project on your local machine:

1. **Download and install Visual Studio Code (VS Code)**
   - [https://code.visualstudio.com/](https://code.visualstudio.com/)

2. **Install the VEX Robotics Extension in VS Code**
   - Open the Extensions view (`Ctrl/Cmd+Shift+X`)
   - Search for and install **"VEX Robotics Extension"** by VEX Robotics

3. **Set up the project folder**
   - Download the latest version's ZIP file titled **`RW-Template.zip`** (not the source code files) from the [Releases](https://github.com/richardbwang/RW-Template/releases) section below
   - Unzip the downloaded file
   - Open the resulting folder with Visual Studio Code
   - If you are an existing user, copy and paste the **custom** folder of the old version into the new project for updating to newer versions

---

## 📘 Usage Guide

**Note: Anything modified outside of the custom folder will not be preserved in updates.**

### 1. Project Structure

Your project is organized into the following folders and files:

**custom/include/**
- `autonomous.h` — Declarations for autonomous routines  
- `robot-config.h` — Declarations for robot devices
- `user.h` — Declarations for user functions

**custom/src/**
- `autonomous.cpp` — Your autonomous routines  
- `robot-config.cpp` — Your robot's device configurations
- `user.cpp` — User functions

**include/**
- `motor-control.h` — Core drive and motion control function declarations  
- `pid.h` — Reusable PID controller class declaration  
- `utils.h` — Math and geometry utility functions  
- `vex.h` — Standard VEX libraries and device setup

**src/**
- `main.cpp` — Competition entry point and control flow  
- `motor-control.cpp` — Core drive and motion control function implementations  
- `pid.cpp` — PID controller logic  
- `utils.cpp` — Math and geometry utility function implementations

### 2. Robot Configuration

Edit `custom/src/robot-config.cpp` and `custom/include/robot-config.h` to match your robot’s hardware.

- Set the correct ports for motors, sensors, and other robot devices
- Group drive motors into `left_chassis` and `right_chassis` motor groups

### 3. Tuning Your Robot

In `custom/src/robot-config.cpp`, locate the section labeled **USER-CONFIGURABLE PARAMETERS** and adjust the following:

- `distance_between_wheels`: Distance between the left and right wheels (in inches)  
- `wheel_distance_in`: Wheel circumference (see comments in code for help)  
- PID constants: `distance_kp`, `distance_ki`, `distance_kd`, etc.

If using a horizontal and/or vertical tracking wheel:

- Set `using_horizontal_tracker = true` and/or `using_vertical_tracker = true`
- Configure tracker distances and diameters accordingly

### 4. Autonomous Programming

Edit `custom/src/autonomous.cpp` to define your autonomous routines.

You can use the motion functions from `motor-control.h`:

- `driveTo(distance, time_limit, exit, max_output)`  
- `turnToAngle(angle, time_limit, exit, max_output)`  
- `moveToPoint(x, y, dir, time_limit, exit, max_output, overturn)`  
- `boomerang(x, y, dir, angle, dlead, time_limit, exit, max_output, overturn)`

Select the routine to run inside the `runAutonomous()` function in `custom/src/user.cpp`.

### 5. Driver Control

Edit the `runDriver()` function in `custom/src/user.cpp`.

By default, it uses tank drive:

`driveChassis(ch3 * 0.12, ch2 * 0.12);`

You can customize this for:

- Arcade drive  
- Split arcade  
- Adding button controls for mechanisms

### 6. Competition Setup

This template follows the VEX Competition structure:

- `pre_auton()` which calls `runPreAutonomous()` runs once at startup (ideal for sensor calibration)  
- `autonomous()` which calls `runAutonomous()` runs during the autonomous period  
- `usercontrol()` which calls `runDriver()` runs during the driver control period  
- The `main()` function connects all of these automatically

### 7. Tips for Success

- Read comments in each file — they clarify how each function and variable works  
- Tune PID values for optimal performance  
- Test motion functions like `driveTo`, `turnToAngle`, and `moveToPoint` individually  
- Use odometry for advanced navigation (with or without tracking wheels)  
- Don’t hesitate to reach out to me or the VEX community for help
- Join our discord (link is in the description)

### 8. Where to Start

- Set up ports and devices in `custom/src/robot-config.cpp` and `custom/include/robot-config.h`  
- Input chassis measurements and PID values in `custom/src/robot-config.cpp`  
- Confirm movement and controls via driver control testing  
- Create and test simple autonomous routines  
- Expand with more complex logic and paths as you grow confident

---

This template is **competition-ready** and provides a strong foundation for building reliable, high-performance autonomous routines using **proven robotics algorithms**, whether or not your robot uses advanced odometry. 

If you find this template useful, please star this repository and subscribe to https://www.youtube.com/@1698V to follow my team's progress and learn more about our programming. 

