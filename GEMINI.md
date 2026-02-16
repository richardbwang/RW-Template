# GEMINI.md - RW-Template Project Context

## Project Overview
**RW-Template** is an advanced **VEX V5 autonomous robotics template** designed for precise, algorithm-driven motion planning and control. It provides a robust foundation for VEX Robotics teams to implement sophisticated autonomous routines using proven algorithms like Odometry, PID control, and the Boomerang algorithm.

### Key Features
- **Odometry:** Tracks robot position (X, Y, Heading) using drivetrain encoders or optional horizontal/vertical tracking wheels.
- **PID Control:** Modular PID class for distance, turning, and heading correction.
- **Motion Algorithms:** Includes `driveTo`, `turnToAngle`, `moveToPoint`, and `boomerang` (for smooth pathing to a target pose).
- **Motion Chaining:** Seamlessly links multiple movement commands for fluid autonomous paths.
- **Distance Resets:** Integrated support for distance sensors to reset position based on field elements.

## Tech Stack
- **Language:** C++
- **Platform:** VEX V5 Robotics System
- **SDK:** VEX V5 SDK (VEXcode)
- **Build System:** Makefile (compatible with the VEX Robotics VS Code Extension)

## Project Structure
The project is architected to isolate core template logic from user-specific robot code to simplify updates.

### User-Modifiable Files (`custom/`)
Users should **only** modify files in this directory to ensure their changes are preserved when the template is updated.
- `custom/include/robot-config.h` & `custom/src/robot-config.cpp`: Define robot ports, motor groups, and tune physical parameters (wheel diameter, gear ratios, PID constants).
- `custom/include/autonomous.h` & `custom/src/autonomous.cpp`: Define specific autonomous routines and paths.
- `custom/include/user.h` & `custom/src/user.cpp`: Contains the competition entry points:
  - `runPreAutonomous()`: Sensor calibration and odometry thread initialization.
  - `runAutonomous()`: Logic for selecting and executing autonomous routines.
  - `runDriver()`: Driver control loop (defaulting to tank drive).

### Core Logic (`src/` & `include/`)
These files contain the underlying math and control systems.
- `pid.cpp/h`: Universal PID controller implementation.
- `motor-control.cpp/h`: Core drive functions and motion algorithms.
- `utils.cpp/h`: Math utilities and geometry functions.
- `main.cpp`: Standard VEX competition template entry point.

### VEX Environment (`vex/`)
Contains build configuration (`mkenv.mk`, `mkrules.mk`) and low-level V5 header files.

## Development Conventions
1. **Directory Isolation:** Never modify files outside of the `custom/` directory unless you are updating the core template logic.
2. **Configuration First:** Before running movements, ensure `distance_between_wheels`, `wheel_distance_in`, and PID constants are tuned in `custom/src/robot-config.cpp`.
3. **Competition Flow:** The project follows the standard VEX competition structure. The main tasks are delegated to the functions in `custom/src/user.cpp`.
4. **Odometry Selection:** The template automatically starts the correct odometry thread (with or without tracking wheels) based on the boolean flags `using_horizontal_tracker` and `using_vertical_tracker` in `robot-config.cpp`.

## Building and Running
The project is designed to be used with the **VEX Robotics Extension for Visual Studio Code**.

- **Build:** Click the "Build" button in the VEX extension sidebar or run `make` in the terminal.
- **Upload:** Click the "Download" button to transfer the binary to the VEX V5 Brain via USB or V5 Controller.
- **Clean:** Use the "Clean" option in the VEX extension or run `make clean`.

## Key Commands for LLM
- When asked to add a new autonomous routine:
  1. Define the function in `custom/include/autonomous.h`.
  2. Implement the movement logic in `custom/src/autonomous.cpp`.
  3. Update the `runAutonomous()` switch case in `custom/src/user.cpp`.
- When asked to configure a new motor:
  1. Declare the `extern motor` in `custom/include/robot-config.h`.
  2. Define the motor with its port and gear ratio in `custom/src/robot-config.cpp`.
