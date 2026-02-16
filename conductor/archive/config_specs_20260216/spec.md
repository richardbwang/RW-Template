# Track: Configuring project with robot specifications

## Objective
Configure the template with the physical specifications and device ports of the robot for the 2025-2026 season of Push-Back.

## Scope
- Define motor ports and groupings in `custom/src/robot-config.cpp`.
- Set physical constants: `distance_between_wheels`, `wheel_distance_in`.
- Tune initial PID constants for linear and turning movements.
- Configure sensor ports for inertial sensor, odometry wheels, and distance resets.

## Requirements
- All robot hardware must be accurately reflected in the configuration.
- Motor groupings must be correctly assigned to `left_chassis` and `right_chassis`.
- The `wheel_distance_in` must be calculated accurately based on gear ratio and wheel size.
