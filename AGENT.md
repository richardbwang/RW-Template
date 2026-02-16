# Agent Instructions

Project: RW-Template based VEX V5 codebase for Push Back autonomous.

This repo uses a "custom/" overlay. Only edit files under custom/ unless explicitly asked otherwise.

## Robot Hardware Map (authoritative)

Drive motors:
- Left drivetrain: ports 20, 19, 18
- Right drivetrain: ports 11, 12, 13

Intake motors:
- Ports 1 and 10

Sensors:
- Horizontal odom tracker: port 9
- Vertical odom tracker: port 8
- Front distance sensor: port 5
- Left distance sensor: port 4
- Right distance sensor: port 3
- Back distance sensor: port 6
- Inertial sensor (IMU): port 21

Pneumatics (three-wire ports):
- TripState1: port A
- Odom Lift: port B
- Little Will: port C
- Wing: port D
- TripState2: port E
- Middle De-score: port F
- Fast Score: port G

Odom offsets (inches):
- Vertical offset: 1.0
- Horizontal offset: 2.5

## Autonomous Development Scope

We are focused on autonomous only. Use the motion APIs provided by the template (driveTo, turnToAngle, moveToPoint, boomerang, curveCircle) and odometry when configured.

Primary files to edit:
- custom/src/autonomous.cpp: routines and sequencing
- custom/include/autonomous.h: function declarations for routines
- custom/src/robot-config.cpp: motor/sensor ports and odom setup
- custom/include/robot-config.h: device declarations for ports

Do not modify core files in src/ or include/ unless asked.

## Checklist Before Writing a Routine

- Ensure robot hardware ports in custom/src/robot-config.cpp match the map above.
- Ensure all device declarations exist in custom/include/robot-config.h.
- Set tracking wheel usage flags and diameters if odom trackers are used.
- Apply tracker offsets using the provided distances in inches.
- Confirm distance sensor offsets for any distance resets.

## Routine Style Notes

- Keep autonomous routines deterministic and linear unless concurrency is required.
- Prefer short, named helper functions for repeated actions.
- Use timeouts on motion commands to avoid stalls.
- Reset or sync heading with the IMU when starting long sequences.
- When using odometry moves, make sure odom is enabled and configured.
