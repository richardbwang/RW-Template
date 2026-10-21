# Project Structure

!!! warning
    Updating replaces anything you change outside the `custom` folder. The [updater](../getting-started/installation.md#update-from-an-older-version) saves your copies in `rw-backup/` first.

## Your code: `custom/`

**custom/include/**

- `autonomous.h`: declarations for autonomous routines
- `robot-config.h`: declarations for robot devices
- `user.h`: declarations for user functions

**custom/src/**

- `autonomous.cpp`: your autonomous routines
- `robot-config.cpp`: your robot's device configuration
- `user.cpp`: user functions

## Template code

**include/**

- `motor-control.h`: core drive and motion control function declarations
- `pid.h`: reusable PID controller class declaration
- `utils.h`: math and geometry utility functions
- `mcl.h`: Monte Carlo localization (particle filter)
- `path.h`: motion profile and path math
- `path-following.h`: profiled moves and path followers
- `drive-log.h`: drive calibration for the simulator
- `auton-selector.h`: auton selector
- `driver-control.h`: driver control helpers
- `hardware.h`: the few things RW asks of the robot (motors, sensors, screen, SD card), so the rest of RW is the same on VEXcode and PROS
- `vex.h`: standard VEX libraries and device setup

**src/vexcode/**

- `main.cpp`: competition entry point and control flow
- `hardware.cpp`: the VEXcode version of `hardware.h`, using your devices from `custom/`

**src/** (the same code runs on VEXcode and PROS)

- `motor-control.cpp`: core drive and motion control function implementations
- `pid.cpp`: PID controller logic
- `utils.cpp`: math and geometry utility function implementations (odometry and distance reset math)
- `mcl.cpp`: particle filter math
- `path.cpp`: motion profiles, path smoothing, speed planning, pure pursuit and RAMSETE math
- `path-following.cpp`: `driveProfiled`, `turnProfiled`, `followPath`, `followPathRamsete`, `measureDriveFeedforward`
- `drive-log.cpp`: `calibrateDrive`, which records a run for the simulator
- `auton-selector.cpp`: auton selector
- `driver-control.cpp`: driver control helpers
- `holonomic.cpp`: X-drives, mecanum drives and H-drives: `strafeToPose`, `strafe`, `driveHolonomic` (see [Drive Types](drive-types.md))

## Computer-side tools

These aren't part of the robot build.

- **tests/**: computer-side tests for the odometry and distance reset math, plus an MCL field simulation. See [Testing on a Computer](../simulator/testing.md)
- **sim/**: the [simulator](../simulator/running.md). `run-sim.sh` runs it, `viewer.html` is the replay page, `drive-model.h` is the drive physics, `fit-profile.sh` calibrates it from your robot, `find-compiler.sh` finds a C++ compiler for the scripts, `vex/` holds the stand-in VEX API, and `sim-hooks.h` is how the stand-in talks to the simulated robot
- **tools/**: the [updater](../getting-started/installation.md#update-from-an-older-version). `update.ps1` (Windows) and `update.sh` (Mac and Linux) update the project to a new version, and `rw-files.txt` lists RW's files so they know what to replace. In VS Code's menu bar, **Terminal > Run Task... > Update RW-Template** runs them

## The PROS version

In a PROS project, your files are `src/rw-config.cpp`, `src/rw-autons.cpp`, `src/rw-user.cpp` and `include/rw-config.h`, which do the job of the `custom/` folder. RW's own files are `include/rw/`, `firmware/RW-Template.a`, `firmware/rw-template.mk` and `sim/` (with `sim/pros/` in place of `sim/vex/`), and `pros c apply RW-Template` replaces them when you update. See [Using PROS](../getting-started/pros.md).

The [RW-Template repository](https://github.com/richardbwang/RW-Template) builds that template from its `pros/` folder, with the same `src/` and `include/` code as the VEXcode version plus `pros/rw/hardware.cpp`, the PROS version of `hardware.h`.
