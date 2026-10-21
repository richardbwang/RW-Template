# Using PROS

RW-Template also comes as a PROS template, so you can add it to a new or existing PROS project. The motions, settings, odometry and auton selector are the same as in the VEXcode version. Only the device setup uses PROS's types. It needs PROS kernel 4.2.1 or newer: `pros c info-project` shows your project's kernel version, and `pros c upgrade` updates it.

## Install

In VS Code with the PROS extension, open your project and its PROS terminal (the PROS icon on the left, then **Integrated Terminal**), and run:

```
pros c add-depot RW-Template https://raw.githubusercontent.com/richardbwang/RW-Template/depot/stable.json
pros c apply RW-Template
```

The first command tells PROS where to find RW-Template, and you only need it once per computer. The second adds RW to your project:

- `include/rw/` and `firmware/RW-Template.a`: RW itself
- `src/rw-config.cpp`: your devices and settings
- `src/rw-autons.cpp`: your autonomous routines
- `src/rw-user.cpp`: your pre-autonomous, autonomous and driver code
- `include/rw-config.h`: declarations for your own devices and autons
- `sim/`: the [simulator](#simulator), which runs your autons on a computer

## Connect it to main.cpp

In a new PROS project, replace everything in `src/main.cpp` with this:

```cpp
#include "main.h"
#include "rw-config.h"

void initialize() { runPreAutonomous(); }
void disabled() {}
void competition_initialize() {}
void autonomous() { runAutonomous(); }
void opcontrol() { runDriver(); }
```

The `main.cpp` a new PROS project starts with prints to the Brain screen with LLEMU (`pros::lcd`), which would draw over RW's auton selector. In a project you already have, add the three RW calls to your own `initialize()`, `autonomous()` and `opcontrol()` instead, and remove any `pros::lcd` calls.

## Set up your robot

Then follow the [Quick Start](quick-start.md), using these files in place of the `custom` folder:

| VEXcode | PROS |
|---|---|
| `custom/src/robot-config.cpp` | `src/rw-config.cpp` |
| `custom/src/autonomous.cpp` | `src/rw-autons.cpp` |
| `custom/src/user.cpp` | `src/rw-user.cpp` |
| `custom/include/robot-config.h` and `autonomous.h` | `include/rw-config.h` |

Devices use PROS's types, with the names RW expects:

```cpp
pros::MotorGroup left_chassis({-1, -2, 3}, pros::MotorGearset::blue);  // A negative port reverses a motor
pros::Imu inertial_sensor(7);
pros::Rotation horizontal_tracker(-10);
pros::Distance front_sensor(12);
```

Define every one of them in `src/rw-config.cpp`, even devices your robot doesn't have (give those any free port), the same way as on VEXcode.

## Simulator

The [simulator](../simulator/running.md) works in PROS projects too. With a C++ compiler installed on your computer (see [Running the Simulator](../simulator/running.md) for that and the options), run this from the project folder:

```bash
sh sim/run-sim.sh "Tuning Test"
```

It builds your `src/` files with RW's code from `sim/rw/` (`firmware/RW-Template.a` only runs on the robot) and stand-ins for PROS's devices in `sim/pros/`. Then it runs `initialize()` and `autonomous()` from your `src/main.cpp`, the way PROS does, and writes a replay to `sim/out/`.

The stand-ins cover RW and the common parts of PROS: motors and motor groups, the IMU, rotation and distance sensors, three-wire ports, tasks, timing and the controller. Code that uses another library, like LemLib or OkapiLib, won't build in the simulator.

## Update

Run `pros c apply RW-Template` again. PROS replaces RW's own files (`sim/` too, apart from your replays and robot profile) and leaves `src/rw-config.cpp`, `src/rw-autons.cpp`, `src/rw-user.cpp` and `include/rw-config.h` alone. New settings have defaults, so your files keep building.

## Differences from VEXcode

The rest of these docs show VEXcode code. RW's functions are the same on PROS, but a few things are written differently:

| VEXcode | PROS |
| --- | --- |
| `ch3`, `ch4`, `ch1`, `ch2` (the sticks) | `stick(pros::E_CONTROLLER_ANALOG_LEFT_Y)`, `LEFT_X`, `RIGHT_X` and `RIGHT_Y`. `stick()` is in `src/rw-user.cpp` and turns PROS's -127 to 127 into the -100 to 100 RW uses |
| `wait(10, msec)` | `pros::delay(10)` |
| `intake_motor.spin(fwd, 12, volt)` | `intake_motor.move_voltage(12000)` (in millivolts) |
| `intake_motor.stop(hold)` | `intake_motor.set_brake_mode(pros::MotorBrake::hold);` then `intake_motor.brake();` |
| `stopChassis(hold)` | `stopChassis(pros::MotorBrake::hold)` |
| `waitUntil(done());` | `while (!done()) pros::delay(5);` There's no `waitUntil()`, because it would clash with LemLib's |

Also:

- RW's code goes into each upload along with yours, because it uses your devices by name
- The auton selector shows the letters RW where the VEXcode version shows the logo
