<p align="center">
  <img src="assets/logo-animated.svg" width="160" alt="RW-Template logo">
</p>

# RW-Template

Badges below (Downloads and Stars) update automatically every 5-15 minutes

[<img src="https://img.shields.io/github/downloads/richardbwang/RW-Template/total?style=for-the-badge">](https://github.com/richardbwang/RW-Template/releases)

[<img src="https://img.shields.io/github/stars/richardbwang/RW-Template?style=for-the-badge">](https://github.com/richardbwang/RW-Template/stargazers)

## 📦 Project Description

Welcome to RW-Template! This project is an advanced **VEX V5 autonomous robotics template** focused on precise, algorithm-driven motion planning and control. It is designed for teams who want robust, adaptable autonomous routines with support for a wide range of sensor and drive configurations.

---

## 🆕 What's New in 4.0.0

- **Odometry fixes.** Without tracking wheels, every 90° turn in place used to add about 8.7" of position error. With only a vertical tracking wheel, X and Y were swapped. Curves were also slightly off with tracking wheels. All four setups now share one tested implementation.
- **Async motions:** run a motion in the background and use `waitUntilTraveled()`, `waitUntilTurned()`, `waitUntilNear()`, `waitForMotion()` and `cancelMotion()` to time mechanisms to it.
- **Monte Carlo Localization (MCL):** optional. It uses your distance sensors to keep odometry from drifting while the robot drives.
- **Smarter distance resets:** they trace the sensor beam to the wall it actually hits and skip steep, near-corner, or suspicious readings.
- **Auton selector** on the Brain screen and the controller, remembered on the SD card.
- **Driver curves** (exponential, with deadband) for tank, arcade, and split arcade.
- **IMU scale correction**, a **stall exit** for stuck motions, and `setPose()`.
- **Simulator:** run your autons on a computer and watch a replay on an Override field, no robot needed. Optionally calibrate it from a 15-second run on your robot so it drives like yours.
- **Motion profiling:** `driveProfiled()` and `turnProfiled()` speed up, cruise and slow down smoothly using a feedforward model of your drive.
- **Path following:** smooth paths through waypoints (or drawn in path.jerryio), followed with pure pursuit or RAMSETE.

Updating from an older version? Your old `custom` folder still builds, because every new setting has a default. To use the new features, copy the new parts of `custom/src/user.cpp` and `custom/src/robot-config.cpp` into yours.

These features are new. They pass computer tests and run in the simulator, but haven't been tried on many real robots yet. If something doesn't work the way this README says, tell us on Discord or open a GitHub issue.

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

### 🧭📍 Distance Resets
- Seamlessly integrated position resets using distance sensors.
- Has support for sensors on all sides, or on just the specific sides you want.
- Traces each sensor's beam to the wall it actually hits, so angled readings and readings near corners work.
- Skips readings that hit the wall too steeply, land too close to a corner (e.g. Override's loaders), or disagree too much with odometry.

### 🛰️ Monte Carlo Localization (MCL)
- Optional particle filter that uses the distance sensors to correct odometry continuously while the robot drives.
- It accounts for distance sensor delay, ignores readings blocked by robots and game elements, and recovers if the robot gets shoved.
- Moves the position gently, so motions never see a sudden jump.

### ⏱️ Async Motions and Wait Helpers
- Run any motion in the background with `runAsync()`, then start mechanisms partway through it.
- `cancelMotion()` stops a motion early, e.g. when a sensor sees a game element.
- Optional stall exit ends a motion early when the robot is stuck.

### 🗂️ Auton Selector
- Pick a routine on the Brain screen, or with the controller's left and right arrows while the robot is disabled.
- The choice is saved on the SD card, so it survives a program restart.

### 🎮 Driver Control Helpers
- Tank, arcade, and split arcade with an exponential drive curve and a deadband.

### 📈 Motion Profiling
- Trapezoidal speed profiles for straight drives and turns: speed up, cruise, and slow down to stop on target.
- Uses a feedforward model of your drive (kS, kV, kA), measured with one routine or estimated from your motors and wheels.

### 🛣️ Path Following
- Smooth paths through waypoints (centripetal Catmull-Rom splines), or paste a path drawn in path.jerryio.
- Plans the speed along the path, slowing down for curves.
- **Pure pursuit** chases a point ahead on the path. It's forgiving and follows the path's shape.
- **RAMSETE** follows the path on a timetable, so timing is very repeatable.

### 🖥️ Simulator
- Runs your real RW code and autons on a computer, with a simulated robot, sensors, and Override field.
- Writes a replay page showing where the robot really went next to where RW thought it was.
- Optional: calibrate it from a short recorded run on your robot, so it speeds up, turns, slips, and stops like your robot.

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
   - New settings have defaults, so an older **custom** folder keeps building. To use new features, copy the new parts of the template's `custom/src/user.cpp` and `custom/src/robot-config.cpp` into yours

---

## ⚡ Quick Start

New to RW-Template? Do these in order. You only edit files in the `custom/` folder.

1. **Ports:** in `custom/src/robot-config.cpp`, set each motor and sensor to its port (a new device also needs a line in `custom/include/robot-config.h`). Reverse drive motors as needed so `driveChassis(12, 12)` drives forward
2. **Measurements:** set `distance_between_wheels` and `wheel_distance_in` (see [Tuning Your Robot](#3-tuning-your-robot))
3. **Driver control:** download the program, run it, and drive. Tank drive is the default
4. **First auton:** pick **Tuning Test** on the Brain screen and run autonomous (with a competition switch, field control, or **Timed Run**). It drives 60" and turns. Tune the PID constants until it stops on target
5. **Your autons:** write a routine in `custom/src/autonomous.cpp`, declare it in `custom/include/autonomous.h`, and add it to the selector with `addAuton()` in `custom/src/user.cpp`. [Which motion should I use?](#which-motion-should-i-use) has the options
6. **Optional:** try routines in the [simulator](#8-simulator) before running them on the robot

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
- `mcl.h` — Monte Carlo localization (particle filter)  
- `path.h` — Motion profile and path math  
- `path-following.h` — Profiled moves and path followers  
- `drive-log.h` — Drive calibration for the simulator  
- `auton-selector.h` — Auton selector  
- `driver-control.h` — Driver control helpers  
- `vex.h` — Standard VEX libraries and device setup

**src/**
- `main.cpp` — Competition entry point and control flow  
- `motor-control.cpp` — Core drive and motion control function implementations  
- `pid.cpp` — PID controller logic  
- `utils.cpp` — Math and geometry utility function implementations (odometry and distance reset math)  
- `mcl.cpp` — Particle filter math  
- `path.cpp` — Motion profiles, path smoothing, speed planning, pure pursuit and RAMSETE math  
- `path-following.cpp` — `driveProfiled`, `turnProfiled`, `followPath`, `followPathRamsete`, `measureDriveFeedforward`  
- `drive-log.cpp` — `calibrateDrive`, which records a run for the simulator  
- `auton-selector.cpp` — Auton selector  
- `driver-control.cpp` — Driver control helpers

**tests/**
- Computer-side tests for the odometry and distance reset math, plus an MCL field simulation (not part of the robot build)

**sim/**
- The simulator (not part of the robot build): `run-sim.sh` runs it, `viewer.html` is the replay page, `drive-model.h` is the drive physics, `fit-profile.sh` calibrates it from your robot, `find-compiler.sh` finds a C++ compiler for the scripts, and `vex/` holds the stand-in VEX API

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

If using distance resets or MCL:

- Configure your ports to the prebuilt sensor declarations (DO NOT CHANGE THE NAMES)
- Set each sensor's offsets in inches: where the sensor's face is relative to the robot's center (X: + is right, Y: + is forward)
- Use field-centered coordinates: the origin is the field center and the walls are at ±70.25". Start each auton with `setPose(x, y, heading)` at the robot's real field position

Newer settings (in the **NEWER SETTINGS** block of `custom/src/robot-config.cpp`). Every one works at its default, so leave them alone until you use the feature they're for:

- `imu_scale`: corrects an IMU that reads a little off over full turns. Run `measureImuScale(10)`, spin the robot 10 turns by hand, and copy the number it shows
- `stall_exit_msec`: ends a motion once the robot has been stuck that long (0 = off; 350 or more avoids false exits). Leave it off for moves meant to push into something
- `distance_reset_max_angle`, `distance_reset_max_jump`, `distance_reset_corner_margin`: which distance readings resets and MCL trust
- `mcl_particles`, `mcl_max_range`, `distance_sensor_delay_msec`: MCL settings
- `distance_sensor_scale`: corrects distance sensors that read a little long (many read 1-3% long). Put the robot a tape-measured distance from a wall and use measured ÷ reading
- `driver_curve`, `driver_deadband`: driver control feel
- `drive_kS`, `drive_kV`, `drive_kA`: the drive model for motion profiles and paths. Run `measureDriveFeedforward()` with about 4 ft of space in front and behind, and copy the numbers it shows. Leave `drive_kV` and `drive_kA` at 0 to estimate them from `drive_motor_rpm` and `wheel_distance_in`
- `profile_max_speed`, `profile_max_accel`, `profile_turn_speed`, `profile_turn_accel`: limits for profiled moves and paths. Lower `profile_max_accel` if the wheels slip
- `path_lookahead`, `path_max_lateral_accel`: pure pursuit lookahead, and how much to slow down in curves

### 4. Autonomous Programming

Edit `custom/src/autonomous.cpp` to define your autonomous routines.

#### Which motion should I use?

| I want to... | Use |
| --- | --- |
| Drive straight (negative = backward) | `driveTo(24, 2000);` |
| Turn to a heading | `turnToAngle(90, 1000);` |
| Turn to face a point | `turnToPoint(24, 24, 1, 1000);` |
| Swing turn (pivot on one side) | `swing(45, 1, 1000);` |
| Drive an arc (radius + curves right, - curves left) | `curveCircle(90, 24, 2000);` |
| Drive to a point | `moveToPoint(24, 48, 1, 2000);` |
| Drive to a point and end facing a heading | `boomerang(24, 48, 1, 90, 0.5, 3000);` |
| Speed up and slow down smoothly | `driveProfiled(36, 2000);` and `turnProfiled(90, 1000);` ([Motion profiles](#motion-profiles)) |
| Follow a curvy path | `followPath(path, 5000);` ([Path following](#path-following)) |
| Run a mechanism partway through a move | `runAsync()` and the wait helpers ([Async motions](#async-motions)) |
| Set where the robot starts | `setPose(x, y, heading);` |

Every motion takes the same kinds of settings:

- `time_limit_msec`: the motion gives up after this long, so a stuck robot can't freeze your auton
- `dir` (`moveToPoint`, `boomerang`, `turnToPoint`, `swing`): `1` drives forward, `-1` drives backward (back of the robot first)
- `exit` (default `true`): `true` stops on target. `false` ends early while still moving, so the next motion starts right away (motion chaining: faster, a little less exact)
- `max_output` (default 12): top motor voltage. 12 is full speed; lower it for gentler moves
- `driveTo` holds the heading the last motion ended on. To drive straight at another heading without turning first, set `correct_angle = heading;` before it

Positions are in inches and headings in degrees. Heading 0 faces +Y and turns clockwise, so 90 faces +X. When the program starts, the robot is at (0, 0) facing heading 0, so +Y is straight ahead and +X is to its right. To use field positions instead (needed for distance resets, MCL, and paths drawn in path.jerryio), start each auton with `setPose(x, y, heading)` at the robot's real spot: the field center is (0, 0) and the walls are at ±70.25".

Add each routine to the auton selector in `runPreAutonomous()` (`custom/src/user.cpp`). It holds up to 12:

```cpp
addAuton("Red Left", redLeft);
addAuton("Skills", skills);
startAutonSelector();
```

`runAutonomous()` calls `runSelectedAuton()`, which runs whichever routine is picked on the Brain screen.

#### Async motions

Start a motion in the background, then time mechanisms to it:

```cpp
runAsync([]{ moveToPoint(0, 48, 1, 3000); });
waitUntilTraveled(12);                 // 12 inches into the move...
intake_motor.spin(fwd, 12, volt);      // ...start the intake
waitUntil(!isMotionRunning() || intake_distance.objectDistance(mm) < 50);
cancelMotion();                        // stop early once the sensor sees a game element
waitForMotion();
```

- `waitUntilTraveled(inches)`, `waitUntilTurned(degrees)`, `waitUntilNear(x, y, radius)`: wait for progress in the running motion
- `waitForMotion()`: wait until no motion is running
- `cancelMotion()`: end the running motion as if it timed out
- A regular motion called while an async one is running waits for it, so two motions never fight over the drivetrain

#### Motion profiles

```cpp
driveProfiled(36, 3000);      // 36 in forward: smooth speed up, cruise, slow down
turnProfiled(90, 2000);       // Turn to 90 degrees the same way
driveProfiled(-24, 3000, false);  // exit = false: end still moving, to chain into the next motion
```

These use feedforward (volts from the speed you want) plus a little feedback, instead of a PID on distance. Measure your drive with `measureDriveFeedforward()` for the best results. The PID motions (`driveTo`, `turnToAngle`, ...) are unchanged, so use whichever you prefer.

#### Path following

```cpp
setPose(-36, -60, 0);   // Start at the path's first point, facing along it
Path out = pathFromPoints({{-36, -60}, {-36, -32}, {-12, -22}, {12, -30}, {34, -24}});
followPath(out, 5000);                   // Pure pursuit
Path back = pathFromPoints({{34, -24}, {12, -30}, {-12, -22}, {-36, -32}, {-36, -52}});
followPathRamsete(back, 5000, true);     // RAMSETE, driving back along it backward
```

- `pathFromPoints({{x, y}, ...})` makes a smooth curve through each point. Add a third number to slow down there, e.g. `{0, -12, 0.5}` for half speed
- Paths use the same positions as odometry, so start the auton with `setPose()` at the path's first point
- `followPath(path, time_limit, reverse, exit, max_speed, lookahead)`: pure pursuit. A bigger `path_lookahead` is smoother but cuts corners more
- `followPathRamsete(path, time_limit, reverse, exit, max_speed)`: RAMSETE keeps to a timetable, so it's the most repeatable
- Both end lined up with the path's last direction, usually within 1-2" of the end point and a few degrees of its heading. If the next step needs it exact, follow with `moveToPoint()`, `turnToAngle()`, or a distance reset
- Both work with `runAsync()`, the wait helpers, and `cancelMotion()`

#### Drawing paths in path.jerryio

[path.jerryio](https://path.jerryio.com) lets you draw paths on top of the field.

1. Set the format to **LemLib v0.5** (the text format) and pick the **V5RC 2027 - Override** field
2. Draw the path. Speeds you set in jerryio cap the speed along that part of the path. RW plans its own slowing for curves and for the stop at the end, from your robot's settings
3. Save the path file and open it in VS Code. The first line is the start point (x, y, speed)
4. Paste the whole file into your auton:

```cpp
setPose(-36, -60, 0);   // The file's first point, with the heading the robot starts at
Path drawn = pathFromJerryIO(R"(
-36.000, -60.000, 100.000
-36.000, -58.000, 100.000
...the rest of the file...
)");
followPath(drawn, 5000);
```

#### Localization: distance resets and MCL

- `resetPositionFront()` / `Back()` / `Left()` / `Right()` reset X or Y from one sensor. Each returns `true` if it reset
- MCL keeps correcting while the robot drives. Turn it on with `startMcl()` after `startOdometry()` in `runPreAutonomous()`. The selector screen shows its uncertainty and how long each update takes on the Brain. If that's more than a few msec, lower `mcl_particles`
- MCL is new and has only been tested in simulation (see Testing below). Try it on a practice field first, and watch the Brain screen to confirm the position stays right before using it in matches
- If the position lags while driving toward or away from a wall, adjust `distance_sensor_delay_msec`

Override (2026-27) tips:

- Set `distance_reset_corner_margin` to about 18 so the corner loaders don't throw off readings
- Wall centers are the cleanest place for resets; Cup+Pin stacks start against the walls about 24" either side of each center
- Robots can't be touching the perimeter at the end of auton for the Autonomous Win Point, so back off after resetting against a wall
- Rule R11 caps drivetrain motors at 55W total, and drive encoder readings come from the first motor in each `motor_group`. In a mixed 11W/5.5W drive, list an 11W motor first

### 5. Driver Control

Edit the `runDriver()` function in `custom/src/user.cpp`.

By default, it uses tank drive:

`driverTank(ch3, ch2);`

Other options:

- Arcade on the left stick: `driverArcade(ch3, ch4);`
- Split arcade: `driverArcade(ch3, ch1);`
- Adding button controls for mechanisms

`driver_curve` (0 = linear, 2-6 is typical) softens small stick movements without lowering top speed. `driver_deadband` ignores tiny stick drift.

### 6. Competition Setup

This template follows the VEX Competition structure:

- `pre_auton()` which calls `runPreAutonomous()` runs once at startup (sensor calibration, odometry, auton selector)  
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

### 8. Simulator

Run your autons on a computer and watch a replay, no robot needed.

**One-time setup: install a C++ compiler**

- **macOS:** open Terminal and run `xcode-select --install`
- **Windows:** the simulator runs in WSL (Linux inside Windows). Open PowerShell as administrator, run `wsl --install`, restart, and finish the Ubuntu setup that opens. Then, in VS Code's terminal, type `wsl` (the terminal is now Linux, still in your project folder) and run `sudo apt update && sudo apt install -y g++`. Some school laptops block WSL
- **Linux:** install `g++`, e.g. `sudo apt install g++`

**Run an auton** from the project folder (on Windows, type `wsl` in VS Code's terminal first):

```bash
sh sim/run-sim.sh "Async Example"
```

Use the name you gave `addAuton()`; `sh sim/run-sim.sh --list` lists them. It builds your current code (everything in `src/` and `custom/`, minus `src/main.cpp`), runs `runPreAutonomous()` and then your auton on a simulated robot, and writes a replay such as `sim/out/Async-Example.html`. Open it in any browser to play it back (in VS Code, right-click it and choose Reveal in Finder or Reveal in File Explorer, then double-click it). It shows the field, the path the robot really drove, the path RW thought it drove, and the error between them. The terminal prints the same summary.

Options:

- `--list`: list the autons added with `addAuton()`
- `--start X,Y,HEADING`: where the robot starts, in field-centered inches and compass degrees (default `-35,-60,0`). Use the same spot your auton assumes
- `--time SECONDS`: stop after this long (default 15; use 60 for skills)
- `--ideal`: no wheel slip, IMU error, or sensor noise
- `--walls`: empty field without Override's goals, loaders, and stacks
- `--size INCHES`, `--seed N`, `--out FILE`

What it simulates:

- **Robot:** a tank drive using your `robot-config.cpp` measurements, with motor lag, wheel slip, turning scrub, and sideways skid
- **Walls:** the robot stops at the perimeter but its wheels keep spinning, so odometry drifts like it does on a real field
- **Sensors:**
  - IMU with a small scale error and drift
  - tracking wheels
  - the four reset distance sensors, which update every 33 ms, lag about 40 ms, and see the walls, goals, loaders, and stacks
- **Code:** all of your code runs unchanged, including threads, `wait()`, MCL, and the auton selector

#### Make the simulator drive like your robot (optional)

By default the simulator uses a typical drive for your motors and wheels. To match your robot:

1. On the robot, run the **Drive Calibration** auton (`calibrateDrive()`) with about 4 ft clear on every side. It takes about 15 seconds and saves `rw_drive_log.csv` to the SD card. Without an SD card, it prints the log to the terminal; save that as the file
2. Copy `rw_drive_log.csv` from the SD card into the project folder, then run:

```bash
sh sim/fit-profile.sh rw_drive_log.csv
```

It writes `sim/robot-profile.txt`, and the simulator uses it from then on. Use `--no-profile` to compare with the default robot, or `--profile FILE` for another robot.

- It measures: friction, speed per volt, motor lag, the motors' acceleration limit, braking and coasting, and turning scrub (from the IMU)
- With both tracking wheels, it also measures wheel slip, the grip limit, and sideways skid. Without them, those stay at typical values
- Tiles, batteries and wheels change, so recalibrating takes 15 seconds

Limitations:

- The robot drives through goals and game elements; only the walls stop it
- Mechanisms just spin, controller inputs read as zero, and the Brain screen draws nothing
- The SD card is the folder `sim/out/sdcard`. The Drive Calibration auton saves its log there if you give it time: `--time 20`
- Distance sensors other than the four reset sensors read "nothing detected"

If your code uses a VEX function the simulator doesn't have yet, the build error names it. Add it to `sim/vex/v5_vcs.h`, or ask on Discord.

### 9. Testing on a Computer

`tests/` has tests that run on a computer, not the robot:

- the odometry and distance reset math
- the motion profile, path, pure pursuit and RAMSETE math
- a field simulation where MCL corrects drifting odometry with delayed, noisy sensors and Override field elements in the way
- a check that the simulator's calibration fitter recovers a made-up robot

With a C++ compiler installed (see [Simulator](#8-simulator)), run from the project folder:

```bash
sh tests/run-tests.sh
```

---

## 🙏 Credits

The newer features build on work others shared publicly:

- **Localization:** *Probabilistic Robotics* (Thrun, Burgard & Fox); MCL work by 2654E ([Echo](https://github.com/alexDickhans/echo)), [NlGanma](https://github.com/NlGanma/vex-v5-localization) and [1516X](https://github.com/calhighrobotics/push_back_x); jpearman's VEX Forum posts on the V5 sensors
- **Odometry and drive curve:** Team 5225A
- **Path following:** pure pursuit (Coulter, 1992), RAMSETE (De Luca, Oriolo & Vendittelli, 2001), and [path.jerryio](https://github.com/Jerrylum/path.jerryio)
- **Ideas from other libraries:** [LemLib](https://github.com/LemLib/LemLib), [EZ-Template](https://github.com/EZ-Robotics/EZ-Template), [JAR-Template](https://github.com/JacksonAreaRobotics/JAR-Template), [mikLib](https://github.com/EthanMik/mikLib) and [mclib](https://github.com/kevinyhe/mclib)

Thanks also to everyone who has contributed to RW-Template on GitHub.

---

This template is **competition-ready** and provides a strong foundation for building reliable, high-performance autonomous routines using **proven robotics algorithms**, whether or not your robot uses advanced odometry. 

If you find this template useful, please star this repository and subscribe to https://www.youtube.com/@1698V to follow my team's progress and learn more about our programming. 

