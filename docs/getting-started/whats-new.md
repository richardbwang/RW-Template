# What's New

!!! note "These features are new"
    They pass computer tests and run in the simulator, but haven't been tried on many real robots yet. If something doesn't work the way these docs say, tell us on [Discord](https://discord.gg/9FHyyFe22p) or [open a GitHub issue](https://github.com/richardbwang/RW-Template/issues).

## 4.1.0

- **[PROS version](pros.md):** RW-Template now also comes as a PROS template. Add it to a new or existing PROS project with two commands, and update it with one. The motions, settings, odometry, auton selector and simulator are the same as in VEXcode.
- **This website:** a guide for every feature in one place, plus the [PID Playground](../guide/pid-playground.md), where you can try PID tuning in your browser, and a [replay gallery](../simulator/replays.md) of the example autons in the simulator.
- **[One-click updates](installation.md#update-from-an-older-version):** in VS Code's menu bar, **Terminal > Run Task... > Update RW-Template** downloads the newest version and replaces RW's files. Your `custom` folder stays as it is.
- **[X-drive, mecanum and H-drive](../guide/drive-types.md):** every motion works on them, and `strafe()`, `strafeToPose()` and the holonomic driver controls, including field-centric, can strafe.
- **[Turn direction](../guide/autonomous.md#motion-settings):** `turnToAngle(180, clockwise, 1000)` always turns right, even when turning left is a hair shorter, so a 180° turn can't go the wrong way. `turnProfiled()` takes it too.
- **[Simulator](../simulator/running.md):** it now runs PROS projects, and X-drives, mecanum drives and H-drives.
- **Driver control fix:** the template now reads the sticks with `.position()`, which goes from -100 to 100, the range RW's driver functions expect. 4.0.0 used `.value()`, which goes to 127, so the robot reached full speed with the stick only about 80% of the way over.

### Updating from 4.0.0

1. 4.0.0 projects don't have the update task yet, so [update by hand once](installation.md#projects-from-400-and-older). After that, the task does it
2. For the driver control fix, change `.value()` to `.position()` on the four `controller_1.Axis` lines in `runDriver()` in `custom/src/user.cpp` (the by-hand steps include this). Updates never change your `custom` folder, so they can't do it for you
3. Everything else is optional. Your `custom` folder keeps building, and new settings have defaults

## 4.0.0

- **Odometry fixes.** Without tracking wheels, every 90° turn in place used to add about 8.7" of position error. With only a vertical tracking wheel, X and Y were swapped. Curves were also slightly off with tracking wheels. All four setups now share one tested implementation.
- **[Async motions](../guide/async-motions.md):** run a motion in the background and use `waitUntilTraveled()`, `waitUntilTurned()`, `waitUntilNear()`, `waitForMotion()` and `cancelMotion()` to time mechanisms to it.
- **[Monte Carlo Localization (MCL)](../guide/localization.md):** optional. It uses your distance sensors to keep odometry from drifting while the robot drives.
- **Smarter distance resets:** they trace the sensor beam to the wall it actually hits and skip steep, near-corner, or suspicious readings.
- **Auton selector** on the Brain screen and the controller, remembered on the SD card.
- **[Driver curves](../guide/driver-control.md)** (exponential, with deadband) for tank, arcade, and split arcade.
- **IMU scale correction**, a **stall exit** for stuck motions, and `setPose()`.
- **[Simulator](../simulator/running.md):** run your autons on a computer and watch a replay on an Override field, no robot needed. Optionally [calibrate it](../simulator/calibration.md) from a 15-second run on your robot so it drives like yours.
- **[Motion profiling](../guide/motion-profiles.md):** `driveProfiled()` and `turnProfiled()` speed up, cruise and slow down smoothly using a feedforward model of your drive.
- **[Path following](../guide/path-following.md):** smooth paths through waypoints (or drawn in path.jerryio), followed with pure pursuit or RAMSETE.

Updating from an older version? Your old `custom` folder still builds, because every new setting has a default. To use the new features, copy the new parts of `custom/src/user.cpp` and `custom/src/robot-config.cpp` into yours. See [Installation and Updating](installation.md#update-from-an-older-version).
