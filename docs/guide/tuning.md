# Tuning Your Robot

In `custom/src/robot-config.cpp`, find the section labeled **USER-CONFIGURABLE PARAMETERS** and set:

- `distance_between_wheels`: distance between the left and right wheels, in inches
- `wheel_distance_in`: wheel circumference (the comments in the code help)
- X-drive, mecanum or H-drive? Those two work a little differently, and there's also `strafe_distance_in`. See [Drive Types](drive-types.md#x-drive-and-mecanum)
- PID constants: `distance_kp`, `distance_ki`, `distance_kd`, and the rest

Then run the **Tuning Test** auton (it drives 60" and turns) and adjust the PID constants until it stops on target. Test motions like `driveTo`, `turnToAngle` and `moveToPoint` one at a time.

!!! tip "New to PID?"
    Try the [PID Playground](pid-playground.md) first. Drag kP, kI and kD and watch a simulated robot turn, to see what each one does before you tune the real robot.

## Newer settings

These are in the **NEWER SETTINGS** block of `custom/src/robot-config.cpp`. Every one works at its default, so leave them alone until you use the feature they're for.

| Setting | What it does |
| --- | --- |
| `imu_scale` | Corrects an IMU that reads a little off over full turns. Run `measureImuScale(10)`, spin the robot 10 turns by hand, and copy the number it shows |
| `stall_exit_msec` | Ends a motion once the robot has been stuck that long (0 = off; 350 or more avoids false exits). Leave it off for moves meant to push into something |
| `distance_reset_max_angle`, `distance_reset_max_jump`, `distance_reset_corner_margin` | Which distance readings resets and MCL trust |
| `mcl_particles`, `mcl_max_range`, `distance_sensor_delay_msec` | MCL settings |
| `distance_sensor_scale` | Corrects distance sensors that read a little long (many read 1-3% long). Put the robot a tape-measured distance from a wall and use measured ÷ reading |
| `driver_curve`, `driver_deadband` | [Driver control](driver-control.md) feel |
| `strafe_distance_in` | X-drives, mecanum drives and H-drives only: how far the robot moves sideways per turn of a strafing motor. See [Drive Types](drive-types.md) |
| `drive_kS`, `drive_kV`, `drive_kA` | The drive model for [motion profiles](motion-profiles.md) and [paths](path-following.md). Run `measureDriveFeedforward()` with about 4 ft of space in front and behind, and copy the numbers it shows. Leave `drive_kV` and `drive_kA` at 0 to estimate them from `drive_motor_rpm` and `wheel_distance_in` |
| `profile_max_speed`, `profile_max_accel`, `profile_turn_speed`, `profile_turn_accel` | Limits for profiled moves and paths. Lower `profile_max_accel` if the wheels slip |
| `path_lookahead`, `path_max_lateral_accel` | Pure pursuit lookahead, and how much to slow down in curves |

!!! tip
    Read the comments in each file. They explain how each function and variable works.
