# Competition Setup

The template follows the VEX Competition structure:

- `pre_auton()`, which calls `runPreAutonomous()`, runs once at startup (sensor calibration, odometry, auton selector)
- `autonomous()`, which calls `runAutonomous()`, runs during the autonomous period
- `usercontrol()`, which calls `runDriver()`, runs during the driver control period
- `main()` connects all of these automatically

## Tips for success

- Read the comments in each file. They explain how each function and variable works
- Tune your PID values
- Test motions like `driveTo`, `turnToAngle`, and `moveToPoint` one at a time
- Use odometry for advanced navigation, with or without tracking wheels
- Try routines in the [simulator](../simulator/running.md) before running them on the robot
- Ask for help on [our Discord](https://discord.gg/9FHyyFe22p) or from the VEX community
