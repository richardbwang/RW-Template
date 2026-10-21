# Calibrating to Your Robot

By default the simulator uses a typical drive for your motors and wheels. Calibrating makes it speed up, turn, slip, and stop like your robot. It's optional.

1. On the robot, run the **Drive Calibration** auton (`calibrateDrive()`) with about 4 ft clear on every side. It takes about 15 seconds and saves `rw_drive_log.csv` to the SD card. Without an SD card, it prints the log to the terminal; save that as the file
2. Copy `rw_drive_log.csv` from the SD card into the project folder, then run:

    ```bash
    sh sim/fit-profile.sh rw_drive_log.csv
    ```

It writes `sim/robot-profile.txt`, and the simulator uses it from then on. Use `--no-profile` to compare with the default robot, or `--profile FILE` for another robot.

- It measures friction, speed per volt, motor lag, the motors' acceleration limit, braking and coasting, and turning scrub (from the IMU)
- With both tracking wheels, it also measures wheel slip, the grip limit, and sideways skid. Without them, those stay at typical values
- Tiles, batteries and wheels change, and recalibrating only takes 15 seconds
