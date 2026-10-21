# Robot Configuration

Edit `custom/src/robot-config.cpp` and `custom/include/robot-config.h` to match your robot's hardware.

- Set the correct ports for motors, sensors, and other robot devices
- Group drive motors into the `left_chassis` and `right_chassis` motor groups
- Reverse drive motors as needed so `driveChassis(12, 12)` drives forward
- X-drive, mecanum or H-drive? See [Drive Types](drive-types.md)

## Tracking wheels

RW works with no tracking wheels, a vertical one, a horizontal one, or both. Without them it falls back to the drive encoders and the IMU.

If you use a horizontal and/or vertical tracking wheel:

- Set `using_horizontal_tracker = true` and/or `using_vertical_tracker = true`
- Configure the tracker distances and diameters

## Distance sensors

For [distance resets or MCL](localization.md):

- Configure your ports on the prebuilt sensor declarations. **Don't change their names**
- Set each sensor's offsets in inches: where the sensor's face is relative to the robot's center (X: + is right, Y: + is forward)
- Use field-centered coordinates: the origin is the field center and the walls are at ±70.25". Start each auton with `setPose(x, y, heading)` at the robot's real field position

!!! tip "Override (2026-27)"
    Rule R11 caps drivetrain motors at 55W total, and drive encoder readings come from the first motor in each `motor_group`. In a mixed 11W/5.5W drive, list an 11W motor first.

Next: [Tuning Your Robot](tuning.md).
