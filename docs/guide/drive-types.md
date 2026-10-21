# Drive Types

RW-Template is set up for a tank drive (any number of wheels, omni or traction). It also runs X-drives, mecanum drives and H-drives. X-drives and mecanum drives are *holonomic*: they can move in any direction without turning, including straight sideways, which is called *strafing*. An H-drive is a tank drive with an extra sideways wheel, so it can strafe too.

Every RW motion works on these drives, driving like a tank drive. A few more can strafe: `strafe()`, `strafeToPose()` and the holonomic driver controls.

## X-drive and mecanum

Each wheel has its own motor. Make the four corner motors, each reversed so it drives the robot forward at positive voltage, the same way as a tank drive's. Put the left two in `left_chassis` and the right two in `right_chassis`, then tell RW which motor is which corner.

=== "VEXcode"

    In `custom/src/robot-config.cpp`, replace the six drive motors (`left_chassis1` to `right_chassis3`) and the two `motor_group` lines with:

    ```cpp
    motor left_front = motor(PORT1, ratio18_1, true);
    motor left_back = motor(PORT2, ratio18_1, true);
    motor right_front = motor(PORT3, ratio18_1, false);
    motor right_back = motor(PORT4, ratio18_1, false);
    motor_group left_chassis = motor_group(left_front, left_back);
    motor_group right_chassis = motor_group(right_front, right_back);
    ```

    In `custom/include/robot-config.h`, replace the six `extern motor` lines for the old drive motors with:

    ```cpp
    extern motor left_front;
    extern motor left_back;
    extern motor right_front;
    extern motor right_back;
    ```

    In `runPreAutonomous()` in `custom/src/user.cpp`, add this line just above the `startOdometry();` that's already there:

    ```cpp
    useHolonomicDrive(left_front, right_front, left_back, right_back);
    ```

=== "PROS"

    In `src/rw-config.cpp`, replace the two `MotorGroup` lines with:

    ```cpp
    pros::Motor left_front(-1, pros::MotorGearset::green);  // A negative port reverses a motor
    pros::Motor left_back(-2, pros::MotorGearset::green);
    pros::Motor right_front(3, pros::MotorGearset::green);
    pros::Motor right_back(4, pros::MotorGearset::green);
    pros::MotorGroup left_chassis({-1, -2}, pros::MotorGearset::green);
    pros::MotorGroup right_chassis({3, 4}, pros::MotorGearset::green);
    ```

    In `include/rw-config.h`, with your own devices:

    ```cpp
    extern pros::Motor left_front;
    extern pros::Motor left_back;
    extern pros::Motor right_front;
    extern pros::Motor right_back;
    ```

    In `runPreAutonomous()` in `src/rw-user.cpp`, add this line just above the `startOdometry();` that's already there:

    ```cpp
    useHolonomicDrive(left_front, right_front, left_back, right_back);
    ```

Then set these in the settings (`custom/src/robot-config.cpp`, or `src/rw-config.cpp` on PROS):

| Setting | X-drive | Mecanum |
| --- | --- | --- |
| `wheel_distance_in` | gear ratio × wheel diameter × π × 0.707. The wheels sit at 45°, so the robot moves 0.707 of what each wheel rolls | gear ratio × wheel diameter × π |
| `distance_between_wheels` | Between the left and right wheels | Between the left and right wheels, **plus** between the front and back wheels |
| `strafe_distance_in` | Leave at 0 (the same as `wheel_distance_in`) | A little less than `wheel_distance_in`, because mecanum wheels slip sideways. Start with `wheel_distance_in * 0.85` and measure it |

No `strafe_distance_in` line in your settings? Projects from 4.0.0 don't have one, so add `double strafe_distance_in = 0;` next to the others.

To measure `strafe_distance_in`, make an auton that runs `strafe(48, 3000);`, run it, and tape-measure how far the robot really went sideways. Then multiply `strafe_distance_in` (or `wheel_distance_in`, if it's 0) by that distance and divide by 48.

Wheels slip more when they strafe, mecanum ones especially, so tracking wheels help a lot here. With a horizontal tracking wheel, RW tracks sideways motion with it and `strafe_distance_in` stops mattering. Without one, RW works out the sideways motion from the four wheel encoders.

## H-drive

Keep `left_chassis` and `right_chassis` as they are, add the strafe wheel's motor (reversed so positive voltage moves the robot right), and tell RW about it.

=== "VEXcode"

    In `custom/src/robot-config.cpp`:

    ```cpp
    motor strafe_motor = motor(PORT8, ratio18_1, false);  // Your strafe motor's port
    ```

    In `custom/include/robot-config.h`:

    ```cpp
    extern motor strafe_motor;
    ```

    In `runPreAutonomous()` in `custom/src/user.cpp`, just above `startOdometry();`:

    ```cpp
    useHDrive(strafe_motor);
    ```

=== "PROS"

    In `src/rw-config.cpp`:

    ```cpp
    pros::Motor strafe_motor(8, pros::MotorGearset::green);  // Your strafe motor's port
    ```

    In `include/rw-config.h`:

    ```cpp
    extern pros::Motor strafe_motor;
    ```

    In `runPreAutonomous()` in `src/rw-user.cpp`, just above `startOdometry();`:

    ```cpp
    useHDrive(strafe_motor);
    ```

Set `strafe_distance_in` to the strafe wheel's gear ratio × wheel diameter × π.

## Strafing in autonomous

| I want to... | Use |
| --- | --- |
| Strafe sideways, keeping the heading (negative = left) | `strafe(24, 2000);` |
| Drive straight to a point while turning to a heading | `strafeToPose(24, 48, 90, 3000);` |
| Drive, strafe and turn with my own voltages | `driveHolonomic(forward_volts, strafe_volts, turn_volts);` |

`strafeToPose` and `strafe` take the same `exit` and `max_output` settings as the other motions, and use the distance and turn PID constants. On a tank drive, `strafeToPose` (and `strafe`, which uses it) drives to the point and then turns to the heading, so the same auton still runs.

## Driver control

In `runDriver()`, replace the `driverTank(...)` line with this. The left stick drives and strafes, and the right stick turns:

=== "VEXcode"

    ```cpp
    driverHolonomic(ch3, ch4, ch1);
    ```

=== "PROS"

    ```cpp
    driverHolonomic(stick(pros::E_CONTROLLER_ANALOG_LEFT_Y), stick(pros::E_CONTROLLER_ANALOG_LEFT_X),
                    stick(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    ```

    `stick()` is in `src/rw-user.cpp`. It turns PROS's stick values (-127 to 127) into the -100 to 100 that RW uses.

`driverHolonomic()` drives relative to the robot: pushing the stick forward drives the way the robot faces. `driverFieldCentric()` takes the same sticks but drives relative to the field (*field-centric*): pushing the stick forward always drives the same way on the field, whichever way the robot faces. Many drivers find that easier on an X-drive or mecanum drive.

For `driverFieldCentric()`, "forward" is the heading in `driver_forward_heading`: 0 is the way the robot faced when the program started (or at `setPose(..., 0)`). To set it during a match, point the robot away from you and press a button. For example, Y, inside the loop in `runDriver()`:

=== "VEXcode"

    ```cpp
    if (button_y) driver_forward_heading = getInertialHeading();
    ```

=== "PROS"

    ```cpp
    if (controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) driver_forward_heading = getInertialHeading();
    ```

`driver_curve` and `driver_deadband` work the same as for tank and arcade.

## Simulator

The [simulator](../simulator/running.md) drives these too. It reads the setup from `runPreAutonomous()` and prints `Robot: holonomic drive` or `Robot: H-drive` at the start.
