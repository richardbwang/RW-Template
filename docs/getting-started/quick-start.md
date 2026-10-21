# Quick Start

New to RW-Template? Do these in order. You only edit files in the `custom/` folder.

1. **Ports:** in `custom/src/robot-config.cpp`, set each motor and sensor to its port (a new device also needs a line in `custom/include/robot-config.h`). Reverse drive motors as needed so `driveChassis(12, 12)` drives forward. X-drive, mecanum or H-drive? Set it up as in [Drive Types](../guide/drive-types.md), which also covers its measurements
2. **Measurements:** set `distance_between_wheels` and `wheel_distance_in` (see [Tuning Your Robot](../guide/tuning.md))
3. **Driver control:** download the program, run it, and drive. Tank drive is the default
4. **First auton:** pick **Tuning Test** on the Brain screen and run autonomous (with a competition switch, field control, or **Timed Run**). It drives 60" and turns. Tune the PID constants until it stops on target
5. **Your autons:** write a routine in `custom/src/autonomous.cpp`, declare it in `custom/include/autonomous.h`, and add it to the selector with `addAuton()` in `custom/src/user.cpp`. [Which motion should I use?](../guide/autonomous.md#which-motion-should-i-use) has the options
6. **Optional:** try routines in the [simulator](../simulator/running.md) before running them on the robot

This is what Tuning Test looks like in the simulator:

<div class="rw-replay" data-src="../../replays/Tuning-Test.html" data-title="Tuning Test"></div>
