# Motion Profiles

Profiled moves follow a trapezoidal speed profile: speed up, cruise, and slow down to stop on target.

```cpp
driveProfiled(36, 3000);      // 36 in forward: smooth speed up, cruise, slow down
turnProfiled(90, 2000);       // Turn to 90 degrees the same way
turnProfiled(180, clockwise, 2000);  // Turn to 180 turning right, even if left is a little shorter
driveProfiled(-24, 3000, false);  // exit = false: end still moving, to chain into the next motion
```

These use feedforward (volts from the speed you want) plus a little feedback, instead of a PID on distance.

For the best results, measure your drive's feedforward model (`drive_kS`, `drive_kV`, `drive_kA`) with `measureDriveFeedforward()`. See [Tuning](tuning.md#newer-settings).

The PID motions (`driveTo`, `turnToAngle`, ...) are unchanged, so use whichever you prefer.

## In the simulator

The template's **Profiled Example** auton:

<div class="rw-replay" data-src="../../replays/Profiled-Example.html" data-title="Profiled Example"></div>
