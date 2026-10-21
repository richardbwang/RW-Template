# Async Motions

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

For a robot that gets stuck, set `stall_exit_msec` (see [Tuning](tuning.md#newer-settings)) to end a motion early once the robot stops moving.

## In the simulator

The template's **Async Example** auton:

<div class="rw-replay" data-src="../../replays/Async-Example.html" data-title="Async Example"></div>
