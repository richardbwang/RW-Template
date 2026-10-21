# Autonomous Programming

Write your autonomous routines in `custom/src/autonomous.cpp`, and declare each one in `custom/include/autonomous.h`.

## Which motion should I use?

| I want to... | Use |
| --- | --- |
| Drive straight (negative = backward) | `driveTo(24, 2000);` |
| Turn to a heading | `turnToAngle(90, 1000);` |
| Turn a set way, like a 180° turn that has to go right | `turnToAngle(180, clockwise, 1000);` (or `counterclockwise`) |
| Turn to face a point | `turnToPoint(24, 24, 1, 1000);` |
| Swing turn (pivot on one side) | `swing(45, 1, 1000);` |
| Drive an arc (radius + curves right, - curves left) | `curveCircle(90, 24, 2000);` |
| Drive to a point | `moveToPoint(24, 48, 1, 2000);` |
| Drive to a point and end facing a heading | `boomerang(24, 48, 1, 90, 0.5, 3000);` |
| Speed up and slow down smoothly | `driveProfiled(36, 2000);` and `turnProfiled(90, 1000);` ([Motion profiles](motion-profiles.md)) |
| Follow a curvy path | `followPath(path, 5000);` ([Path following](path-following.md)) |
| Run a mechanism partway through a move | `runAsync()` and the wait helpers ([Async motions](async-motions.md)) |
| Strafe (X-drive, mecanum, H-drive) | `strafe(24, 2000);` and `strafeToPose(24, 48, 90, 3000);` ([Drive types](drive-types.md)) |
| Set where the robot starts | `setPose(x, y, heading);` |

`curveCircle` works without tracking wheels or full odometry. It uses heading and distance PID to hold the curve.

## Motion settings

Every motion takes the same kinds of settings:

- `time_limit_msec`: the motion gives up after this long, so a stuck robot can't freeze your auton
- `dir` (`moveToPoint`, `boomerang`, `turnToPoint`, `swing`): `1` drives forward, `-1` drives backward (back of the robot first)
- `exit` (default `true`): `true` stops on target. `false` ends early while still moving, so the next motion starts right away (motion chaining: faster, a little less exact)
- `max_output` (default 12): top motor voltage. 12 is full speed; lower it for gentler moves
- Turn direction (`turnToAngle`, `turnProfiled`): turns go the short way. For a turn of about 180°, the short way can go either way depending on a fraction of a degree of heading, so pass `clockwise` (right) or `counterclockwise` (left) after the angle: `turnToAngle(180, clockwise, 1000);`. A robot already up to 5° past the target still turns the short way, so it doesn't spin almost all the way around
- `driveTo` holds the heading the last motion ended on. To drive straight at another heading without turning first, set `correct_angle = heading;` before it

## Coordinates

Positions are in inches and headings in degrees. Heading 0 faces +Y and turns clockwise, so 90 faces +X.

When the program starts, the robot is at (0, 0) facing heading 0, so +Y is straight ahead and +X is to its right.

To use field positions instead (needed for [distance resets, MCL](localization.md), and [paths drawn in path.jerryio](path-following.md#drawing-paths-in-pathjerryio)), start each auton with `setPose(x, y, heading)` at the robot's real spot: the field center is (0, 0) and the walls are at ±70.25".

## Auton selector

Add each routine to the auton selector in `runPreAutonomous()` (`custom/src/user.cpp`). It holds up to 12:

```cpp
addAuton("Red Left", redLeft);
addAuton("Skills", skills);
startAutonSelector();
```

Pick a routine on the Brain screen, or with the controller's left and right arrows while the robot is disabled. The choice is saved on the SD card, so it survives a program restart.

`runAutonomous()` calls `runSelectedAuton()`, which runs whichever routine is picked.
