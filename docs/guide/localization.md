# Localization: Distance Resets and MCL

Odometry drifts over an auton. RW can correct it with distance sensors in two ways. Both need the sensors set up as in [Robot Configuration](configuration.md#distance-sensors), and field-centered positions from `setPose()`.

## Distance resets

`resetPositionFront()`, `resetPositionBack()`, `resetPositionLeft()` and `resetPositionRight()` reset X or Y from one sensor. Each returns `true` if it reset.

- Use sensors on all sides, or only the sides you want
- Each reset traces the sensor's beam to the wall it actually hits, so angled readings and readings near corners work
- It skips readings that hit the wall too steeply, land too close to a corner (e.g. Override's loaders), or disagree too much with odometry

## Monte Carlo Localization (MCL)

MCL is an optional particle filter that keeps correcting odometry while the robot drives.

- Turn it on with `startMcl()` after `startOdometry()` in `runPreAutonomous()`
- It accounts for distance sensor delay, ignores readings blocked by robots and game elements, and recovers if the robot gets shoved
- It moves the position gently, so motions never see a sudden jump
- The selector screen shows its uncertainty and how long each update takes on the Brain. If that's more than a few msec, lower `mcl_particles`
- If the position lags while driving toward or away from a wall, adjust `distance_sensor_delay_msec`

!!! warning "MCL is new"
    It has only been tested in simulation (see [Testing on a Computer](../simulator/testing.md)). Try it on a practice field first, and watch the Brain screen to confirm the position stays right before using it in matches.

## Override (2026-27) tips

- Set `distance_reset_corner_margin` to about 18 so the corner loaders don't throw off readings
- Wall centers are the cleanest place for resets; Cup+Pin stacks start against the walls about 24" either side of each center
- Robots can't be touching the perimeter at the end of auton for the Autonomous Win Point, so back off after resetting against a wall
- Rule R11 caps drivetrain motors at 55W total, and drive encoder readings come from the first motor in each `motor_group`. In a mixed 11W/5.5W drive, list an 11W motor first
