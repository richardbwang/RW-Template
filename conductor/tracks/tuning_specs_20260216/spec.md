# Track: Tuning Robot Physical Constants and PID

## Objective
Tune the robot's physical constants and PID values to ensure precise autonomous movements and reliable odometry.

## Scope
- Measure and update `distance_between_wheels` for accurate turning.
- Confirm and update `wheel_distance_in` based on physical wheel circumference and gear ratio.
- Tune linear PID (`distance_kp`, `distance_ki`, `distance_kd`) for straight driving.
- Tune turning PID (`turn_kp`, `turn_ki`, `turn_kd`) for precise headings.
- Tune heading correction PID for maintaining straight paths.
- Test and verify boomerang and curveCircle movements after tuning.

## Requirements
- Final PID values must result in < 1 inch error for linear moves and < 1 degree error for turns.
- Physical constants must match the real-world robot dimensions.
