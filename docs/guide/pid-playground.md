---
description: Drag kP, kI and kD and watch a simulated robot run RW's turnToAngle.
---

# PID Playground

Drag the sliders and watch a simulated robot run `turnToAngle()`. It uses the same PID math and exit rules as RW, so numbers that work here are a good starting point on your robot.

<div class="pid-playground"></div>

!!! note
    This is a simple model of a robot, not yours. Use it to get a feel for what each constant does, then fine-tune on the real robot with the **Tuning Test** auton ([Tuning Your Robot](tuning.md)).

## What each one does

- **kP (proportional)** pushes harder the farther the robot is from the target. Too little and it crawls or stops short. Too much and it swings past the target.
- **kD (derivative)** works against how fast the error is shrinking, so it brakes as the robot closes in. It's what stops the overshoot. Too much and the robot creeps toward the end.
- **kI (integral)** adds up error that's left over near the end and nudges the robot the last degree or two. In `turnToAngle` it only builds up within 3° of the target and resets if the robot passes it, so it can't cause a big swing. Most robots need little or none.

## Try these

1. Click **No kD** and watch the robot wobble past the target. Raise kD until it stops cleanly
2. Click **Too much kP**. Can you fix it with kD alone, without lowering kP?
3. Switch to **Heavy pusher**. RW's defaults overshoot on it. Find new numbers that settle in under 1 second
4. Click **Too gentle**. The robot stalls short of the target, because a small push can't beat friction. Raise kP until it gets there

## The same idea for driving

`driveTo()` uses the same PID with `distance_kp`, `distance_ki` and `distance_kd` (error in inches instead of degrees), and `heading_correction_kp` and friends keep the robot straight while it drives. Tune turning first, then driving, with the **Tuning Test** auton.
