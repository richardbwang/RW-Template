#include "hardware.h"
#include <cmath>
#include <stdio.h>
#include "utils.h"
#include "pid.h"
#include "motor-control.h"

// ============================================================================
// HOLONOMIC DRIVES (X-DRIVE, MECANUM) AND H-DRIVES
// Set one up with useHolonomicDrive() or useHDrive() in runPreAutonomous(). Every other RW motion
// works on them too, driving like a tank drive; the functions here can also strafe.
// ============================================================================

// How far the robot moves sideways per revolution of a strafing motor (inches, 0 = the same as
// wheel_distance_in, which is right for an X-drive). Mecanum wheels slip when they strafe, so they move
// a bit less. For an H-drive, use the strafe wheel's gear ratio * diameter * pi
RW_DEFAULT double strafe_distance_in = 0;

// True for holonomic drives and H-drives
bool driveCanStrafe() {
  return hw::holonomicDrive() || hw::hDrive();
}

/*
 * strafeEncoderDegrees
 * How far the drive has strafed, in motor degrees (odometry turns it into inches with strafe_distance_in).
 * Holonomic drives: the corner wheels' sideways mix. H-drives: the strafe wheel. Tank drives: 0.
 */
double strafeEncoderDegrees() {
  if (hw::holonomicDrive()) {
    double left_front = 0, right_front = 0, left_back = 0, right_back = 0;
    hw::cornerDegrees(left_front, right_front, left_back, right_back);
    return (left_front - right_front - left_back + right_back) / 4.0;
  }
  return hw::hDrive() ? hw::strafeDegrees() : 0;
}

/*
 * driveHolonomic
 * Drives forward, sideways and turning at the same time.
 * - forward_volts, strafe_volts, turn_volts: + = forward, right, and clockwise.
 * If a wheel would need more than 12 V, all of them scale down together so the robot keeps its direction.
 * Tank drives ignore strafe_volts; H-drives send it to the strafe wheel.
 */
void driveHolonomic(double forward_volts, double strafe_volts, double turn_volts) {
  if (hw::holonomicDrive()) {
    double left_front = forward_volts + strafe_volts + turn_volts;
    double right_front = forward_volts - strafe_volts - turn_volts;
    double left_back = forward_volts - strafe_volts + turn_volts;
    double right_back = forward_volts + strafe_volts - turn_volts;
    double biggest = fmax(fmax(fabs(left_front), fabs(right_front)), fmax(fabs(left_back), fabs(right_back)));
    if (biggest > 12) {
      double scale = 12 / biggest;
      left_front *= scale;
      right_front *= scale;
      left_back *= scale;
      right_back *= scale;
    }
    hw::cornerVolts(left_front, right_front, left_back, right_back);
    return;
  }
  double left = forward_volts + turn_volts, right = forward_volts - turn_volts;
  double biggest = fmax(fabs(left), fabs(right));
  if (biggest > 12) {
    left = left / biggest * 12;
    right = right / biggest * 12;
  }
  hw::driveVolts(left, right);
  if (hw::hDrive()) {
    hw::strafeVolts(fmax(-12, fmin(12, strafe_volts)));
  }
}

/*
 * strafeToPose
 * Drives straight to a point while turning to a heading, strafing as needed (holonomic and H-drives).
 * On a tank drive it drives to the point and then turns instead.
 * - x, y: Field point (in inches).
 * - heading_deg: Heading to end at (in degrees).
 * - time_limit_msec: Maximum time allowed (in milliseconds).
 * - exit: If true, stops on target; if false, ends near the point still moving, to chain into the next motion.
 * - max_output: Maximum voltage output to motors.
 */
void strafeToPose(double x, double y, double heading_deg, double time_limit_msec, bool exit, double max_output) {
  if (!driveCanStrafe()) {
    static bool warned = false;
    if (!warned) {
      printf("strafeToPose: this drive can't strafe (see useHolonomicDrive() and useHDrive()), so it drives there and turns\n");
      warned = true;
    }
    moveToPoint(x, y, 1, time_limit_msec, true, max_output);
    turnToAngle(heading_deg, time_limit_msec, exit, max_output);
    return;
  }
  hw::stopDrive(hw::Stop::coast);
  beginMotion();
  double target_heading = normalizeTarget(heading_deg);

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  pid_distance.setIntegralMax(0);
  pid_distance.setIntegralRange(3);
  pid_distance.setSmallBigErrorTolerance(0.5, 1.5);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  PID pid_turn = PID(turn_kp, turn_ki, turn_kd);
  pid_turn.setTarget(target_heading);
  pid_turn.setIntegralMax(0);
  pid_turn.setIntegralRange(3);
  pid_turn.setSmallBigErrorTolerance(1, 3);
  pid_turn.setSmallBigErrorDuration(50, 250);
  pid_turn.setDerivativeTolerance(4.5);

  double start_time = hw::timeMsec();
  while (motionActive(start_time, time_limit_msec)) {
    double error_x = x - x_pos, error_y = y - y_pos;
    double distance = hypot(error_x, error_y);
    // Chaining: hand over to the next motion once close
    if (!exit && distance < 3) {
      break;
    }
    // The direction to the point, split into the robot's forward and sideways
    double heading = getInertialHeading();
    double heading_rad = degToRad(heading);
    double forward = error_x * sin(heading_rad) + error_y * cos(heading_rad);
    double sideways = error_x * cos(heading_rad) - error_y * sin(heading_rad);

    pid_distance.setTarget(distance);
    double speed = fmax(-max_output, fmin(max_output, pid_distance.update(0)));
    double turn = fmax(-max_output, fmin(max_output, pid_turn.update(heading)));
    if (exit && pid_distance.targetArrived() && pid_turn.targetArrived()) {
      break;
    }
    double forward_volts = distance > 0.01 ? speed * forward / distance : 0;
    double strafe_volts = distance > 0.01 ? speed * sideways / distance : 0;
    driveHolonomic(forward_volts, strafe_volts, turn);
    hw::sleepMsec(10);
  }
  if (exit) {
    hw::stopDrive(hw::Stop::hold);
  }
  correct_angle = target_heading;
  endMotion();
}

/*
 * strafe
 * Strafes sideways from where the robot is, holding its heading (holonomic and H-drives).
 * - distance_in: How far (in inches, + = right, - = left).
 * - time_limit_msec, exit, max_output: Like strafeToPose().
 */
void strafe(double distance_in, double time_limit_msec, bool exit, double max_output) {
  double heading_rad = degToRad(correct_angle);
  strafeToPose(x_pos + distance_in * cos(heading_rad), y_pos - distance_in * sin(heading_rad), correct_angle,
               time_limit_msec, exit, max_output);
}
