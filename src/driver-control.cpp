#include <cmath>
#include "motor-control.h"
#include "driver-control.h"

// ============================================================================
// DRIVER CONTROL SETTINGS (DEFAULTS)
// To change one, define it again in custom/src/robot-config.cpp, e.g. "double driver_curve = 4;"
// ============================================================================
RW_DEFAULT double driver_curve = 0;     // 0 = linear, higher = finer control at low stick values (2-6 is typical)
RW_DEFAULT double driver_deadband = 3;  // Stick values below this (percent) count as zero

/*
 * driverCurve
 * Applies the deadband and an exponential curve to a joystick value.
 * Full stick still gives full power; the curve only softens the low end.
 * - stick: Joystick value (-100 to 100).
 * Returns the curved value (-100 to 100).
 */
double driverCurve(double stick) {
  if (fabs(stick) < driver_deadband) {
    return 0;
  }
  if (driver_curve <= 0) {
    return stick;
  }
  double low_end_gain = exp(-driver_curve / 10.0);
  return (low_end_gain + exp((fabs(stick) - 100) / 10.0) * (1 - low_end_gain)) * stick;
}

/*
 * driverTank
 * Tank drive: one stick drives the left side, the other drives the right side.
 * - left_stick, right_stick: Joystick values (-100 to 100), e.g. controller_1.Axis3.position().
 */
void driverTank(double left_stick, double right_stick) {
  driveChassis(driverCurve(left_stick) * 0.12, driverCurve(right_stick) * 0.12);
}

/*
 * driverArcade
 * Arcade drive: one stick axis drives forward/backward, another turns.
 * If forward + turn adds up to more than full power, both sides scale down
 * together so the robot still turns at full speed.
 * - throttle_stick: Forward/backward joystick value (-100 to 100).
 * - turn_stick: Turning joystick value (-100 to 100), positive turns right.
 */
void driverArcade(double throttle_stick, double turn_stick) {
  double throttle = driverCurve(throttle_stick);
  double turn = driverCurve(turn_stick);
  double left = throttle + turn;
  double right = throttle - turn;
  double biggest = fmax(fabs(left), fabs(right));
  if (biggest > 100) {
    left = left / biggest * 100;
    right = right / biggest * 100;
  }
  driveChassis(left * 0.12, right * 0.12);
}

// The heading that's "forward" for driverFieldCentric() (degrees). 0 = the way the robot faced when the
// program started (or at setPose(..., 0)). Set it to getInertialHeading() to make forward the way the robot faces right now
RW_DEFAULT double driver_forward_heading = 0;

/*
 * driverHolonomic
 * Holonomic drive (X-drive, mecanum, H-drive): forward/backward, strafing and turning on three stick axes.
 * Forward and sideways are the robot's own (robot-centric). Tank drives ignore the strafe stick.
 * - forward_stick, strafe_stick, turn_stick: Joystick values (-100 to 100), + = forward, right, and clockwise.
 */
void driverHolonomic(double forward_stick, double strafe_stick, double turn_stick) {
  driveHolonomic(driverCurve(forward_stick) * 0.12, driverCurve(strafe_stick) * 0.12, driverCurve(turn_stick) * 0.12);
}

/*
 * driverFieldCentric
 * Like driverHolonomic(), but pushing the stick forward always drives the same way on the field
 * (driver_forward_heading), whichever way the robot is facing. Uses the IMU heading.
 * - forward_stick, strafe_stick, turn_stick: Joystick values (-100 to 100).
 */
void driverFieldCentric(double forward_stick, double strafe_stick, double turn_stick) {
  double forward = driverCurve(forward_stick), sideways = driverCurve(strafe_stick);
  double heading_rad = (getInertialHeading() - driver_forward_heading) * M_PI / 180.0;
  // The stick direction on the field, turned into the robot's forward and sideways
  double robot_forward = forward * cos(heading_rad) + sideways * sin(heading_rad);
  double robot_sideways = sideways * cos(heading_rad) - forward * sin(heading_rad);
  driveHolonomic(robot_forward * 0.12, robot_sideways * 0.12, driverCurve(turn_stick) * 0.12);
}
