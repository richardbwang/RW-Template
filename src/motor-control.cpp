#include "hardware.h"
#include "utils.h"
#include "pid.h"
#include <cmath>
#include <stdint.h>
#include <stdio.h>
#include "motor-control.h"
#include "mcl.h"

// ============================================================================
// INTERNAL STATE (DO NOT CHANGE)
// ============================================================================
bool is_turning = false;
double prev_left_output = 0, prev_right_output = 0;
double x_pos = 0, y_pos = 0;
double correct_angle = 0;
// Field config for distance resets, DO NOT CHANGE
double field_half_size = 70.25;  // Half field size in inches
// Colors of the PID graph that turns draw on the Brain screen
const uint32_t graph_background = 0x000000, graph_target = 0x00FF00, graph_trace = 0xFF0000;

// Odometry extras: total distance driven (for waitUntilTraveled) and current speeds (for stall exits)
double odom_traveled_in = 0;
double odom_speed_ips = 0, odom_turn_rate_dps = 0;
bool odom_resync = false;
// Raw odometry movement, never changed by resets or MCL (MCL uses it to move its particles)
double odom_motion_x = 0, odom_motion_y = 0;

// MCL state (see runMcl)
bool mcl_running = false, mcl_recenter = false;
double mcl_x = 0, mcl_y = 0, mcl_spread = 0;
double mcl_cycle_msec = 0; // How long each MCL update takes on the Brain

// Motion state shared by the motion functions and the async/wait helpers
bool motion_running = false, async_running = false, motion_cancel_requested = false;
double motion_start_traveled_in = 0, motion_start_heading = 0;
double stall_start_time = -1;
intptr_t async_thread_id = -1;
void (*async_motion)() = nullptr;

// ============================================================================
// NEWER SETTINGS (DEFAULTS)
// These have defaults here so custom folders from older versions still build.
// To change one, define it again in custom/src/robot-config.cpp, e.g. "double imu_scale = 1.0035;"
// ============================================================================
RW_DEFAULT double imu_scale = 1.0;                 // Multiplies the IMU reading, see measureImuScale()
RW_DEFAULT double stall_exit_msec = 0;             // End a motion after the robot is stuck this long (0 = off)
RW_DEFAULT double distance_reset_max_angle = 30;   // Skip distance resets when the beam hits the wall at a steeper angle (degrees)
RW_DEFAULT double distance_reset_max_jump = 0;     // Skip distance resets that move the position more than this (inches, 0 = off)
RW_DEFAULT double distance_reset_corner_margin = 0; // Skip distance resets that hit the wall this close to a corner (inches, 0 = off)
RW_DEFAULT int mcl_particles = 200;                // MCL particle count (more = smoother but slower, up to 1000)
RW_DEFAULT double mcl_max_range = 60;              // MCL ignores distance readings longer than this (inches)
RW_DEFAULT double distance_sensor_delay_msec = 40; // How far behind the distance sensor readings are (msec)
RW_DEFAULT double distance_sensor_scale = 1.0;     // Multiplies distance readings (V5 sensors often read 1-3% long)

// ============================================================================
// CHASSIS CONTROL FUNCTIONS
// ============================================================================

/*
 * Sets the voltage output for the left and right chassis motors.
 * - left_power: Voltage for the left side (in volts).
 * - right_power: Voltage for the right side (in volts).
 */
void driveChassis(double left_power, double right_power) {
  // Spin left and right chassis motors with specified voltages
  hw::driveVolts(left_power, right_power);
}

// stopChassis() is in src/vexcode/hardware.cpp (pros/rw/hardware.cpp on PROS), since it takes the platform's brake type

/*
 * Resets the rotation position of both chassis motors to zero.
 */
void resetChassis() {
  // Set both chassis motor encoders to zero
  hw::resetDriveDegrees();
}

/*
 * Returns the current rotation of the left chassis motor in degrees.
 */
double getLeftRotationDegree() {
  // Get left chassis motor position in degrees
  return hw::leftDegrees();
}

/*
 * Returns the current rotation of the right chassis motor in degrees.
 */
double getRightRotationDegree() {
  // Get right chassis motor position in degrees
  return hw::rightDegrees();
}

/*
 * Normalizes an angle to be within +/-180 degrees of the current heading.
 * - angle: The target angle to normalize.
 */
double normalizeTarget(double angle) {
  // Adjust angle to be within +/-180 degrees of the inertial sensor's rotation
  return turnTarget(angle, getInertialHeading(), 0);
}

/*
 * Returns the current inertial sensor heading in degrees, corrected by imu_scale.
 */
double getInertialHeading() {
  // Get inertial sensor rotation in degrees
  return hw::imuDegrees() * imu_scale;
}

// ============================================================================
// OUTPUT SCALING HELPER FUNCTIONS
// ============================================================================

/*
 * Ensures output values are at least the specified minimum for both sides.
 * - left_output: Reference to left output voltage.
 * - right_output: Reference to right output voltage.
 * - min_output: Minimum allowed output voltage.
 */
void scaleToMin(double& left_output, double& right_output, double min_output) {
  // Scale outputs to ensure minimum voltage is met for both sides
  if (fabs(left_output) <= fabs(right_output) && left_output < min_output && left_output > 0) {
    right_output = right_output / left_output * min_output;
    left_output = min_output;
  } else if (fabs(right_output) < fabs(left_output) && right_output < min_output && right_output > 0) {
    left_output = left_output / right_output * min_output;
    right_output = min_output;
  } else if (fabs(left_output) <= fabs(right_output) && left_output > -min_output && left_output < 0) {
    right_output = right_output / left_output * -min_output;
    left_output = -min_output;
  } else if (fabs(right_output) < fabs(left_output) && right_output > -min_output && right_output < 0) {
    left_output = left_output / right_output * -min_output;
    right_output = -min_output;
  }
}

/*
 * Ensures output values do not exceed the specified maximum for both sides.
 * - left_output: Reference to left output voltage.
 * - right_output: Reference to right output voltage.
 * - max_output: Maximum allowed output voltage.
 */
void scaleToMax(double& left_output, double& right_output, double max_output) {
  // Scale outputs to ensure maximum voltage is not exceeded for both sides
  if (fabs(left_output) >= fabs(right_output) && left_output > max_output) {
    right_output = right_output / left_output * max_output;
    left_output = max_output;
  } else if (fabs(right_output) > fabs(left_output) && right_output > max_output) {
    left_output = left_output / right_output * max_output;
    right_output = max_output;
  } else if (fabs(left_output) > fabs(right_output) && left_output < -max_output) {
    right_output = right_output / left_output * -max_output;
    left_output = -max_output;
  } else if (fabs(right_output) > fabs(left_output) && right_output < -max_output) {
    left_output = left_output / right_output * -max_output;
    right_output = -max_output;
  }
}

// ============================================================================
// MOTION LIFECYCLE, ASYNC MOTIONS, AND WAIT HELPERS
// ============================================================================

/*
 * beginMotion / endMotion
 * Called at the start and end of every motion function. They track what is running
 * for the wait helpers, cancelMotion(), stall exits, and correctHeading().
 */
void beginMotion() {
  // A regular motion started while an async motion is still running waits for it,
  // so two motions never fight over the drivetrain
  if (async_running && hw::threadId() != async_thread_id) {
    waitForMotion();
  }
  motion_running = true;
  is_turning = true;
  motion_cancel_requested = false;
  motion_start_traveled_in = odom_traveled_in;
  motion_start_heading = getInertialHeading();
  stall_start_time = -1;
}

void endMotion() {
  motion_running = false;
  is_turning = false;
}

/*
 * motionActive
 * Loop condition shared by every motion. Returns false once the motion times out,
 * is cancelled with cancelMotion(), or (when stall_exit_msec > 0) has been stuck that long.
 * - start_time: hw::timeMsec() when the motion started.
 * - time_limit_msec: Maximum time allowed for the motion (in milliseconds).
 */
bool motionActive(double start_time, double time_limit_msec) {
  double now = hw::timeMsec();
  if (now - start_time > time_limit_msec || motion_cancel_requested) {
    return false;
  }
  // Stall exit: ignore the first 300 msec while the robot gets moving
  if (stall_exit_msec > 0 && now - start_time > 300) {
    bool stopped = odom_speed_ips < 1.5 && odom_turn_rate_dps < 10;
    if (!stopped) {
      stall_start_time = -1;
    } else if (stall_start_time < 0) {
      stall_start_time = now;
    } else if (now - stall_start_time >= stall_exit_msec) {
      return false;
    }
  }
  return true;
}

/*
 * asyncMotionThread
 * Thread body for runAsync(). Runs the queued motion, then marks the async slot free.
 */
void asyncMotionThread() {
  async_thread_id = hw::threadId();
  async_motion();
  async_running = false;
}

/*
 * runAsync
 * Runs a motion (or a few chained motions) in the background so your autonomous
 * code keeps going. Only one async motion runs at a time: if one is still running,
 * this waits for it to finish first. A regular motion called while an async one
 * is running also waits for it.
 * - motion: Function to run, usually a lambda such as []{ moveToPoint(24, 48, 1, 2000); }
 *
 * Example:
 *   runAsync([]{ moveToPoint(24, 48, 1, 2000); });
 *   waitUntilTraveled(20);             // 20 inches into the move...
 *   intake_motor.spin(fwd, 12, volt);  // ...start the intake
 *   waitForMotion();
 */
void runAsync(void (*motion)()) {
  // Called from inside an async motion: just run it here instead of waiting on ourselves
  if (async_running && hw::threadId() == async_thread_id) {
    motion();
    return;
  }
  waitForMotion();
  async_motion = motion;
  async_running = true;
  // Start the progress counters now so wait helpers called right after this measure from here
  motion_start_traveled_in = odom_traveled_in;
  motion_start_heading = getInertialHeading();
  hw::startThread(asyncMotionThread);
}

/*
 * waitForMotion
 * Blocks until no motion is running (async or regular).
 */
void waitForMotion() {
  // Inside an async motion, its own motions run one after another, so there's nothing to wait for
  if (async_running && hw::threadId() == async_thread_id) {
    return;
  }
  while (async_running || motion_running) {
    hw::sleepMsec(10);
  }
}

/*
 * waitUntilTraveled
 * Blocks until the running motion has driven this far (path length, inches), or it ends.
 * Distance is measured from the start of the motion that is currently running.
 * - distance_in: Distance to wait for (in inches).
 */
void waitUntilTraveled(double distance_in) {
  while ((async_running || motion_running) && odom_traveled_in - motion_start_traveled_in < distance_in) {
    hw::sleepMsec(10);
  }
}

/*
 * waitUntilTurned
 * Blocks until the running motion has turned this many degrees (either way), or it ends.
 * - angle_deg: Amount of turning to wait for (in degrees).
 */
void waitUntilTurned(double angle_deg) {
  while ((async_running || motion_running) && fabs(getInertialHeading() - motion_start_heading) < angle_deg) {
    hw::sleepMsec(10);
  }
}

/*
 * waitUntilNear
 * Blocks until the robot is within radius_in of (x, y), or the running motion ends.
 * - x, y: Field point (in inches).
 * - radius_in: How close counts as "near" (in inches).
 */
void waitUntilNear(double x, double y, double radius_in) {
  while ((async_running || motion_running) && hypot(x - x_pos, y - y_pos) > radius_in) {
    hw::sleepMsec(10);
  }
}

/*
 * cancelMotion
 * Ends the motion that is currently running, as if it had timed out (it still brakes if exit is true).
 * Call it from another thread or after a wait helper, e.g. to stop driving once a sensor sees an object.
 */
void cancelMotion() {
  motion_cancel_requested = true;
}

/*
 * isMotionRunning
 * Returns true while a motion (async or regular) is running.
 */
bool isMotionRunning() {
  return async_running || motion_running;
}

/*
 * setPose
 * Sets the robot's position and heading, e.g. at the start of autonomous.
 * Safe to call while odometry is running.
 * - x, y: Robot position (in inches).
 * - heading_deg: Robot heading (in degrees, 0 faces +Y, clockwise positive).
 */
void setPose(double x, double y, double heading_deg) {
  hw::setImuDegrees(heading_deg / imu_scale);
  x_pos = x;
  y_pos = y;
  correct_angle = heading_deg;
  // Tell the odometry thread not to count the heading jump as a turn, and restart MCL here
  odom_resync = true;
  mcl_recenter = true;
}

/*
 * measureImuScale
 * Helper for tuning imu_scale. Run it (e.g. from runDriver), line the robot up with a tile edge,
 * spin it by hand exactly `turns` full turns clockwise, and line it up again. The Brain screen
 * shows the scale to put in custom/src/robot-config.cpp. Never returns.
 * - turns: Number of full turns you will spin the robot (more turns = more accurate, 5-10 is good).
 */
void measureImuScale(int turns) {
  hw::setImuDegrees(0);
  hw::screenClear(0x000000);
  while (true) {
    double raw = hw::imuDegrees();
    hw::screenRow(1, "Spin the robot %d full turns clockwise", turns);
    hw::screenRow(2, "IMU reads: %.2f deg (should be %d)     ", raw, turns * 360);
    if (fabs(raw) > 90) {
      hw::screenRow(3, "imu_scale = %.5f     ", turns * 360.0 / raw);
    }
    hw::sleepMsec(50);
  }
}

// ============================================================================
// MAIN DRIVE AND TURN FUNCTIONS
// ============================================================================

/*
 * Turns the robot to a specified angle using PID control, the short way.
 * - turn_angle: Target angle to turn to (in degrees).
 * - time_limit_msec: Maximum time allowed for the turn (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void turnToAngle(double turn_angle, double time_limit_msec, bool exit, double max_output) {
  turnToAngle(turn_angle, TurnDirection::shortest, time_limit_msec, exit, max_output);
}

/*
 * Turns the robot to a specified angle using PID control, the way you pick.
 * - turn_angle: Target angle to turn to (in degrees).
 * - direction: clockwise (right) or counterclockwise (left). For turns of about 180 degrees, where
 *   the short way can go either way. Up to 5 degrees past the target, it still turns the short way.
 * - time_limit_msec: Maximum time allowed for the turn (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void turnToAngle(double turn_angle, TurnDirection direction, double time_limit_msec, bool exit, double max_output) {
  // Prepare for turn
  hw::stopDrive(hw::Stop::coast);
  beginMotion();
  double threshold = 1;
  PID pid = PID(turn_kp, turn_ki, turn_kd);

  // Pick the target angle for the direction and set the PID target
  turn_angle = turnTarget(turn_angle, getInertialHeading(), (int)direction);
  pid.setTarget(turn_angle);
  pid.setIntegralMax(0);  
  pid.setIntegralRange(3);
  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(50, 250);
  pid.setDerivativeTolerance(threshold * 4.5);

  // Draw baseline for visualization
  double draw_amplifier = 230 / fabs(turn_angle);
  hw::screenClear(graph_background);
  hw::screenLine(0, fabs(turn_angle) * draw_amplifier, 600, fabs(turn_angle) * draw_amplifier, graph_target);

  // PID loop for turning
  double start_time = hw::timeMsec();
  double output;
  double current_heading = getInertialHeading();
  double previous_heading = 0;
  int index = 1;
  if(exit == false && correct_angle < turn_angle) {
    // Turn right without stopping at end
    while (getInertialHeading() < turn_angle && motionActive(start_time, time_limit_msec)) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading); // PID update for heading
      // Draw heading trace
      hw::screenLine(index * 3, fabs(previous_heading) * draw_amplifier, (index + 1) * 3, fabs(current_heading * draw_amplifier), graph_trace);
      index++;
      previous_heading = current_heading;
      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(output, -output);
      hw::sleepMsec(10);
    }
  } else if(exit == false && correct_angle > turn_angle) {
    // Turn left without stopping at end
    while (getInertialHeading() > turn_angle && motionActive(start_time, time_limit_msec)) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      hw::screenLine(index * 3, fabs(previous_heading) * draw_amplifier, (index + 1) * 3, fabs(current_heading * draw_amplifier), graph_trace);
      index++;
      previous_heading = current_heading;
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(-output, output);
      hw::sleepMsec(10);
    }
  } else {
    // Standard PID turn
    while (!pid.targetArrived() && motionActive(start_time, time_limit_msec)) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      hw::screenLine(index * 3, fabs(previous_heading) * draw_amplifier, (index + 1) * 3, fabs(current_heading * draw_amplifier), graph_trace);
      index++;
      previous_heading = current_heading;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(output, -output);
      hw::sleepMsec(10);
    }
  }
  if(exit) {
    hw::stopDrive(hw::Stop::hold);
  }
  correct_angle = turn_angle;
  endMotion();
}

/*
 * Drives the robot a specified distance (in inches) using PID control.
 * - distance_in: Target distance to drive (positive or negative).
 * - time_limit_msec: Maximum time allowed for the move (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void driveTo(double distance_in, double time_limit_msec, bool exit, double max_output) {
  // Store initial encoder values
  double start_left = getLeftRotationDegree(), start_right = getRightRotationDegree();
  hw::stopDrive(hw::Stop::coast);
  beginMotion();
  double threshold = 0.5;
  int drive_direction = distance_in > 0 ? 1 : -1;
  double max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    // Adjust slew rates and min speed for chaining
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = drive_direction > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  distance_in = distance_in * drive_direction;
  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // Configure PID controllers
  pid_distance.setTarget(distance_in);
  pid_distance.setIntegralMax(3);  
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(correct_angle));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  double start_time = hw::timeMsec();
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_distance = 0, current_angle = 0;

  // Main PID loop for driving straight
  while (((!pid_distance.targetArrived()) && motionActive(start_time, time_limit_msec) && exit) || (exit == false && current_distance < distance_in && motionActive(start_time, time_limit_msec))) {
    // Calculate current distance and heading
    current_distance = (fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in) + fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in)) / 2;
    current_angle = getInertialHeading();
    left_output = pid_distance.update(current_distance) * drive_direction;
    right_output = left_output;
    correction_output = pid_heading.update(current_angle);

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }
    if(!exit) {
      left_output = 24 * drive_direction;
      right_output = 24 * drive_direction;
    }

    left_output += correction_output;
    right_output -= correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output);
    hw::sleepMsec(10);
  }
  if(exit) {
    prev_left_output = 0;
    prev_right_output = 0;
    hw::stopDrive(hw::Stop::hold);
  }
  endMotion();
}

/*
 * CurveCircle
 * Drives the robot in a circular arc with a specified radius and angle.
 * - result_angle_deg: Target ending angle (in degrees) for the arc.
 * - center_radius: Radius of the circle's center (positive for curve to the right, negative for curve to the left).
 * - time_limit_msec: Maximum time allowed for the curve (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit, double max_output) {
  // Store initial encoder values for both sides
  double start_right = getRightRotationDegree(), start_left = getLeftRotationDegree();
  double in_arc, out_arc;
  double real_angle = 0, current_angle = 0;
  double ratio, result_angle;

  // Normalize the target angle to be within +/-180 degrees of the current heading
  result_angle_deg = normalizeTarget(result_angle_deg);
  result_angle = (result_angle_deg - correct_angle) * 3.14159265359 / 180;

  // Calculate arc lengths for inner and outer wheels
  in_arc = fabs((fabs(center_radius) - (distance_between_wheels / 2)) * result_angle);
  out_arc = fabs((fabs(center_radius) + (distance_between_wheels / 2)) * result_angle);
  ratio = in_arc / out_arc;

  hw::stopDrive(hw::Stop::coast);
  beginMotion();
  double threshold = 0.5;

  // Determine curve and drive direction
  int curve_direction = center_radius > 0 ? 1 : -1;
  int drive_direction = 0;
  if ((curve_direction == 1 && (result_angle_deg - correct_angle) > 0) || (curve_direction == -1 && (result_angle_deg - correct_angle) < 0)) {
    drive_direction = 1;
  } else {
    drive_direction = -1;
  }

  // Slew rate and minimum speed logic for chaining
  double max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = drive_direction > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  // Initialize PID controllers for arc distance and heading correction
  PID pid_out = PID(distance_kp, distance_ki, distance_kd);
  PID pid_turn = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid_out.setTarget(out_arc);
  pid_out.setIntegralMax(0);  
  pid_out.setIntegralRange(5);
  pid_out.setSmallBigErrorTolerance(0.3, 0.9);
  pid_out.setSmallBigErrorDuration(50, 250);
  pid_out.setDerivativeTolerance(threshold * 4.5);

  pid_turn.setTarget(0);
  pid_turn.setIntegralMax(0);  
  pid_turn.setIntegralRange(1);
  pid_turn.setSmallBigErrorTolerance(0, 0);
  pid_turn.setSmallBigErrorDuration(0, 0);
  pid_turn.setDerivativeTolerance(0);
  pid_turn.setArrive(false);

  double start_time = hw::timeMsec();
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_right = 0, current_left = 0;

  // Main control loop for each curve/exit configuration
  if (curve_direction == -1 && exit == true) {
    // Left curve, stop at end
    while (!pid_out.targetArrived() && motionActive(start_time, time_limit_msec)) {
      current_angle = getInertialHeading();
      current_right = fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in);
      // Calculate the real angle along the arc
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      right_output = pid_out.update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pid_turn.update(current_angle);

      // Enforce minimum output if chaining
      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      // Apply heading correction
      left_output += correction_output;
      right_output -= correction_output;

      // Enforce maximum output
      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      hw::sleepMsec(10);
    }
  } else if (curve_direction == 1 && exit == true) {
    // Right curve, stop at end
    while (!pid_out.targetArrived() && motionActive(start_time, time_limit_msec)) {
      current_angle = getInertialHeading();
      current_left = fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      left_output = pid_out.update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      hw::sleepMsec(10);
    }
  } else if (curve_direction == -1 && exit == false) {
    // Left curve, chaining (do not stop at end)
    while (current_right < out_arc && motionActive(start_time, time_limit_msec)) {
      current_angle = getInertialHeading();
      current_right = fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in);
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      right_output = pid_out.update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }
      
      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      hw::sleepMsec(10);
    }
  } else {
    // Right curve, chaining (do not stop at end)
    while (current_left < out_arc && motionActive(start_time, time_limit_msec)) {
      current_angle = getInertialHeading();
      current_left = fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      left_output = pid_out.update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      hw::sleepMsec(10);
    }
  }
  // Stop the chassis if required
  if(exit == true) {
    hw::stopDrive(hw::Stop::hold);
  }
  // Update the global heading
  correct_angle = result_angle_deg;
  endMotion();
}

/*
 * Swing
 * Performs a swing turn, rotating the robot around a point while driving forward or backward.
 * - swing_angle: Target angle to swing to (in degrees).
 * - drive_direction: Direction to drive (1 for forward, -1 for backward).
 * - time_limit_msec: Maximum time allowed for the swing (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit, double max_output) {
  hw::stopDrive(hw::Stop::coast); // Stop chassis before starting swing
  beginMotion();
  double threshold = 1;
  PID pid = PID(turn_kp, turn_ki, turn_kd); // Initialize PID for turning

  swing_angle = normalizeTarget(swing_angle); // Normalize target angle
  pid.setTarget(swing_angle);                 // Set PID target
  pid.setIntegralMax(0);  
  pid.setIntegralRange(5);

  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(50, 250);
  pid.setDerivativeTolerance(threshold * 4.5);

  // Draw the baseline for visualization
  double draw_amplifier = 230 / fabs(swing_angle);
  hw::screenClear(graph_background);
  hw::screenLine(0, fabs(swing_angle) * draw_amplifier, 600, fabs(swing_angle) * draw_amplifier, graph_target);

  // Start the PID loop
  double start_time = hw::timeMsec();
  double output;
  double current_heading = correct_angle;
  double previous_heading = 0;
  int index = 1;
  int choice = 1;

  // Determine which side to swing and direction
  if(swing_angle - correct_angle < 0 && drive_direction == 1) {
    choice = 1;
  } else if(swing_angle - correct_angle > 0 && drive_direction == 1) {
    choice = 2;
  } else if(swing_angle - correct_angle < 0 && drive_direction == -1) {
    choice = 3;
  } else {
    choice = 4;
  }

  // Swing logic for each case, chaining (exit == false)
  if(choice == 1 && exit == false) {
    // Swing left, forward
    while (current_heading > swing_angle && motionActive(start_time, time_limit_msec)) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Draw heading trace
      hw::screenLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier), graph_trace);
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      hw::stopLeft(hw::Stop::hold); // Hold left, swing right
      hw::rightVolts(output * drive_direction);
      hw::sleepMsec(10);
    }
  } else if(choice == 2 && exit == false) {
    // Swing right, forward
    while (current_heading < swing_angle && motionActive(start_time, time_limit_msec)) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Draw heading trace
      hw::screenLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier), graph_trace);
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      hw::leftVolts(output * drive_direction);
      hw::stopRight(hw::Stop::hold); // Hold right, swing left
      hw::sleepMsec(10);
    }
  } else if(choice == 3 && exit == false) {
    // Swing left, backward
    while (current_heading > swing_angle && motionActive(start_time, time_limit_msec)) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Draw heading trace
      hw::screenLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier), graph_trace);
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      hw::leftVolts(output * drive_direction);
      hw::stopRight(hw::Stop::hold);
      hw::sleepMsec(10);
    }
  } else {
    // Swing right, backward
    while (current_heading < swing_angle && motionActive(start_time, time_limit_msec) && exit == false) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Draw heading trace
      hw::screenLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier), graph_trace);
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      hw::stopLeft(hw::Stop::hold);
      hw::rightVolts(output * drive_direction);
      hw::sleepMsec(10);
    }
  }

  // PID loop for exit == true (stop at end)
  while (!pid.targetArrived() && motionActive(start_time, time_limit_msec) && exit == true) {
    current_heading = getInertialHeading();
    output = pid.update(current_heading);

    // Draw heading trace
    hw::screenLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier), graph_trace);
    index++;
    previous_heading = current_heading;

    // Clamp output
    if(output > max_output) output = max_output;
    else if(output < -max_output) output = -max_output;

    // Apply output to correct side based on swing direction
    switch(choice) {
    case 1:
      hw::stopLeft(hw::Stop::hold);
      hw::rightVolts(-output * drive_direction);
      break;
    case 2:
      hw::leftVolts(output * drive_direction);
      hw::stopRight(hw::Stop::hold);
      break;
    case 3:
      hw::leftVolts(-output * drive_direction);
      hw::stopRight(hw::Stop::hold);
      break;
    case 4:
      hw::stopLeft(hw::Stop::hold);
      hw::rightVolts(output * drive_direction);
      break;
    }
    hw::sleepMsec(10);
  }
  if(exit == true) {
    hw::stopDrive(hw::Stop::hold); // Stop chassis at end if required
  }
  correct_angle = swing_angle; // Update global heading
  endMotion();
}

/*
 * heading_correction
 * Continuously adjusts the robot's heading to maintain a straight course.
 * Uses a PID controller to minimize the error between the current heading and the target heading.
 */
void correctHeading() {
  double output = 0;
  PID pid = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid.setTarget(correct_angle); // Set PID target to current heading
  pid.setIntegralRange(fabs(correct_angle) / 2.5);

  pid.setSmallBigErrorTolerance(0, 0);
  pid.setSmallBigErrorDuration(0, 0);
  pid.setDerivativeTolerance(0);
  pid.setArrive(false);

  // Continuously correct heading while enabled
  while(heading_correction) {
    pid.setTarget(correct_angle);
    if(is_turning == false) {
      output = pid.update(getInertialHeading());
      driveChassis(output, -output); // Apply correction to chassis
    }
    hw::sleepMsec(10);
  }
}

/*
 * runOdometry
 * Tracks the robot's position (x_pos, y_pos) from the inertial heading plus tracking wheels
 * and/or drivetrain encoders. Runs forever, so start it in a thread (see startOdometry()).
 * - use_horizontal: true if a horizontal (sideways) tracking wheel is installed.
 * - use_vertical: true if a vertical (forward) tracking wheel is installed.
 *   Without one, the drivetrain encoders measure forward motion instead.
 * Holonomic drives and H-drives without a horizontal tracking wheel measure sideways motion with
 * their wheels instead (see strafeEncoderDegrees() in src/holonomic.cpp).
 */
void runOdometry(bool use_horizontal, bool use_vertical) {
  resetChassis();
  // Start from the current readings so a sensor that wasn't at zero doesn't look like movement
  double prev_heading_deg = getInertialHeading();
  double prev_left_deg = getLeftRotationDegree(), prev_right_deg = getRightRotationDegree();
  double prev_horizontal_deg = use_horizontal ? hw::horizontalTrackerDegrees() : 0;
  double prev_vertical_deg = use_vertical ? hw::verticalTrackerDegrees() : 0;
  bool wheels_strafe = false;
  double prev_strafe_deg = 0;
  double prev_time = hw::timeMsec();

  while (true) {
    double heading_deg = getInertialHeading();
    double left_deg = getLeftRotationDegree();
    double right_deg = getRightRotationDegree();
    double horizontal_deg = use_horizontal ? hw::horizontalTrackerDegrees() : 0;
    double vertical_deg = use_vertical ? hw::verticalTrackerDegrees() : 0;
    // The drive's own sideways measurement, when there's no horizontal tracking wheel to do it
    bool strafe_from_wheels = !use_horizontal && driveCanStrafe();
    double strafe_deg = strafe_from_wheels ? strafeEncoderDegrees() : 0;
    if (strafe_from_wheels != wheels_strafe) {
      // The drive just started (or stopped) strafing, e.g. useHolonomicDrive() after startOdometry()
      wheels_strafe = strafe_from_wheels;
      prev_strafe_deg = strafe_deg;
    }
    double now = hw::timeMsec();

    if (odom_resync) {
      // setPose() just moved the pose; skip this step so the jump isn't counted as motion
      odom_resync = false;
    } else {
      double delta_left_in = (left_deg - prev_left_deg) * wheel_distance_in / 360.0;
      double delta_right_in = (right_deg - prev_right_deg) * wheel_distance_in / 360.0;
      double delta_horizontal_in = (horizontal_deg - prev_horizontal_deg) * horizontal_tracker_diameter * M_PI / 360.0;
      double delta_vertical_in = (vertical_deg - prev_vertical_deg) * vertical_tracker_diameter * M_PI / 360.0;
      double horizontal_offset = horizontal_tracker_dist_from_center;
      if (wheels_strafe) {
        // Strafing measured at the robot's center, like a horizontal tracker there
        double strafe_in_per_rev = strafe_distance_in > 0 ? strafe_distance_in : wheel_distance_in;
        delta_horizontal_in = (strafe_deg - prev_strafe_deg) * strafe_in_per_rev / 360.0;
        horizontal_offset = 0;
      }
      bool sideways = use_horizontal || wheels_strafe;

      // A wheel can't really travel 3 inches in 10 msec. A jump like that is a glitch, such as a motor
      // briefly disconnecting and resetting its encoder, so skip this step instead of jumping
      double forward_delta_in = use_vertical ? fabs(delta_vertical_in) : fmax(fabs(delta_left_in), fabs(delta_right_in));
      bool glitch = forward_delta_in > 3 || (sideways && fabs(delta_horizontal_in) > 3);

      double delta_x_in = 0, delta_y_in = 0;
      if (!glitch) {
        odomStep(prev_heading_deg, heading_deg, delta_left_in, delta_right_in, delta_horizontal_in, delta_vertical_in,
                 sideways, use_vertical, distance_between_wheels, horizontal_offset,
                 vertical_tracker_dist_from_center, delta_x_in, delta_y_in);
      }
      x_pos += delta_x_in;
      y_pos += delta_y_in;
      odom_motion_x += delta_x_in;
      odom_motion_y += delta_y_in;

      // Distance and speeds for the wait helpers and stall exits (lightly filtered)
      double step_in = hypot(delta_x_in, delta_y_in);
      double dt_sec = (now - prev_time) / 1000.0;
      odom_traveled_in += step_in;
      if (dt_sec > 0) {
        odom_speed_ips = 0.7 * odom_speed_ips + 0.3 * (step_in / dt_sec);
        odom_turn_rate_dps = 0.7 * odom_turn_rate_dps + 0.3 * (fabs(heading_deg - prev_heading_deg) / dt_sec);
      }
    }

    prev_heading_deg = heading_deg;
    prev_left_deg = left_deg;
    prev_right_deg = right_deg;
    prev_horizontal_deg = horizontal_deg;
    prev_vertical_deg = vertical_deg;
    prev_strafe_deg = strafe_deg;
    prev_time = now;

    hw::sleepMsec(10);
  }
}

/*
 * trackNoOdomWheel / trackXYOdomWheel / trackXOdomWheel / trackYOdomWheel
 * Odometry thread functions for each tracking wheel setup (kept so older custom folders still work).
 * New code can call startOdometry() instead, which picks the right one.
 */
void trackNoOdomWheel() { runOdometry(false, false); }
void trackXYOdomWheel() { runOdometry(true, true); }
void trackXOdomWheel() { runOdometry(true, false); }
void trackYOdomWheel() { runOdometry(false, true); }

/*
 * startOdometry
 * Starts the odometry thread for the tracking wheels enabled in custom/src/robot-config.cpp.
 */
void startOdometry() {
  // Only once: a second odometry thread would count every movement twice
  static bool started = false;
  if (started) {
    printf("startOdometry: odometry is already running, so this call does nothing\n");
    return;
  }
  started = true;
  if (using_horizontal_tracker && using_vertical_tracker) {
    hw::startThread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    hw::startThread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    hw::startThread(trackYOdomWheel);
  } else {
    hw::startThread(trackNoOdomWheel);
  }
}

/*
 * turnToPoint
 * Turns the robot to face a specific point in the field.
 * - x, y: Coordinates of the target point.
 * - direction: Direction to face the point (1 for forward, -1 for backward).
 * - time_limit_msec: Maximum time allowed for the turn (in milliseconds).
 */
void turnToPoint(double x, double y, int direction, double time_limit_msec) {
  hw::stopDrive(hw::Stop::coast); // Stop chassis before turning
  beginMotion();
  double threshold = 1, add = 0;
  if(direction == -1) {
    add = 180; // Add 180 degrees if turning to face backward
  }
  // Calculate target angle using atan2 and normalize
  double turn_angle = normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add);
  PID pid = PID(turn_kp, turn_ki, turn_kd);

  pid.setTarget(turn_angle); // Set PID target
  pid.setIntegralMax(0);  
  pid.setIntegralRange(3);

  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(100, 500);
  pid.setDerivativeTolerance(threshold * 4.5);

  // Draw the baseline for visualization
  double draw_amplifier = 230 / fabs(turn_angle);
  hw::screenClear(graph_background);
  hw::screenLine(0, fabs(turn_angle) * draw_amplifier, 
                        600, fabs(turn_angle) * draw_amplifier, graph_target);

  // Start the PID loop
  double start_time = hw::timeMsec();
  double output;
  double current_heading;
  double previous_heading = 0;
  int index = 1;
  while (!pid.targetArrived() && motionActive(start_time, time_limit_msec)) {
    // Continuously update target as robot moves
    pid.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
    current_heading = getInertialHeading();
    output = pid.update(current_heading);

    // Draw heading trace
    hw::screenLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier), graph_trace);
    index++;
    previous_heading = current_heading;

    driveChassis(output, -output); // Apply output to chassis
    hw::sleepMsec(10);
  }  
  hw::stopDrive(hw::Stop::hold); // Stop at end
  correct_angle = getInertialHeading(); // Update global heading
  endMotion();
}

/*
 * moveToPoint
 * Moves the robot to a specific point in the field, adjusting heading as needed.
 * - x, y: Coordinates of the target point.
 * - dir: Direction to move in (1 for forward, -1 for backward).
 * - time_limit_msec: Maximum time allowed for the move (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 * - overturn: If true, allows overturning for sharp turns.
 */
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit, double max_output, bool overturn) {
  hw::stopDrive(hw::Stop::coast); // Stop chassis before moving
  beginMotion();
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    // Adjust slew rates and min speed for chaining
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = dir > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = dir > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = dir > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = dir > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // Set PID targets for distance and heading
  pid_distance.setTarget(hypot(x - x_pos, y - y_pos));
  pid_distance.setIntegralMax(0);  
  pid_distance.setIntegralRange(3);
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);
  
  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  // Reset the chassis
  double start_time = hw::timeMsec();
  double left_output = 0, right_output = 0, correction_output = 0, prev_left_output = 0, prev_right_output = 0;
  double exittolerance = 1;
  bool perpendicular_line = false, prev_perpendicular_line = true;

  double current_angle = 0, overturn_value = 0;
  bool ch = true;

  // Main PID loop for moving to point
  while (motionActive(start_time, time_limit_msec)) {
    // Continuously update targets as robot moves
    pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
    pid_distance.setTarget(hypot(x - x_pos, y - y_pos));
    current_angle = getInertialHeading();
    // Calculate drive output based on heading and distance
    left_output = pid_distance.update(0) * cos(degToRad(atan2(x - x_pos, y - y_pos) * 180 / M_PI + add - current_angle)) * dir;
    right_output = left_output;
    // Check if robot has crossed the perpendicular line to the target
    perpendicular_line = ((y_pos - y) * -cos(degToRad(normalizeTarget(current_angle + add))) <= (x_pos - x) * sin(degToRad(normalizeTarget(current_angle + add))) + exittolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Only apply heading correction if far from target
    if(hypot(x - x_pos, y - y_pos) > 8 && ch == true) {
      correction_output = pid_heading.update(current_angle);
    } else {
      correction_output = 0;
      ch = false;
    }

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }

    // Overturn logic for sharp turns
    overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
    if(overturn_value > 0 && overturn) {
      if(left_output > 0) {
        left_output -= overturn_value;
      }
      else {
        left_output += overturn_value;
      }
    }
    right_output = left_output;
    left_output = left_output + correction_output;
    right_output = right_output - correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output); // Apply output to chassis
    hw::sleepMsec(10);
  }
  if(exit == true) {
    prev_left_output = 0;
    prev_right_output = 0;
    hw::stopDrive(hw::Stop::hold); // Stop at end if required
  }
  correct_angle = getInertialHeading(); // Update global heading
  endMotion();
}

/*
 * boomerang
 * Drives the robot in a boomerang-shaped path to a target point.
 * - x, y: Coordinates of the target point.
 * - a: Final angle of the robot to target (in degrees).
 * - dlead: Distance to lead the target by (in inches, set higher for curvier path, don't set above 0.6).
 * - time_limit_msec: Maximum time allowed for the maneuver (in milliseconds).
 * - dir: Direction to move in (1 for forward, -1 for backward).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 * - overturn: If true, allows overturning for sharp turns.
 */
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit, double max_output, bool overturn) {
  hw::stopDrive(hw::Stop::coast); // Stop chassis before moving
  beginMotion();
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    // Adjust slew rates and min speed for chaining
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = dir > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = dir > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = dir > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = dir > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid_distance.setTarget(0); // Target is dynamically updated
  pid_distance.setIntegralMax(3);  
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos))));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  double start_time = hw::timeMsec();
  double left_output = 0, right_output = 0, correction_output = 0, slip_speed = 0, overturn_value = 0;
  double exit_tolerance = 3;
  bool perpendicular_line = false, prev_perpendicular_line = true;
  double current_angle = 0, hypotenuse = 0, carrot_x = 0, carrot_y = 0;

  // Main PID loop for boomerang path
  while ((!pid_distance.targetArrived()) && motionActive(start_time, time_limit_msec)) {
    hypotenuse = hypot(x_pos - x, y_pos - y); // Distance to target
    // Calculate carrot point for path leading
    carrot_x = x - hypotenuse * sin(degToRad(a + add)) * dlead;
    carrot_y = y - hypotenuse * cos(degToRad(a + add)) * dlead;
    pid_distance.setTarget(hypot(carrot_x - x_pos, carrot_y - y_pos) * dir);
    current_angle = getInertialHeading();
    // Calculate drive output based on carrot point
    left_output = pid_distance.update(0) * cos(degToRad(atan2(carrot_x - x_pos, carrot_y - y_pos) * 180 / M_PI + add - current_angle));
    right_output = left_output;
    // Check if robot has crossed the perpendicular line to the target
    perpendicular_line = ((y_pos - y) * -cos(degToRad(normalizeTarget(a))) <= (x_pos - x) * sin(degToRad(normalizeTarget(a))) + exit_tolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }

    // Heading correction logic based on distance to carrot/target
    if(hypot(carrot_x - x_pos, carrot_y - y_pos) > 8) {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(carrot_x - x_pos, carrot_y - y_pos)) + add));
      correction_output = pid_heading.update(current_angle);
    } else if(hypot(x - x_pos, y - y_pos) > 6) {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
      correction_output = pid_heading.update(current_angle);
    } else {
      pid_heading.setTarget(normalizeTarget(a));
      correction_output = pid_heading.update(current_angle);
      if(exit && hypot(x - x_pos, y - y_pos) < 5) {
        break;
      }
    }

    // Limit slip speed for smoother curves
    slip_speed = sqrt(chase_power * getRadius(x_pos, y_pos, carrot_x, carrot_y, current_angle) * 9.8);
    if(left_output > slip_speed) {
      left_output = slip_speed;
    } else if(left_output < -slip_speed) {
      left_output = -slip_speed;
    }

    // Overturn logic for sharp turns
    overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
    if(overturn_value > 0 && overturn) {
      if(left_output > 0) {
        left_output -= overturn_value;
      }
      else {
        left_output += overturn_value;
      }
    }
    right_output = left_output;
    left_output = left_output + correction_output;
    right_output = right_output - correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output); // Apply output to chassis
    hw::sleepMsec(10);
  }
  if(exit) {
    prev_left_output = 0;
    prev_right_output = 0;
    hw::stopDrive(hw::Stop::hold); // Stop at end if required
  }
  correct_angle = a;      // Update global heading
  endMotion();
}

// ============================================================================
// MONTE CARLO LOCALIZATION (MCL)
// ============================================================================

/*
 * runMcl
 * MCL thread: while the robot drives, uses the distance sensors to keep x_pos/y_pos
 * from drifting. The particle filter itself is in src/mcl.cpp. Start it with startMcl().
 * Uses the same sensors, offsets, and checks as the distance reset functions.
 */
void runMcl() {
  // Odometry movement and heading over the last 160 msec, for matching up delayed sensor readings
  const int history_size = 16;
  double history_x[history_size], history_y[history_size], history_heading[history_size];
  for (int i = 0; i < history_size; i++) {
    history_x[i] = odom_motion_x;
    history_y[i] = odom_motion_y;
    history_heading[i] = getInertialHeading();
  }
  double last_motion_x = odom_motion_x, last_motion_y = odom_motion_y, last_heading = getInertialHeading();
  double moved_since_correct = 0;

  const hw::DistanceSensor sensors[4] = {hw::front_distance, hw::right_distance, hw::back_distance, hw::left_distance};
  double sensor_x[4] = {front_sensor_offsetX, right_sensor_offsetX, back_sensor_offsetX, left_sensor_offsetX};
  double sensor_y[4] = {front_sensor_offsetY, right_sensor_offsetY, back_sensor_offsetY, left_sensor_offsetY};
  double sensor_angle[4] = {0, 90, 180, 270};
  // Distance sensors only measure about every 33 msec, so remember each one's last reading to skip repeats
  uint32_t last_stamp[4] = {0, 0, 0, 0};
  double last_reading[4] = {-1, -1, -1, -1};

  mclInit(mcl_particles, x_pos, y_pos, 1.0);
  mcl_running = true;
  int tick = 0;

  while (true) {
    double heading = getInertialHeading();
    history_x[tick % history_size] = odom_motion_x;
    history_y[tick % history_size] = odom_motion_y;
    history_heading[tick % history_size] = heading;

    if (mcl_recenter) {
      // setPose() or a distance reset just set the position; start the particles there
      mcl_recenter = false;
      mclInit(mcl_particles, x_pos, y_pos, 1.0);
    }

    // Run the filter every 30 msec (about how often the distance sensors update)
    if (tick % 3 == 0) {
      uint64_t cycle_start = hw::timeMicros();
      double delta_x = odom_motion_x - last_motion_x, delta_y = odom_motion_y - last_motion_y;
      double delta_heading = heading - last_heading;
      last_motion_x = odom_motion_x;
      last_motion_y = odom_motion_y;
      last_heading = heading;
      mclPredict(delta_x, delta_y, delta_heading);
      moved_since_correct += hypot(delta_x, delta_y) + fabs(delta_heading) * 0.05;

      // Readings lag behind, so compare them with where odometry says the robot was back then
      int delay_ticks = (int)fmin(fmax(round(distance_sensor_delay_msec / 10.0), 0), history_size - 1);
      int then = ((tick - delay_ticks) % history_size + history_size) % history_size;
      double shift_x = odom_motion_x - history_x[then], shift_y = odom_motion_y - history_y[then];
      double heading_then = history_heading[then];

      // Keep readings that hit a wall cleanly (same checks as the distance resets)
      MclReading readings[4];
      int count = 0;
      for (int i = 0; i < 4; i++) {
        if (!hw::distanceInstalled(sensors[i]) || !hw::distanceDetected(sensors[i])) {
          continue;
        }
        double reading_in = hw::distanceInches(sensors[i]) * distance_sensor_scale;
        uint32_t stamp = hw::distanceTimestamp(sensors[i]);
        bool repeat = stamp == last_stamp[i] || reading_in == last_reading[i];
        last_stamp[i] = stamp;
        last_reading[i] = reading_in;
        if (repeat || reading_in < 1.5 || reading_in > mcl_max_range) {
          continue;
        }
        double unused = 0;
        if (distanceResetPosition(x_pos - shift_x, y_pos - shift_y, heading_then, sensor_x[i], sensor_y[i], sensor_angle[i],
                                  reading_in, field_half_size, distance_reset_max_angle, distance_reset_corner_margin, unused) == 0) {
          continue;
        }
        readings[count].s_x = sensor_x[i];
        readings[count].s_y = sensor_y[i];
        readings[count].angle_deg = sensor_angle[i];
        readings[count].reading_in = reading_in;
        count++;
      }

      // Only weigh after moving a little, so a parked robot doesn't get overconfident from repeated
      // readings, and not while spinning fast, when delayed readings are least trustworthy
      if (count > 0 && moved_since_correct > 0.25 && odom_turn_rate_dps < 200) {
        mclCorrect(heading_then, readings, count, field_half_size, odom_speed_ips * 0.01, shift_x, shift_y);
        moved_since_correct = 0;
      }
      double spread_x = 0, spread_y = 0;
      mclEstimate(mcl_x, mcl_y, spread_x, spread_y);
      mcl_spread = fmax(spread_x, spread_y);
      mclNudge(x_pos, y_pos, mcl_x, mcl_y, spread_x, spread_y);
      mcl_cycle_msec = 0.8 * mcl_cycle_msec + 0.2 * (hw::timeMicros() - cycle_start) / 1000.0;
    }

    tick++;
    hw::sleepMsec(10);
  }
}

/*
 * startMcl
 * Starts MCL in the background. Call it after startOdometry() and setPose(), and use
 * field-centered coordinates (origin at the field center, walls at +/-70.25 in).
 */
void startMcl() {
  hw::startThread(runMcl);
}

// ============================================================================
// DISTANCE SENSOR POSITION RESET FUNCTIONS
// ============================================================================

/*
 * resetPositionWithReading
 * Resets X or Y using one distance sensor reading and the field wall its beam hits.
 * The current position estimate decides which wall that is, so angled readings
 * and readings near corners work. Needs field-centered coordinates (see setPose()).
 * Returns true if the position was reset, false if the reading was skipped.
 * resetPositionWithSensor() (in the platform's hardware.cpp) reads a sensor and calls this.
 *
 * - raw_reading_in: What the sensor measured (in inches, before distance_sensor_scale)
 * - s_x, s_y: Sensor position relative to the robot's center (in inches, +x = right, +y = forward)
 * - sensor_angle_offset: Direction the sensor faces on the robot (0° = front, 90° = right, 180° = back, 270° = left)
 * - field_half_size: Distance from the field center to each wall (in inches)

 ** - IMPORTANT NOTE FOR THE USER - **
 * - DO NOT CALL THIS FUNCTION DIRECTLY, use the specific direction functions below instead.
 */
bool resetPositionWithReading(double raw_reading_in, double s_x, double s_y, double sensor_angle_offset, double field_half_size) {
  // The V5 distance sensor reads up to about 2000 mm (78 in); anything else means no wall was seen
  double reading_in = raw_reading_in * distance_sensor_scale;
  if (reading_in <= 0 || reading_in > 78) {
    return false;
  }

  double new_pos = 0;
  int axis = distanceResetPosition(x_pos, y_pos, getInertialHeading(), s_x, s_y, sensor_angle_offset, reading_in,
                                   field_half_size, distance_reset_max_angle, distance_reset_corner_margin, new_pos);
  if (axis == 0) {
    return false;
  }

  // Optionally skip resets that disagree a lot with odometry (probably seeing a robot or game element)
  double old_pos = axis == 1 ? x_pos : y_pos;
  if (distance_reset_max_jump > 0 && fabs(new_pos - old_pos) > distance_reset_max_jump) {
    return false;
  }

  if (axis == 1) x_pos = new_pos; else y_pos = new_pos;
  mcl_recenter = true;
  return true;
}

// Resets the position with one of the four reset sensors, if it sees something
bool resetPositionFrom(hw::DistanceSensor sensor, double s_x, double s_y, double sensor_angle_offset) {
  if (!hw::distanceDetected(sensor)) {
    return false;
  }
  return resetPositionWithReading(hw::distanceInches(sensor), s_x, s_y, sensor_angle_offset, field_half_size);
}

/*
 * resetPositionFront / resetPositionBack / resetPositionLeft / resetPositionRight
 * Reset position using the distance sensor on that side of the robot.
 * Each returns true if the position was reset.
 */
bool resetPositionFront() {
  return resetPositionFrom(hw::front_distance, front_sensor_offsetX, front_sensor_offsetY, 0.0);
}

bool resetPositionBack() {
  return resetPositionFrom(hw::back_distance, back_sensor_offsetX, back_sensor_offsetY, 180.0);
}

bool resetPositionLeft() {
  return resetPositionFrom(hw::left_distance, left_sensor_offsetX, left_sensor_offsetY, 270.0);
}

bool resetPositionRight() {
  return resetPositionFrom(hw::right_distance, right_sensor_offsetX, right_sensor_offsetY, 90.0);
}

// ============================================================================
// TEMPLATE NOTE
// ============================================================================
// This file is intended as a template for VEX/V5 robotics teams.
// All functions and variables use clear, consistent naming conventions.
// Comments are concise and explain the intent of each section.
// Teams can adapt PID values, drive base geometry, and logic as needed for their robot.
