#include "vex.h"
#include <cmath>
#include <vector>
#include "utils.h"
#include "motor-control.h"
#include "path-following.h"

// ============================================================================
// SETTINGS (DEFAULTS)
// To change one, define it again in custom/src/robot-config.cpp, e.g. "double drive_kV = 0.17;"
// ============================================================================
RW_DEFAULT double drive_kS = 0.5;               // Volts to get the drive moving
RW_DEFAULT double drive_kV = 0;                 // Volts per in/s (0 = estimate from drive_motor_rpm and wheel_distance_in)
RW_DEFAULT double drive_kA = 0;                 // Volts per in/s^2 (0 = estimate)
RW_DEFAULT double drive_motor_rpm = 600;        // Drive motor cartridge speed (600 blue, 200 green), used for the estimates
RW_DEFAULT double drive_velocity_kP = 0.03;     // Extra volts per in/s a side is off from its target speed
RW_DEFAULT double profile_max_speed = 0;        // Top speed for profiled moves and paths (in/s, 0 = 80% of the drive's top speed)
RW_DEFAULT double profile_max_accel = 100;      // How fast profiled moves and paths speed up and slow down (in/s^2)
RW_DEFAULT double profile_turn_speed = 360;     // Top turning speed for turnProfiled (degrees/s)
RW_DEFAULT double profile_turn_accel = 1200;    // Turning acceleration for turnProfiled (degrees/s^2)
RW_DEFAULT double path_lookahead = 12;          // Pure pursuit lookahead (inches): bigger = smoother, smaller = tighter
RW_DEFAULT double path_max_lateral_accel = 120; // Slows the robot in curves so it doesn't slide (in/s^2)

// ============================================================================
// INTERNAL STATE (DO NOT CHANGE)
// ============================================================================
double left_wheel_speed = 0, right_wheel_speed = 0;  // Measured wheel speeds (in/s)
double last_left_deg = 0, last_right_deg = 0, last_speed_time = -1;

// ============================================================================
// FEEDFORWARD AND WHEEL SPEED CONTROL
// ============================================================================

// Drive top speed with no load (in/s), from the motor cartridge and wheel_distance_in
double driveFreeSpeed() { return drive_motor_rpm / 60.0 * wheel_distance_in; }
double driveKV() { return drive_kV > 0 ? drive_kV : 12.0 / (driveFreeSpeed() * 0.92); }
double driveKA() { return drive_kA > 0 ? drive_kA : driveKV() * 0.13; }

// Top speed for a profiled move or path (in/s)
double profileSpeed(double requested) {
  if (requested > 0) return requested;
  if (profile_max_speed > 0) return profile_max_speed;
  return (12 - drive_kS) / driveKV() * 0.8;
}

/*
 * resetWheelSpeeds / updateWheelSpeeds
 * Wheel speed estimates from the drive encoders, lightly filtered. Reset at the start of a
 * motion (from the motors' own speed readings), then update once per loop.
 */
void resetWheelSpeeds() {
  left_wheel_speed = left_chassis.velocity(rpm) * wheel_distance_in / 60.0;
  right_wheel_speed = right_chassis.velocity(rpm) * wheel_distance_in / 60.0;
  last_left_deg = getLeftRotationDegree();
  last_right_deg = getRightRotationDegree();
  last_speed_time = Brain.timer(msec);
}

void updateWheelSpeeds() {
  double now = Brain.timer(msec), left_deg = getLeftRotationDegree(), right_deg = getRightRotationDegree();
  if (last_speed_time >= 0 && now > last_speed_time) {
    double dt = (now - last_speed_time) / 1000.0;
    left_wheel_speed = 0.6 * left_wheel_speed + 0.4 * (left_deg - last_left_deg) / 360.0 * wheel_distance_in / dt;
    right_wheel_speed = 0.6 * right_wheel_speed + 0.4 * (right_deg - last_right_deg) / 360.0 * wheel_distance_in / dt;
  }
  last_left_deg = left_deg;
  last_right_deg = right_deg;
  last_speed_time = now;
}

/*
 * driveWheelSpeeds
 * Drives each side at a target speed using the feedforward model (drive_kS/kV/kA) plus a
 * little feedback on the measured speed. If a side needs more than 12 V, both scale down
 * together so the robot still curves the right way.
 * - left_speed, right_speed: Target wheel speeds (in/s).
 * - left_accel, right_accel: Target accelerations (in/s^2), for the kA term.
 */
void driveWheelSpeeds(double left_speed, double right_speed, double left_accel, double right_accel) {
  updateWheelSpeeds();
  double kV = driveKV(), kA = driveKA();
  double left_volts = feedforwardVolts(left_speed, left_accel, drive_kS, kV, kA) + drive_velocity_kP * (left_speed - left_wheel_speed);
  double right_volts = feedforwardVolts(right_speed, right_accel, drive_kS, kV, kA) + drive_velocity_kP * (right_speed - right_wheel_speed);
  double biggest = fmax(fabs(left_volts), fabs(right_volts));
  if (biggest > 12) {
    left_volts = left_volts / biggest * 12;
    right_volts = right_volts / biggest * 12;
  }
  driveChassis(left_volts, right_volts);
}

// ============================================================================
// PROFILED MOVES
// ============================================================================

/*
 * driveProfiled
 * Drives straight with a motion profile: speeds up smoothly, cruises, and slows down to stop
 * on target, using feedforward plus feedback instead of a distance PID. Holds the heading.
 * - distance_in: Distance to drive (negative = backward).
 * - time_limit_msec: Maximum time allowed (in milliseconds).
 * - exit: If true, stops on target; if false, ends still moving to chain into the next motion.
 * - max_speed: Top speed (in/s, 0 = profile_max_speed).
 */
void driveProfiled(double distance_in, double time_limit_msec, bool exit, double max_speed) {
  beginMotion();
  resetWheelSpeeds();
  double speed = profileSpeed(max_speed);
  double direction = distance_in < 0 ? -1 : 1;
  double current_speed = fmax(0, direction * (left_wheel_speed + right_wheel_speed) / 2);
  MotionProfile profile = makeProfile(distance_in, speed, profile_max_accel, current_speed, exit ? 0 : speed * 0.5);
  double start_left = getLeftRotationDegree(), start_right = getRightRotationDegree();
  double heading_target = normalizeTarget(correct_angle);
  double start_time = Brain.timer(msec);

  while (motionActive(start_time, time_limit_msec)) {
    double t = (Brain.timer(msec) - start_time) / 1000.0;
    double planned_position = 0, planned_speed = 0, planned_accel = 0;
    profileAt(profile, t, planned_position, planned_speed, planned_accel);
    double traveled = ((getLeftRotationDegree() - start_left) + (getRightRotationDegree() - start_right)) / 2.0 / 360.0 * wheel_distance_in;
    double position_error = planned_position - traveled;
    // Done once the plan is over and the robot is on target (or right away when chaining)
    if (t >= profile.total_time && (!exit || fabs(position_error) < 0.5 || t > profile.total_time + 0.75)) {
      break;
    }
    // Catch up if behind the plan, and steer back to the heading
    double speed_command = planned_speed + 5.0 * position_error;
    double turn_command = 0.8 * (heading_target - getInertialHeading());
    driveWheelSpeeds(speed_command + turn_command, speed_command - turn_command, planned_accel, planned_accel);
    wait(10, msec);
  }
  if (exit) {
    stopChassis(vex::hold);
  }
  endMotion();
}

/*
 * turnProfiled
 * Turns in place with a motion profile: speeds up, turns, and slows down to stop on the angle.
 * - angle_deg: Heading to turn to (in degrees).
 * - time_limit_msec: Maximum time allowed (in milliseconds).
 * - exit: If true, stops on target; if false, ends still turning to chain into the next motion.
 * - max_speed: Top turning speed (degrees/s, 0 = profile_turn_speed).
 */
void turnProfiled(double angle_deg, double time_limit_msec, bool exit, double max_speed) {
  beginMotion();
  resetWheelSpeeds();
  double target = normalizeTarget(angle_deg);
  double start_heading = getInertialHeading();
  double speed = max_speed > 0 ? max_speed : profile_turn_speed;
  MotionProfile profile = makeProfile(target - start_heading, speed, profile_turn_accel, 0, exit ? 0 : speed * 0.5);
  double half_track = distance_between_wheels / 2.0;
  double start_time = Brain.timer(msec);

  while (motionActive(start_time, time_limit_msec)) {
    double t = (Brain.timer(msec) - start_time) / 1000.0;
    double planned_angle = 0, planned_rate = 0, planned_accel = 0;
    profileAt(profile, t, planned_angle, planned_rate, planned_accel);
    double heading = getInertialHeading();
    if (t >= profile.total_time && (!exit || fabs(target - heading) < 1 || t > profile.total_time + 0.75)) {
      break;
    }
    // Turn rate from the plan, plus a correction for being ahead of or behind it
    double rate_command = planned_rate + 6.0 * (start_heading + planned_angle - heading);
    double wheel_speed = degToRad(rate_command) * half_track;
    double wheel_accel = degToRad(planned_accel) * half_track;
    driveWheelSpeeds(wheel_speed, -wheel_speed, wheel_accel, -wheel_accel);
    wait(10, msec);
  }
  if (exit) {
    stopChassis(vex::hold);
  }
  correct_angle = target;
  endMotion();
}

// ============================================================================
// PATH FOLLOWING
// ============================================================================

// True once the robot has reached the end of a path (or driven past it)
bool atPathEnd(const Path& path, int closest) {
  int n = (int)path.samples.size();
  const PathSample& end = path.samples[n - 1];
  double past = (x_pos - end.x) * sin(degToRad(end.heading_deg)) + (y_pos - end.y) * cos(degToRad(end.heading_deg));
  return closest >= n - 3 && (hypot(end.x - x_pos, end.y - y_pos) < 1.0 || past > 0);
}

/*
 * followPath
 * Follows a path with pure pursuit: the robot chases a point path_lookahead inches ahead on the
 * path, slowing down for curves and at the end. Odometry must use the same coordinates as the path.
 * - path: From pathFromPoints() or pathFromJerryIO().
 * - time_limit_msec: Maximum time allowed (in milliseconds).
 * - reverse: If true, drives the path backward (back of the robot first).
 * - exit: If true, stops at the end; if false, ends still moving to chain into the next motion.
 * - max_speed: Top speed (in/s, 0 = profile_max_speed).
 * - lookahead: Lookahead distance (inches, 0 = path_lookahead).
 */
void followPath(const Path& path, double time_limit_msec, bool reverse, bool exit, double max_speed, double lookahead) {
  beginMotion();
  int n = (int)path.samples.size();
  if (n < 2) {
    printf("followPath: the path is empty, skipping it\n");
    endMotion();
    return;
  }
  resetWheelSpeeds();
  double speed = profileSpeed(max_speed);
  double lookahead_in = lookahead > 0 ? lookahead : path_lookahead;
  std::vector<double> speeds = planPathSpeeds(path, speed, profile_max_accel, path_max_lateral_accel, 0, exit ? 0 : speed * 0.5, false);
  double direction = reverse ? -1 : 1;
  double command = fmax(0, direction * (left_wheel_speed + right_wheel_speed) / 2);
  double half_track = distance_between_wheels / 2.0;
  int closest = 0;
  double start_time = Brain.timer(msec), last_time = start_time;

  while (motionActive(start_time, time_limit_msec)) {
    double now = Brain.timer(msec), dt = fmax((now - last_time) / 1000.0, 0.001);
    last_time = now;
    closest = closestSample(path, x_pos, y_pos, closest);
    if (atPathEnd(path, closest)) {
      break;
    }
    // Chase the lookahead point (or the next stretch of path if the robot is way off it)
    double target_x = 0, target_y = 0;
    if (!lookaheadPoint(path, x_pos, y_pos, lookahead_in, closest, target_x, target_y)) {
      int ahead = closest + (int)lookahead_in < n - 1 ? closest + (int)lookahead_in : n - 1;
      target_x = path.samples[ahead].x;
      target_y = path.samples[ahead].y;
    }
    double travel_heading = getInertialHeading() + (reverse ? 180 : 0);
    double curvature = curvatureToPoint(x_pos, y_pos, travel_heading, target_x, target_y);

    // Planned speed here, rising no faster than profile_max_accel (and creeping in at the end)
    double accel = 0;
    if (speeds[closest] > command) {
      command = fmin(speeds[closest], command + profile_max_accel * dt);
      accel = profile_max_accel;
    } else {
      command = speeds[closest];
    }
    command = fmax(command, exit ? 4.0 : 0);

    // Faster on the outside of the curve; backward driving mirrors which side is outside
    double outside = curvature * half_track * (reverse ? -1 : 1);
    double v = direction * command;
    driveWheelSpeeds(v * (1 + outside), v * (1 - outside), direction * accel * (1 + outside), direction * accel * (1 - outside));
    wait(10, msec);
  }
  if (exit) {
    stopChassis(vex::hold);
  }
  correct_angle = getInertialHeading();
  endMotion();
}

/*
 * followPathRamsete
 * Follows a path on a timetable with RAMSETE: plans when to be where (slowing for curves),
 * then steers and adjusts speed to stay on schedule. Very repeatable timing.
 * Odometry must use the same coordinates as the path.
 * - path: From pathFromPoints() or pathFromJerryIO().
 * - time_limit_msec: Maximum time allowed (in milliseconds).
 * - reverse: If true, drives the path backward (back of the robot first).
 * - exit: If true, stops at the end; if false, ends still moving to chain into the next motion.
 * - max_speed: Top speed (in/s, 0 = profile_max_speed).
 */
void followPathRamsete(const Path& path, double time_limit_msec, bool reverse, bool exit, double max_speed) {
  beginMotion();
  int n = (int)path.samples.size();
  if (n < 2) {
    printf("followPathRamsete: the path is empty, skipping it\n");
    endMotion();
    return;
  }
  resetWheelSpeeds();
  double direction = reverse ? -1 : 1;
  double speed = profileSpeed(max_speed);
  double current_speed = fmax(0, direction * (left_wheel_speed + right_wheel_speed) / 2);
  std::vector<double> speeds = planPathSpeeds(path, speed, profile_max_accel, path_max_lateral_accel, current_speed, exit ? 0 : speed * 0.5, true);
  std::vector<double> times = planPathTimes(path, speeds);
  double half_track = distance_between_wheels / 2.0;
  int index = 0;
  double start_time = Brain.timer(msec);

  while (motionActive(start_time, time_limit_msec)) {
    double t = (Brain.timer(msec) - start_time) / 1000.0;
    // Where the timetable says to be now (between two samples)
    while (index + 1 < n - 1 && times[index + 1] <= t) index++;
    const PathSample& a = path.samples[index];
    const PathSample& b = path.samples[index + 1];
    double span = times[index + 1] - times[index];
    double f = span > 1e-9 ? fmin(fmax((t - times[index]) / span, 0.0), 1.0) : 1.0;
    double ref_x = a.x + (b.x - a.x) * f, ref_y = a.y + (b.y - a.y) * f;
    double ref_heading = a.heading_deg + remainder(b.heading_deg - a.heading_deg, 360.0) * f;
    double ref_speed = speeds[index] + (speeds[index + 1] - speeds[index]) * f;
    double ref_curvature = a.curvature + (b.curvature - a.curvature) * f;
    double ref_accel = span > 1e-9 ? (speeds[index + 1] - speeds[index]) / span : 0;
    if (t >= times[n - 1]) {
      // Past the end of the timetable: hold the end point
      ref_x = path.samples[n - 1].x;
      ref_y = path.samples[n - 1].y;
      ref_heading = path.samples[n - 1].heading_deg;
      ref_speed = speeds[n - 1];
      ref_curvature = 0;
      ref_accel = 0;
    }

    // At the end of the timetable: finish (settling on the end point and heading if exit is true)
    double end_error = hypot(path.samples[n - 1].x - x_pos, path.samples[n - 1].y - y_pos);
    double end_heading_error = fabs(remainder(path.samples[n - 1].heading_deg + (reverse ? 180 : 0) - getInertialHeading(), 360.0));
    if (t >= times[n - 1] && (!exit || (end_error < 1.0 && end_heading_error < 2.0) || t > times[n - 1] + 0.3)) {
      break;
    }

    // Backward: the robot faces away from the direction of travel
    double robot_heading = getInertialHeading();
    double speed_command = 0, turn_command = 0;
    ramsete(x_pos, y_pos, robot_heading, ref_x, ref_y, ref_heading + (reverse ? 180 : 0),
            direction * ref_speed, ref_speed * ref_curvature, speed_command, turn_command);
    double turn_speed = turn_command * half_track;
    driveWheelSpeeds(speed_command + turn_speed, speed_command - turn_speed, direction * ref_accel, direction * ref_accel);
    wait(10, msec);
  }
  if (exit) {
    stopChassis(vex::hold);
  }
  correct_angle = getInertialHeading();
  endMotion();
}

// ============================================================================
// MEASURING THE DRIVETRAIN
// ============================================================================

/*
 * measureDriveFeedforward
 * Measures drive_kS, drive_kV and drive_kA for your drivetrain. Give the robot about 4 feet of
 * open space in front and behind, run it (e.g. from runDriver), and copy the numbers on the
 * Brain screen into custom/src/robot-config.cpp. Never returns.
 */
void measureDriveFeedforward() {
  const double volts_steps[6] = {3, -3, 6, -6, 9, -9};
  double steady_speed[6];
  double accel_sum = 0, accel_speed_sum = 0;
  int accel_count = 0;
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Measuring the drivetrain...");

  for (int i = 0; i < 6; i++) {
    resetWheelSpeeds();
    double start = Brain.timer(msec), speed_sum = 0, early_speed = 0, early_time = -1;
    int speed_count = 0;
    while (Brain.timer(msec) - start < 800) {
      driveChassis(volts_steps[i], volts_steps[i]);
      updateWheelSpeeds();
      double t = Brain.timer(msec) - start;
      double speed = fabs(left_wheel_speed + right_wheel_speed) / 2;
      // Steady speed near the end of each step gives kS and kV
      if (t > 550) {
        speed_sum += speed;
        speed_count++;
      }
      // How fast the 9 V steps speed up gives kA
      if (fabs(volts_steps[i]) == 9) {
        if (t >= 30 && early_time < 0) {
          early_time = t;
          early_speed = speed;
        } else if (early_time >= 0 && t >= early_time + 100 && t < early_time + 110) {
          accel_sum += (speed - early_speed) / ((t - early_time) / 1000.0);
          accel_speed_sum += (speed + early_speed) / 2;
          accel_count++;
        }
      }
      wait(10, msec);
    }
    stopChassis(vex::brake);
    wait(500, msec);
    steady_speed[i] = speed_count > 0 ? speed_sum / speed_count : 0;
  }

  // Straight line through (speed, volts): volts = kS + kV * speed
  double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
  for (int i = 0; i < 6; i++) {
    double x = steady_speed[i], y = fabs(volts_steps[i]);
    sum_x += x; sum_y += y; sum_xx += x * x; sum_xy += x * y;
  }
  double kV = (6 * sum_xy - sum_x * sum_y) / fmax(6 * sum_xx - sum_x * sum_x, 1e-9);
  double kS = (sum_y - kV * sum_x) / 6;
  double kA = 0;
  if (accel_count > 0 && accel_sum > 0) {
    double accel = accel_sum / accel_count, mid_speed = accel_speed_sum / accel_count;
    kA = (9 - kS - kV * mid_speed) / accel;
  }

  printf("drive_kS = %.3f; drive_kV = %.4f; drive_kA = %.4f\n", kS, kV, kA);
  while (true) {
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Put these in custom/src/robot-config.cpp:");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("double drive_kS = %.3f;", kS);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("double drive_kV = %.4f;", kV);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("double drive_kA = %.4f;", kA);
    wait(200, msec);
  }
}
