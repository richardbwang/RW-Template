#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include "motor-control.h"
#include "autonomous.h"

// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================

// Distance between the middles of the left and right wheels of the drive (in inches)
double distance_between_wheels = 14;

// motor to wheel gear ratio * wheel diameter (in inches) * pi
double wheel_distance_in = (48.0 / 84.0) * 4 * M_PI;

// PID Constants for movement
// distance_* : Linear PID for straight driving
// turn_*     : PID for turning in place
// heading_correction_* : PID for heading correction during linear movement
double distance_kp = 2, distance_ki = 0.1, distance_kd = 8;
double turn_kp = 0.5, turn_ki = 0.1, turn_kd = 3;
double heading_correction_kp = 1, heading_correction_ki = 0, heading_correction_kd = 5;

bool using_tracking_wheels = false; // Set to true if you are using tracking wheels

// IGNORE THESE IF YOU ARE NOT USING TRACKING WHEELS
// These comments are in the perspective of a top down view of the robot when the robot is facing forward
// Vertical distance from the center of the bot to the sideways tracker wheel (in inches, positive is when the wheel is behind the center)
double sideways_tracker_dist_from_center = 0;
// Horizontal distance from the center of the bot to the forward tracker wheel (in inches, positive is when the wheel is to the right of the center)
double forward_tracker_dist_from_center = 0;
double forward_tracker_diameter = 2.75; // Diameter of the forward tracker wheel (in inches)
double sideways_tracker_diameter = 2.75; // Diameter of the sideways tracker wheel (in inches)

// ============================================================================
// ADVANCED TUNING (OPTIONAL)
// ============================================================================

bool heading_correction = true; // Use heading correction when the bot is stationary

// Set to true for more accuracy and smoothness, false for more speed
bool dir_change_start = true;   // Allow direction change at start of movement
bool dir_change_end = true;     // Allow direction change at end of movement

double min_output = 8; // Minimum output voltage to motors while chaining movements

// Maximum allowed change in voltage output per 10 msec during movement
double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;

// Prevents too much slipping during boomerang movements
// Decrease if there is too much drifting and inconsistency during boomerang
// Increase for more speed during boomerang
double chase_power = 2;

// ============================================================================
// INTERNAL STATE (DO NOT CHANGE)
// ============================================================================
bool is_turning = false;
double prev_left_output = 0, prev_right_output = 0;
double xpos = 0, ypos = 0;
double correct_angle = 0;

// ============================================================================
// CHASSIS CONTROL FUNCTIONS
// ============================================================================

void driveChassis(double left_power, double right_power) {
  left_chassis.spin(fwd, left_power, voltageUnits::volt);
  right_chassis.spin(fwd, right_power, voltageUnits::volt);
}

void stopChassis(brakeType type) {
  left_chassis.stop(type);
  right_chassis.stop(type);
}

void resetChassis() {
  left_chassis.setPosition(0, degrees);
  right_chassis.setPosition(0, degrees);
}

double getLeftRotationDegree() {
  return left_chassis.position(degrees);
}

double getRightRotationDegree() {
  return right_chassis.position(degrees);
}

double normalizeTarget(double angle) {
  // Normalize angle to be within +/-180 degrees of the current heading
  if (angle - inertial_sensor.rotation() > 180) {
    while (angle - inertial_sensor.rotation() > 180) angle -= 360;
  } else if (angle - inertial_sensor.rotation() < -180) {
    while (angle - inertial_sensor.rotation() < -180) angle += 360;
  }
  return angle;
}

double getInertialHeading(bool normalize) {
  return inertial_sensor.rotation(degrees);
}

// ============================================================================
// OUTPUT SCALING HELPER FUNCTIONS
// ============================================================================
// Ensures output values stay within specified min/max bounds for safe and consistent control.

void scaleToMin(double& left_output, double& right_output, double min_output) {
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

void scaleToMax(double& left_output, double& right_output, double max_output) {
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
// MAIN DRIVE AND TURN FUNCTIONS
// ============================================================================
// Each function below is documented with its purpose and parameters.
// All logic is modular and uses PID for smooth, accurate movement.

/*
 * TurnToAngle
 * Turns the robot to a specified angle using PID control.
 * - turn_angle: Target angle to turn to (in degrees).
 * - time_limit_msec: Maximum time allowed for the turn (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void turnToAngle(double turn_angle, double time_limit_msec, bool exit, double max_output) {
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 1;
  PID pid = PID(turn_kp, turn_ki, turn_kd);
  
  turn_angle = normalizeTarget(turn_angle);
  pid.setTarget(turn_angle);
  pid.setIntegralMax(0);  
  pid.setIntegralRange(3);
  
  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(50, 250);
  pid.setDerivativeTolerance(threshold * 4.5);

  // Draw the baseline.
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 
                        600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);
  
  // Start the PID loop.
  double start_time = Brain.timer(msec);
  double output;
  double current_heading = getInertialHeading();
  double previous_heading = 0;
  int index = 1;
  if(exit == false && correct_angle < turn_angle) {
    while (getInertialHeading() < turn_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output < min_output) {
        output = min_output;
      }
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }
      // End
      driveChassis(output, -output);
      wait(10, msec);
    }
  } else if(exit == false && correct_angle > turn_angle) {
    while (getInertialHeading() > turn_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output < min_output) {
        output = min_output;
      }
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }
      // End
      driveChassis(-output, output);
      wait(10, msec);
    }
  } else {
    while (!pid.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }
      // End
      driveChassis(output, -output);
      wait(10, msec);
    }
  }
  if(exit) {
    stopChassis(vex::hold);
  }
  correct_angle = turn_angle;
  is_turning = false;
}

/*
 * DriveTo
 * Drives the robot a specified distance (in inches) using PID control.
 * - distance_in: Target distance to drive (positive or negative).
 * - time_limit_msec: Maximum time allowed for the move (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void driveTo(double distance_in, double time_limit_msec, bool exit, double max_output) {
  double start_left = getLeftRotationDegree(), start_right = getRightRotationDegree();
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 0.5;
  int drive_direction = distance_in > 0 ? 1 : -1;
  double max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;;
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

  distance_in = distance_in * drive_direction;
  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

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

  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0;

  double current_distance = 0, current_angle = 0;

  while (((!pid_distance.targetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec && exit) || (exit == false && current_distance < distance_in && Brain.timer(msec) - start_time <= time_limit_msec)) {
    current_distance = (fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in) + fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in)) / 2;
    current_angle = getInertialHeading();
    left_output = pid_distance.update(current_distance) * drive_direction;
    right_output = left_output;
    correction_output = pid_heading.update(current_angle);

    //Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }
    if(!exit) {
      left_output = 24 * drive_direction;
      right_output = 24 * drive_direction;
    }

    left_output += correction_output;
    right_output -= correction_output;

    //Max Output Check
    scaleToMax(left_output, right_output, max_output);

    //Max Acceleration/Deceleration Check
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
    wait(10, msec);
  }
  if(exit) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::hold);
  }
  is_turning = false;
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
  double start_right = getRightRotationDegree(), start_left = getLeftRotationDegree();
  double in_arc, out_arc;
  double real_angle = 0, current_angle = 0;
  double ratio, result_angle;
  result_angle_deg = normalizeTarget(result_angle_deg);
  result_angle = (result_angle_deg - correct_angle) * 3.14159265359 / 180;
  in_arc = fabs((fabs(center_radius) - (distance_between_wheels / 2)) * result_angle);
  out_arc = fabs((fabs(center_radius) + (distance_between_wheels / 2)) * result_angle);
  ratio = in_arc / out_arc;
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  // Tuned parameters, DO NOT CHANGE!!!
  double threshold = 0.5;

  int curve_direction = center_radius > 0 ? 1 : -1;
  int drive_direction = 0;
  if ((curve_direction == 1 && (result_angle_deg - correct_angle) > 0) || (curve_direction == -1 && (result_angle_deg - correct_angle) < 0)) {
    drive_direction = 1;
  } else {
    drive_direction = -1;
  }
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

  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_right = 0, current_left = 0;
  
  if (curve_direction == -1 && exit == true) {
    while (!pid_out.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_right = fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in);
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      right_output = pid_out.update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pid_turn.update(current_angle);

      //Minimum Output Check
      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      //Max Output Check
      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else if (curve_direction == 1 && exit == true) {
    while (!pid_out.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_left = fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      left_output = pid_out.update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pid_turn.update(current_angle);

      //Minimum Output Check
      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      //Max Output Check
      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else if (curve_direction == -1 && exit == false) {
    while (current_right < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_right = fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in);
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      right_output = pid_out.update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pid_turn.update(current_angle);

      //Minimum Output Check
      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }
      
      left_output += correction_output;
      right_output -= correction_output;

      //Max Output Check
      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else {
    while (current_left < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_left = fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      left_output = pid_out.update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pid_turn.update(current_angle);

      //Minimum Output Check
      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      //Max Output Check
      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  }
  if(exit == true) {
    stopChassis(vex::brakeType::hold);
  }
  correct_angle = result_angle_deg;
  is_turning = false;
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
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 1;
  PID pid = PID(turn_kp, turn_ki, turn_kd);
  
  swing_angle = normalizeTarget(swing_angle);
  pid.setTarget(swing_angle);
  pid.setIntegralMax(0);  
  pid.setIntegralRange(5);
  
  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(50, 250);
  pid.setDerivativeTolerance(threshold * 4.5);
  
  // Draw the baseline.
  double draw_amplifier = 230 / fabs(swing_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(swing_angle) * draw_amplifier, 
                        600, fabs(swing_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);
  
  // Start the PID loop.
  double start_time = Brain.timer(msec);
  double output;
  double current_heading = correct_angle;
  double previous_heading = 0;
  int index = 1;
  int choice = 1;
  if(swing_angle - correct_angle < 0 && drive_direction == 1) {
    choice = 1;
  } else if(swing_angle - correct_angle > 0 && drive_direction == 1) {
    choice = 2;
  } else if(swing_angle - correct_angle < 0 && drive_direction == -1) {
    choice = 3;
  } else {
    choice = 4;
  }
  if(choice == 1 && exit == false) {
    while (current_heading > swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      // End
      if(output < min_output) {
        output = min_output;
      }
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }

      left_chassis.stop(hold);
      right_chassis.spin(fwd, output * drive_direction, volt);
      wait(10, msec);
    }
  } else if(choice == 2 && exit == false) {
    while (current_heading < swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      // End
      if(output < min_output) {
        output = min_output;
      }
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }

      left_chassis.spin(fwd, output * drive_direction, volt);
      right_chassis.stop(hold);
      wait(10, msec);
    }
  } else if(choice == 3 && exit == false) {
    while (current_heading > swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      // End
      if(output < min_output) {
        output = min_output;
      }
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }

      left_chassis.spin(fwd, output * drive_direction, volt);
      right_chassis.stop(hold);
      wait(10, msec);
    }
  } else {
    while (current_heading < swing_angle && Brain.timer(msec) - start_time <= time_limit_msec && exit == false) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      
      // Draw line
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      // End
      if(output < min_output) {
        output = min_output;
      }
      if(output > max_output) {
        output = max_output;
      } else if(output < -max_output) {
        output = -max_output;
      }

      left_chassis.stop(hold);
      right_chassis.spin(fwd, output * drive_direction, volt);
      wait(10, msec);
    }
  }
  while (!pid.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec && exit == true) {
    current_heading = getInertialHeading();
    output = pid.update(current_heading);
    
    // Draw line
    Brain.Screen.drawLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;
    // End

    if(output > max_output) {
      output = max_output;
    } else if(output < -max_output) {
      output = -max_output;
    }

    switch(choice) {
    case 1:
      left_chassis.stop(hold);
      right_chassis.spin(fwd, -output * drive_direction, volt);
      break;
    case 2:
      left_chassis.spin(fwd, output * drive_direction, volt);
      right_chassis.stop(hold);
      break;
    case 3:
      left_chassis.spin(fwd, -output * drive_direction, volt);
      right_chassis.stop(hold);
      break;
    case 4:
      left_chassis.stop(hold);
      right_chassis.spin(fwd, output * drive_direction, volt);
      break;
    }
    wait(10, msec);
  }
  if(exit == true) {
    stopChassis(vex::hold);
  }
  correct_angle = swing_angle;
  is_turning = false;
}

/*
 * heading_correction
 * Continuously adjusts the robot's heading to maintain a straight course.
 * Uses a PID controller to minimize the error between the current heading and the target heading.
 */
void correctHeading() {
  double output = 0;
  PID pid = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);
  
  pid.setTarget(correct_angle);
  //pid.setIntegralMax(300);  
  pid.setIntegralRange(fabs(correct_angle) / 2.5);
  
  pid.setSmallBigErrorTolerance(0, 0);
  pid.setSmallBigErrorDuration(0, 0);
  pid.setDerivativeTolerance(0);
  pid.setArrive(false);
  
  while(heading_correction) {
    pid.setTarget(correct_angle);
    if(is_turning == false) {
      output = pid.update(getInertialHeading());
      driveChassis(output, -output);
    }
    wait(10, msec);
  }
}

/*
 * trackOdom
 * Tracks the robot's position using odometry based on drivetrain wheel rotations and inertial sensor data.
 * Continuously updates the global xpos, ypos, and heading variables.
 */
void trackOdom() {
  resetChassis();
  double left_deg = 0, right_deg = 0;
  double delta_left = 0, delta_right = 0;
  double heading_rad = 0, delta_heading = 0;
  double delta_y_left = 0, delta_y_right = 0, delta_y = 0;

  while (true) {
    delta_heading = degToRad(getInertialHeading()) - heading_rad;
    delta_left = (getLeftRotationDegree() - left_deg) * wheel_distance_in / 360.0;
    delta_right = (getRightRotationDegree() - right_deg) * wheel_distance_in / 360.0;
    // If no heading change, treat as straight movement
    if (fabs(delta_heading) < 1e-9) {
      delta_y = (delta_left + delta_right) / 2.0;
    } else {
      delta_y_left = 2.0 * sin(delta_heading / 2.0) * (delta_left / delta_heading + distance_between_wheels / 2.0);
      delta_y_right = 2.0 * sin(delta_heading / 2.0) * (delta_right / delta_heading + distance_between_wheels / 2.0);
      delta_y = (delta_y_left + delta_y_right) / 2.0;
    }
    xpos += delta_y * sin(heading_rad + delta_heading / 2.0);
    ypos += delta_y * cos(heading_rad + delta_heading / 2.0);
    heading_rad = degToRad(getInertialHeading());
    left_deg = getLeftRotationDegree();
    right_deg = getRightRotationDegree();
    wait(10, msec);
  }
}

/*
 * trackOdomWheel
 * Tracks the robot's position using odometry based on tracker wheel rotations and inertial sensor data.
 * Continuously updates the global xpos, ypos, and heading variables.
 */
void trackOdomWheel() {
  xpos = 0;
  ypos = 0;
  resetChassis();
  double prev_heading = degToRad(getInertialHeading());
  double prev_x = sideways_tracker.position(degrees);
  double prev_y = forward_tracker.position(degrees);

  while (true) {
    double curr_x = sideways_tracker.position(degrees);
    double curr_y = forward_tracker.position(degrees);
    double forward_delta = (curr_y - prev_y) * forward_tracker_diameter * M_PI / 360.0;
    double sideways_delta = (curr_x - prev_x) * sideways_tracker_diameter * M_PI / 360.0;
    double new_heading = degToRad(getInertialHeading());
    double delta_heading = new_heading - prev_heading;
    prev_x = curr_x;
    prev_y = curr_y;

    double local_x_pos, local_y_pos;

    if (fabs(delta_heading) < 1e-9) {
      local_x_pos = sideways_delta;
      local_y_pos = forward_delta;
    } else {
      double sin_calc = 2 * sin(delta_heading / 2);
      local_x_pos = sin_calc * ((sideways_delta / delta_heading) + sideways_tracker_dist_from_center);
      local_y_pos = sin_calc * ((forward_delta / delta_heading) + forward_tracker_dist_from_center);
    }

    double local_polar_angle, local_polar_length;
    if (fabs(local_x_pos) < 1e-9 && fabs(local_y_pos) < 1e-9) {
      local_polar_angle = 0;
      local_polar_length = 0;
    } else {
      local_polar_angle = atan2(local_y_pos, local_x_pos);
      local_polar_length = sqrt(pow(local_x_pos, 2) + pow(local_y_pos, 2));
    }

    double global_polar_angle = local_polar_angle - prev_heading - (delta_heading / 2);

    double x_pos_delta = local_polar_length * cos(global_polar_angle);
    double y_pos_delta = local_polar_length * sin(global_polar_angle);
    xpos += x_pos_delta;
    ypos += y_pos_delta;
    prev_heading = new_heading;
    wait(10, msec);
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
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 1, add = 0;
  if(direction == -1) {
    add = 180;
  }
  double turn_angle = normalizeTarget(radToDeg(atan2(x - xpos, y - ypos))) + add;
  PID pid = PID(turn_kp, turn_ki, turn_kd);
  
  pid.setTarget(turn_angle);
  pid.setIntegralMax(0);  
  pid.setIntegralRange(3);
  
  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(100, 500);
  pid.setDerivativeTolerance(threshold * 4.5);
  
  // Draw the baseline.
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 
                        600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);
  
  // Start the PID loop.
  double start_time = Brain.timer(msec);
  double output;
  double current_heading;
  double previous_heading = 0;
  int index = 1;
  while (!pid.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
    pid.setTarget(normalizeTarget(radToDeg(atan2(x - xpos, y - ypos))) + add);
    current_heading = getInertialHeading();
    output = pid.update(current_heading);
    
    // Draw line
    Brain.Screen.drawLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;
    // End
    driveChassis(output, -output);
    wait(10, msec);
  }  
  stopChassis(vex::hold);
  correct_angle = getInertialHeading();
  is_turning = false;
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
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
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

  //double maxslewfwd = 0.9;
  //double maxslewrev = 0.3;
  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid_distance.setTarget(hypot(x - xpos, y - ypos));
  pid_distance.setIntegralMax(0);  
  pid_distance.setIntegralRange(3);
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);
  
  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - xpos, y - ypos)) + add));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  // Reset the chassis.
  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0, prev_left_output = 0, prev_right_output = 0;
  double exittolerance = 1;
  bool perpendicular_line = false, prev_perpendicular_line = true;

  double current_angle = 0, overturn_value = 0;
  bool ch = true;

  while (Brain.timer(msec) - start_time <= time_limit_msec) {
    pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - xpos, y - ypos)) + add));
    pid_distance.setTarget(hypot(x - xpos, y - ypos));
    current_angle = getInertialHeading();
    left_output = pid_distance.update(0) * cos(degToRad(atan2(x - xpos, y - ypos) * 180 / M_PI + add - current_angle)) * dir;
    right_output = left_output;
    perpendicular_line = ((ypos - y) * -cos(degToRad(normalizeTarget(current_angle + add))) <= (xpos - x) * sin(degToRad(normalizeTarget(current_angle + add))) + exittolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    if(hypot(x - xpos, y - ypos) > 8 && ch == true) {
      correction_output = pid_heading.update(current_angle);
    } else {
      correction_output = 0;
      ch = false;
    }

    //Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }

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

    //Max Output Check
    scaleToMax(left_output, right_output, max_output);

    //Max Acceleration/Deceleration Check
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
    wait(10, msec);
  }
  if(exit == true) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::hold);
  }
  correct_angle = getInertialHeading();
  is_turning = false;
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
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
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

  pid_distance.setTarget(0);
  pid_distance.setIntegralMax(3);  
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - xpos, y - ypos))));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0, slip_speed = 0, overturn_value = 0;
  double exit_tolerance = 3;
  bool perpendicular_line = false, prev_perpendicular_line = true;
  double current_angle = 0, hypotenuse = 0, carrot_x = 0, carrot_y = 0;

  while ((!pid_distance.targetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec) {
    hypotenuse = hypot(xpos - x, ypos - y);
    carrot_x = x - hypotenuse * sin(degToRad(a + add)) * dlead;
    carrot_y = y - hypotenuse * cos(degToRad(a + add)) * dlead;
    pid_distance.setTarget(hypot(carrot_x - xpos, carrot_y - ypos) * dir);
    current_angle = getInertialHeading();
    left_output = pid_distance.update(0) * cos(degToRad(atan2(carrot_x - xpos, carrot_y - ypos) * 180 / M_PI + add - current_angle));
    right_output = left_output;
    perpendicular_line = ((ypos - y) * -cos(degToRad(normalizeTarget(a))) <= (xpos - x) * sin(degToRad(normalizeTarget(a))) + exit_tolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }

    if(hypot(carrot_x - xpos, carrot_y - ypos) > 8) {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(carrot_x - xpos, carrot_y - ypos)) + add));
      correction_output = pid_heading.update(current_angle);
    } else if(hypot(x - xpos, y - ypos) > 6) {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - xpos, y - ypos)) + add));
      correction_output = pid_heading.update(current_angle);
    } else {
      pid_heading.setTarget(normalizeTarget(a));
      correction_output = pid_heading.update(current_angle);
      if(exit && hypot(x - xpos, y - ypos) < 5) {
        break;
      }
    }

    slip_speed = sqrt(chase_power * getRadius(xpos, ypos, carrot_x, carrot_y, current_angle) * 9.8);
    if(left_output > slip_speed) {
      left_output = slip_speed;
    } else if(left_output < -slip_speed) {
      left_output = -slip_speed;
    }

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
    driveChassis(left_output, right_output);
    wait(10, msec);
  }
  if(exit) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::hold);
  }
  correct_angle = a;
  is_turning = false;
}

// ============================================================================
// TEMPLATE NOTE
// ============================================================================
// This file is intended as a template for VEX/V5 robotics teams.
// All functions and variables use clear, consistent naming conventions.
// Comments are concise and explain the intent of each section.
// Teams can adapt PID values, drive base geometry, and logic as needed for their robot.