#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller controller_1 = controller(primary);

// IMPORTANT: Remember to modify the example motors according to the guide. 
// Also remember to add respective device declarations to custom/include/robot-config.h
// Format: motor(port, gearSetting, reversed)
// gearSetting is one of the following: ratio36_1(red), ratio18_1(green), ratio6_1(blue)
// all chassis motors should be reversed appropriately so that they spin vertical when given a positive voltage input
// such as driveChassis(12, 12)
// Override (2026-27) rule R11 caps drivetrain motors at 55W total, e.g. 4 x 11W, 5 x 11W, or 4 x 11W + 2 x 5.5W.
// Drive encoder readings come from the FIRST motor in each motor_group, so with mixed motors
// list an 11W motor first and set wheel_distance_in for that motor's gearing.
motor left_chassis1 = motor(PORT1, ratio6_1, true);
motor left_chassis2 = motor(PORT2, ratio6_1, true);
motor left_chassis3 = motor(PORT3, ratio6_1, false);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2, left_chassis3);
motor right_chassis1 = motor(PORT4, ratio6_1, false);
motor right_chassis2 = motor(PORT5, ratio6_1, false);
motor right_chassis3 = motor(PORT6, ratio6_1, true);
motor_group right_chassis = motor_group(right_chassis1, right_chassis2, right_chassis3);

inertial inertial_sensor = inertial(PORT7);
optical example_optical_sensor = optical(PORT8);
distance example_distance_sensor = distance(PORT9);
digital_out example_piston = digital_out(Brain.ThreeWirePort.A);

// Format is rotation(port, reversed)
// just set these to random ports if you don't use tracking wheels
rotation horizontal_tracker = rotation(PORT10, true);
rotation vertical_tracker = rotation(PORT11, true);

// Distance reset sensors
// Set these to random ports if you are not using distance resets
distance front_sensor = distance(PORT12);
distance left_sensor = distance(PORT13);
distance right_sensor = distance(PORT14);
distance back_sensor = distance(PORT15);

// game specific devices for high stakes
motor arm_motor1 = motor(PORT16, ratio18_1, true);
motor arm_motor2 = motor(PORT17, ratio18_1, false);
motor_group arm_motor = motor_group(arm_motor1, arm_motor2);
motor intake_motor = motor(PORT18, ratio18_1, true);
digital_out claw = digital_out(Brain.ThreeWirePort.B);
digital_out rush_arm = digital_out(Brain.ThreeWirePort.C);
optical optical_sensor = optical(PORT19);
distance intake_distance = distance(PORT20);
distance clamp_distance = distance(PORT21);
digital_out mogo_mech = digital_out(Brain.ThreeWirePort.D);

// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================

// Distance between the middles of the left and right wheels of the drive (in inches)
double distance_between_wheels = 12.3;

// motor to wheel gear ratio * wheel diameter (in inches) * pi
double wheel_distance_in = (36.0 / 48.0) * 3.17 * M_PI;

// PID Constants for movement
// distance_* : Linear PID for straight driving
// turn_*     : PID for turning in place
// heading_correction_* : PID for heading correction during linear movement
double distance_kp = 1.1, distance_ki = 0.1, distance_kd = 7;
double turn_kp = 0.3, turn_ki = 0, turn_kd = 2.5;
double heading_correction_kp = 0.6, heading_correction_ki = 0, heading_correction_kd = 4;

// Enable or disable the use of tracking wheels
bool using_horizontal_tracker = false;  // Set to true if a horizontal tracking wheel is installed and used for odometry
bool using_vertical_tracker = false;   // Set to true if a vertical tracking wheel is installed and used for odometry

// IGNORE THESE IF YOU ARE NOT USING TRACKING WHEELS
// These comments are in the perspective of a top down view of the robot when the robot is facing vertical
// Vertical distance from the center of the bot to the horizontal tracking wheel (in inches, positive is when the wheel is behind the center)
double horizontal_tracker_dist_from_center = 2.71875;
// Horizontal distance from the center of the bot to the vertical tracking wheel (in inches, positive is when the wheel is to the right of the center)
double vertical_tracker_dist_from_center = -0.03125;
double horizontal_tracker_diameter = 1.975; // Diameter of the horizontal tracker wheel (in inches)
double vertical_tracker_diameter = 1.975; // Diameter of the vertical tracker wheel (in inches)

// Distance Reset setup
// If you are not using all four distance sensors, just set the unused ones to 0
// If you are not using distance resets these values will be ignored
// Add your sensor offsets here (where the sensor's face is, relative to the robot's center)
// If a sensor is dead-center but 6.5" forward relative to the robot's center:
// double front_sensor_offsetX = 0.0; 
// double front_sensor_offsetY = 6.5;
// For the X offset, a negative value indicates that the sensor is located on the left side of the robot, while a positive value indicates that it is on the right side. 

double front_sensor_offsetX = 0.0; 
double front_sensor_offsetY = 0.0;

double left_sensor_offsetX = 0.0; 
double left_sensor_offsetY = 0.0;

double right_sensor_offsetX = 0.0;
double right_sensor_offsetY = 0.0;

double back_sensor_offsetX = 0.0;
double back_sensor_offsetY = 0.0;

// ============================================================================
// ADVANCED TUNING (OPTIONAL)
// ============================================================================

bool heading_correction = true; // Use heading correction when the bot is stationary

// Set to true for more accuracy and smoothness, false for more speed
bool dir_change_start = true;   // Less accel/decel due to expecting direction change at start of movement
bool dir_change_end = true;     // Less accel/decel due to expecting direction change at end of movement

double min_output = 10; // Minimum output voltage to motors while chaining movements

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
// NEWER SETTINGS (OPTIONAL)
// Every setting here works at its default. Change one only when you use the feature it's for.
// These also have defaults in the core files, so custom folders without them still build.
// ============================================================================

// Multiplies the IMU heading. Many V5 IMUs read a little off over full turns (e.g. 357 for 360).
// Find yours with measureImuScale(), 1.0 = no correction
double imu_scale = 1.0;

// End a motion early once the robot has been stuck (not moving or turning) for this long, in msec.
// Saves time if the robot gets caught on something. 0 = off.
// Leave it off for moves meant to push into something until they time out. 350 or more avoids false exits.
double stall_exit_msec = 0;

// Distance resets: skip readings where the beam hits the wall at a steeper angle than this (degrees).
// The sensor's beam is a cone about 12 degrees wide on each side, so steep readings get less exact
double distance_reset_max_angle = 30;
// Distance resets: skip readings that would move the position more than this (inches, 0 = off).
// 6-10 helps ignore other robots and game elements in front of the wall
double distance_reset_max_jump = 0;
// Distance resets: skip readings that hit the wall this close to a corner (inches, 0 = off).
// For Override, about 18 keeps the corner loaders out of the readings
double distance_reset_corner_margin = 0;

// MCL (see startMcl() in custom/src/user.cpp): particle count (more = smoother but slower, up to 1000),
// longest distance reading it trusts (inches), and how far behind the distance sensor readings are (msec).
// If the position lags while driving toward or away from a wall, adjust the delay.
// The selector screen shows how long each MCL update takes; lower the particle count if it's more than a few msec
int mcl_particles = 200;
double mcl_max_range = 60;
double distance_sensor_delay_msec = 40;

// Multiplies every distance sensor reading (resets and MCL). V5 distance sensors often read 1-3% long.
// To measure yours: put the robot a tape-measured distance from a wall (2-4 ft), then use
// (measured distance) / (what the sensor reads). 1.0 = no correction
double distance_sensor_scale = 1.0;

// Motion profiles and path following (driveProfiled, turnProfiled, followPath, followPathRamsete).
// These use a model of your drive: drive_kS = volts to start moving, drive_kV = volts per in/s,
// drive_kA = volts per in/s^2. Leave kV and kA at 0 to estimate them from drive_motor_rpm and
// wheel_distance_in, or measure all three with measureDriveFeedforward()
double drive_kS = 0.5;
double drive_kV = 0;
double drive_kA = 0;
double drive_motor_rpm = 600;          // Drive motor cartridge: 600 (blue), 200 (green), 100 (red)
double profile_max_speed = 0;          // Top speed (in/s, 0 = 80% of the drive's top speed)
double profile_max_accel = 100;        // Speed up / slow down rate (in/s^2): lower if the wheels slip
double profile_turn_speed = 360;       // turnProfiled top speed (degrees/s)
double profile_turn_accel = 1200;      // turnProfiled acceleration (degrees/s^2)
double path_lookahead = 12;            // Pure pursuit lookahead (inches): bigger = smoother, smaller = tighter
double path_max_lateral_accel = 120;   // Slows down in curves so the robot doesn't slide (in/s^2)

// Driver control feel, used by driverTank() and driverArcade()
double driver_curve = 0;     // 0 = linear, higher = finer control at low stick values (2-6 is typical)
double driver_deadband = 3;  // Stick values below this (percent) count as zero

// ============================================================================
// DO NOT CHANGE ANYTHING BELOW
// ============================================================================

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}
