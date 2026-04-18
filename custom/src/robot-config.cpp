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
motor left_chassis1 = motor(PORT13, ratio18_1, true);
motor left_chassis2 = motor(PORT15, ratio18_1, true);
motor left_chassis3 = motor(PORT18, ratio18_1, true);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2, left_chassis3);
motor right_chassis1 = motor(PORT10, ratio18_1, false);
motor right_chassis2 = motor(PORT19, ratio18_1, false);
motor right_chassis3 = motor(PORT12, ratio18_1, false);
motor_group right_chassis = motor_group(right_chassis1, right_chassis2, right_chassis3);

motor end_roll = motor(PORT21, ratio18_1, false);
motor flaps = motor(PORT8, ratio18_1, false);
motor first_intake = motor(PORT4, ratio18_1, false);


inertial inertial_sensor = inertial(PORT11);
optical optical_sensor = optical(PORT17);
digital_out hood = digital_out(Brain.ThreeWirePort.A);
digital_out matchloader = digital_out(Brain.ThreeWirePort.B);
digital_out descorer = digital_out(Brain.ThreeWirePort.C);
digital_out middle = digital_out(Brain.ThreeWirePort.E);

// Format is rotation(port, reversed)
// just set these to random ports if you don't use tracking wheels
rotation horizontal_tracker = rotation(PORT5, false);
rotation vertical_tracker = rotation(PORT16, false);

// Distance reset sensors
// Set these to random ports if you are not using distance resets

//distance front_sensor = distance(PORT12);
distance left_sensor = distance(PORT9);
distance right_sensor = distance(PORT6);
distance back_sensor = distance(PORT3);

// game specific devices for high stakes
/*
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
*/


// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================

// Distance between the middles of the left and right wheels of the drive (in inches)
double distance_between_wheels = 11.25;

// motor to wheel gear ratio * wheel diameter (in inches) * pi
double wheel_distance_in = (36.0 / 48.0) * 3.25 * M_PI;

// PID Constants for movement
// distance_* : Linear PID for straight driving
// turn_*     : PID for turning in place
// heading_correction_* : PID for heading correction during linear movement
double distance_kp = 1, distance_ki = 0.0, distance_kd = 7.9;
double turn_kp = 1.0, turn_ki = 0, turn_kd = 6.3;
double heading_correction_kp = 0.1, heading_correction_ki = 0.0, heading_correction_kd = 0.4;

// Enable or disable the use of tracking wheels
bool using_horizontal_tracker = true;  // Set to true if a horizontal tracking wheel is installed and used for odometry
bool using_vertical_tracker = true;   // Set to true if a vertical tracking wheel is installed and used for odometry

// IGNORE THESE IF YOU ARE NOT USING TRACKING WHEELS
// These comments are in the perspective of a top down view of the robot when the robot is facing vertical
// Vertical distance from the center of the bot to the horizontal tracking wheel (in inches, positive is when the wheel is behind the center)
double horizontal_tracker_dist_from_center = 1.5;
// Horizontal distance from the center of the bot to the vertical tracking wheel (in inches, positive is when the wheel is to the right of the center)
double vertical_tracker_dist_from_center = 5.75;
double horizontal_tracker_diameter = 1.975; // Diameter of the horizontal tracker wheel (in inches)
double vertical_tracker_diameter = 1.975; // Diameter of the vertical tracker wheel (in inches)

// Distance Reset setup
// Set all of these values to the distance from the respective distance sensor to the robot's center along the axis it faces(in inches)
// The front sensor offset is the distance from the front distance sensor to the robot center along the Y axis
// The back sensor offset is the distance from the back distance sensor to the robot center along the Y axis
// The left sensor offset is the distance from the left distance sensor to the robot center along the X axis
// The right sensor offset is the distance from the right distance sensor to the robot center along the X axis
// All values should be positive numbers
// If you are not using all four distance sensors, just set the unused ones to 0
// If you are not using distance resets these values will be ignored
double front_sensor_offset = 0.0;
double left_sensor_offset = 0.0;
double right_sensor_offset = 0.0;
double back_sensor_offset = 0.0;

// ============================================================================
// ADVANCED TUNING (OPTIONAL)
// ============================================================================

bool heading_correction = true; // Use heading correction when the bot is stationary

// Set to true for more accuracy and smoothness, false for more speed
bool dir_change_start = true;   // Less accel/decel due to expecting direction change at start of movement
bool dir_change_end = true;     // Less accel/decel due to expecting direction change at end of movement

double min_output = 10; // Minimum output voltage to motors while chaining movements

// Maximum allowed change in voltage output per 10 msec during movement
double max_slew_accel_fwd = 80;
double max_slew_decel_fwd = 80;
double max_slew_accel_rev = 80;
double max_slew_decel_rev = 80;

// Prevents too much slipping during boomerang movements
// Decrease if there is too much drifting and inconsistency during boomerang
// Increase for more speed during boomerang
double chase_power = 2;

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
void vexcodeInit() {
  // nothing to initialize
}