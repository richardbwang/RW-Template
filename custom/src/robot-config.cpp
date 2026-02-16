#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller controller_1 = controller(primary);

// Drive motors
motor left_chassis1 = motor(PORT20, ratio6_1, true);
motor left_chassis2 = motor(PORT19, ratio6_1, true);
motor left_chassis3 = motor(PORT18, ratio6_1, true);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2, left_chassis3);

motor right_chassis1 = motor(PORT11, ratio6_1, false);
motor right_chassis2 = motor(PORT12, ratio6_1, false);
motor right_chassis3 = motor(PORT13, ratio6_1, false);
motor_group right_chassis = motor_group(right_chassis1, right_chassis2, right_chassis3);

// Intake motors
motor intake_motor1 = motor(PORT1, ratio18_1, false);
motor intake_motor2 = motor(PORT10, ratio18_1, true);
motor_group intake_motors = motor_group(intake_motor1, intake_motor2);

// Sensors
inertial inertial_sensor = inertial(PORT21);
rotation horizontal_tracker = rotation(PORT9, true);
rotation vertical_tracker = rotation(PORT8, true);

// Distance reset sensors
distance front_sensor = distance(PORT5);
distance left_sensor = distance(PORT4);
distance right_sensor = distance(PORT3);
distance back_sensor = distance(PORT6);

// Pneumatics (three-wire ports)
digital_out trip_state_1 = digital_out(Brain.ThreeWirePort.A);
digital_out odom_lift = digital_out(Brain.ThreeWirePort.B);
digital_out little_will = digital_out(Brain.ThreeWirePort.C);
digital_out wing = digital_out(Brain.ThreeWirePort.D);
digital_out trip_state_2 = digital_out(Brain.ThreeWirePort.E);
digital_out middle_descore = digital_out(Brain.ThreeWirePort.F);
digital_out fast_score = digital_out(Brain.ThreeWirePort.G);

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
bool using_horizontal_tracker = true; 
bool using_vertical_tracker = true;   

// Odom offsets (inches) from AGENT.md
// These comments are in the perspective of a top down view of the robot when the robot is facing vertical      
// Vertical distance from the center of the bot to the horizontal tracking wheel (in inches, positive is when the wheel is behind the center)
double horizontal_tracker_dist_from_center = 2.5;
// Horizontal distance from the center of the bot to the vertical tracking wheel (in inches, positive is when the wheel is to the right of the center)
double vertical_tracker_dist_from_center = 1.0;

double horizontal_tracker_diameter = 2.0; // Defaulting to 2.0 unless specified
double vertical_tracker_diameter = 2.0;   // Defaulting to 2.0 unless specified

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

bool heading_correction = true; 
bool dir_change_start = true;   
bool dir_change_end = true;     

double min_output = 10; 

double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;

double chase_power = 2;

// ============================================================================
// DO NOT CHANGE ANYTHING BELOW
// ============================================================================

bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  // nothing to initialize
}
