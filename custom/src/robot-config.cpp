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
motor left_chassis1 = motor(PORT15, ratio6_1, true); //back
motor left_chassis2 = motor(PORT5, ratio6_1, false); //front
motor_group left_chassis = motor_group(left_chassis1, left_chassis2);
motor right_chassis1 = motor(PORT16, ratio6_1, false); //back
motor right_chassis2 = motor(PORT6, ratio6_1, true); //front
motor_group right_chassis = motor_group(right_chassis1, right_chassis2);

motor toggle = motor(PORT11,ratio18_1,false);

inertial inertial_sensor = inertial(PORT21);
optical example_optical_sensor = optical(PORT8);
distance example_distance_sensor = distance(PORT19);
digital_out example_piston = digital_out(Brain.ThreeWirePort.A);

// Format is rotation(port, reversed)
// just set these to random ports if you don't use tracking wheels
rotation horizontal_tracker = rotation(PORT7, true);
rotation vertical_tracker = rotation(PORT20, true);
rotation liftHeight = rotation(PORT5);


// Distance reset sensors
// Set these to random ports if you are not using distance resets
distance front_sensor = distance(PORT21);
distance left_sensor = distance(PORT19);
distance right_sensor = distance(PORT19);
distance back_sensor = distance(PORT19);


motor rightLift = motor(PORT3,ratio36_1,false);
motor leftLift = motor(PORT8,ratio36_1,true); //for test purposes, change back to false later-3
motor_group lift = motor_group(rightLift,leftLift);

motor claw = motor(PORT12,ratio18_1,true);
motor clawRollers = motor(PORT13,ratio18_1,false);




// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================

// Distance between the middles of the left and right wheels of the drive (in inches)
double distance_between_wheels = 11.3;

// motor to wheel gear ratio * wheel diameter (in inches) * pi
double wheel_distance_in = (36.0 / 48.0) * 2.75 * M_PI;

// PID Constants for movement
// distance_* : Linear PID for straight driving
// turn_*     : PID for turning in place
// heading_correction_* : PID for heading correction during linear movement
//og double distance_kp = 1.1, distance_ki = 0.1, distance_kd = 7;

double distance_kp = 1.4, distance_ki = .5, distance_kd =10;
double turn_kp = 0.2, turn_ki = 0, turn_kd = .75;
double heading_correction_kp = .72, heading_correction_ki = .2, heading_correction_kd = 2;

// Enable or disable the use of tracking wheels
bool using_horizontal_tracker = true;  // Set to true if a horizontal tracking wheel is installed and used for odometry
bool using_vertical_tracker = true;   // Set to true if a vertical tracking wheel is installed and used for odometry

// IGNORE THESE IF YOU ARE NOT USING TRACKING WHEELS
// These comments are in the perspective of a top down view of the robot when the robot is facing vertical
// Vertical distance from the center of the bot to the horizontal tracking wheel (in inches, positive is when the wheel is behind the center)
double horizontal_tracker_dist_from_center = 0;
// Horizontal distance from the center of the bot to the vertical tracking wheel (in inches, positive is when the wheel is to the right of the center)
double vertical_tracker_dist_from_center = .53;
double horizontal_tracker_diameter = 2.0; // Diameter of the horizontal tracker wheel (in inches)
double vertical_tracker_diameter = 2.0; // Diameter of the vertical tracker wheel (in inches)

// Distance Reset setup
// If you are not using all four distance sensors, just set the unused ones to 0
// If you are not using distance resets these values will be ignored
// Add your sensor offsets here
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

double min_output = 6; // Minimum output voltage to motors while chaining movements

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
// CASCADE LIFT CONFIGURATION (see LIFT_TUNING.md)
// All positions are in MOTOR-DEGREES (encoder degrees at the motor shaft,
// before any external gearing). Re-zero with lift.zeroPosition() or
// lift.home() and re-measure presets from there.
// ============================================================================

// PID gains. Output is in volts, input in degrees. Start with kG, then P,
// then D, then I - see the tuning guide.
double lift_pid_kp = 0.10;   // volts per degree of error
double lift_pid_ki = 0.0;    // start at 0, add slowly
double lift_pid_kd = 0.5;    // volts per (deg/s) of error change

// Gravity feedforward (volts). The flat upward voltage that holds the lift
// at a constant height against gravity. For a cascade lift the load is nearly
// constant across the range, so a single number works well.
double lift_gravity_ff = 1.5;

// Trapezoidal motion-profile limits for automatic (setHeight) moves.
double lift_profile_speed = 300.0;  // cruise speed (deg/s) at the motor
double lift_profile_accel  = 800.0;  // acceleration     (deg/s^2)

// Arrival detection: how close (deg) and how long settled (msec) before a
// move is considered complete (isAtTarget() == true).
double lift_arrival_tolerance = 3.0;  // degrees
double lift_arrival_settle    = 150.0; // msec

// Soft limits (motor-degrees). Set these to your real mechanical travel so
// the controller never commands past the stops. 0 = bottom after homing.
double lift_min_position = 0.0;
double lift_max_position = 1200.0;

// Max output voltage (V).
double lift_max_output = 12.0;

// Named preset heights (motor-degrees). Measure these with the lift zeroed
// at the bottom (0). Replace these placeholder numbers with your real values.
double lift_preset_bottom = 0.0;
double lift_preset_low    = 250.0;
double lift_preset_medium = 550.0;
double lift_preset_high   = 850.0;
double lift_preset_top    = 1150.0;

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
