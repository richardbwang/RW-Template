using namespace vex;

// Format: extern device deviceName;

extern brain Brain;

// VEXcode devices
extern controller controller_1;
extern motor left_chassis1;
extern motor left_chassis2;
extern motor left_chassis3;
extern motor_group left_chassis;
extern motor right_chassis1;
extern motor right_chassis2;
extern motor right_chassis3;
extern motor_group right_chassis;
extern inertial inertial_sensor;
extern optical example_optical_sensor;
extern distance example_distance_sensor;
extern digital_out example_piston;
extern rotation horizontal_tracker;
extern rotation vertical_tracker;

extern motor arm_motor1;
extern motor arm_motor2;
extern motor_group arm_motor;
extern motor intake_motor;
extern digital_out claw;
extern digital_out rush_arm;
extern optical optical_sensor;
extern distance intake_distance;
extern distance clamp_distance;
extern digital_out mogo_mech;

// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
extern double distance_between_wheels;
extern double wheel_distance_in;
extern double distance_kp, distance_ki, distance_kd;
extern double turn_kp, turn_ki, turn_kd;
extern double heading_correction_kp, heading_correction_ki, heading_correction_kd;

extern bool using_horizontal_tracker;
extern bool using_vertical_tracker;
extern double horizontal_tracker_dist_from_center;
extern double vertical_tracker_dist_from_center;
extern double horizontal_tracker_diameter;
extern double vertical_tracker_diameter;

extern bool heading_correction;
extern bool dir_change_start;
extern bool dir_change_end;
extern double min_output;
extern double max_slew_accel_fwd;
extern double max_slew_decel_fwd;
extern double max_slew_accel_rev;
extern double max_slew_decel_rev;
extern double chase_power;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);