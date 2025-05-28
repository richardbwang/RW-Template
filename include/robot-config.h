using namespace vex;

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
//extern optical example_optical_sensor;
//extern distance example_distance_sensor;
//extern digital_out example_piston;
extern rotation sideways_tracker;
extern rotation forward_tracker;

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

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );