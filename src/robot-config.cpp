#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller controller_1 = controller(primary);

// IMPORTANT: Remember to modify the example motors according to the guide. 
// Format is motor(port, gearSetting, reversed)
// gearSetting is one of the following: ratio36_1(red), ratio18_1(green), ratio6_1(blue)
// all chassis motors should be reversed appropriately so that they spin forward when given a positive voltage input
// such as driveChassis(12, 12)
motor left_chassis1 = motor(PORT5, ratio6_1, true);
motor left_chassis2 = motor(PORT6, ratio6_1, true);
motor left_chassis3 = motor(PORT7, ratio6_1, false);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2, left_chassis3);
motor right_chassis1 = motor(PORT3, ratio6_1, false);
motor right_chassis2 = motor(PORT4, ratio6_1, false);
motor right_chassis3 = motor(PORT11, ratio6_1, true);
motor_group right_chassis = motor_group(right_chassis1, right_chassis2, right_chassis3);

inertial inertial_sensor = inertial(PORT9);
//optical example_optical_sensor = optical(PORT8);
//distance example_distance_sensor = distance(PORT9);
//digital_out example_piston = digital_out(Brain.ThreeWirePort.A);

// Format is rotation(port, reversed)
rotation sideways_tracker = rotation(PORT13, true);
rotation forward_tracker = rotation(PORT19, true);

// game specific devices for high stakes
motor arm_motor1 = motor(PORT2, ratio18_1, true);
motor arm_motor2 = motor(PORT1, ratio18_1, false);
motor_group arm_motor = motor_group(arm_motor1, arm_motor2);
motor intake_motor = motor(PORT8, ratio18_1, true);
digital_out claw = digital_out(Brain.ThreeWirePort.F);
digital_out rush_arm = digital_out(Brain.ThreeWirePort.G);
optical optical_sensor = optical(PORT18);
distance intake_distance = distance(PORT20);
distance clamp_distance = distance(PORT12);
digital_out mogo_mech = digital_out(Brain.ThreeWirePort.E);

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