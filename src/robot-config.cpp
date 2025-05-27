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
motor left_chassis1 = motor(PORT1, ratio6_1, true);
motor left_chassis2 = motor(PORT2, ratio6_1, true);
motor left_chassis3 = motor(PORT3, ratio6_1, true);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2, left_chassis3);
motor right_chassis1 = motor(PORT4, ratio6_1, false);
motor right_chassis2 = motor(PORT5, ratio6_1, false);
motor right_chassis3 = motor(PORT6, ratio6_1, false);
motor_group right_chassis = motor_group(right_chassis1, right_chassis2, right_chassis3);
inertial inertial_sensor = inertial(PORT7);
//optical example_optical_sensor = optical(PORT8);
//distance example_distance_sensor = distance(PORT9);
//digital_out example_piston = digital_out(Brain.ThreeWirePort.A);

// Format is rotation(port, reversed)
rotation sideways_tracker = rotation(PORT10, true);
rotation forward_tracker = rotation(PORT11, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}