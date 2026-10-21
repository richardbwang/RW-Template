#include "main.h"
#include "rw-config.h"

// PROS runs these. Each one calls RW's code in src/rw-user.cpp, the same way the VEXcode
// version's main.cpp does. In your own PROS project, add the same three calls to your main.cpp.

/**
 * Runs once when the program starts: calibrates the IMU, starts odometry, and shows the auton
 * selector. Autonomous and driver control wait for it to finish.
 */
void initialize() {
  runPreAutonomous();
}

/**
 * Runs while the robot is disabled. The auton selector keeps running in the background.
 */
void disabled() {}

/**
 * Runs after initialize() when the robot is connected to field control or a competition switch.
 */
void competition_initialize() {}

void autonomous() {
  runAutonomous();
}

void opcontrol() {
  runDriver();
}
