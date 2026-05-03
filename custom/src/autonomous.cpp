#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "../custom/include/robot-config.h"
#include "motor-control.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }

void exampleAuton() {
  // Use this for tuning linear and turn pid
  moveToPoint(0, 24, 1, 1000, true, 100);
  //turnToAngle(90, 1000);
  //turnToAngle(135, 2000);
  //turnToAngle(150, 2000);
  //turnToAngle(160, 2000);
  //turnToAngle(165, 2000);
  //turnToAngle(0, 2000);
  //driveTo(-24, 1000, true, 100);
}

void intake(){
  first_intake.spin(fwd, 120, volt);
  flaps.spin(fwd, 120, volt);
}

void outtake(){
  first_intake.spin(reverse, 90, volt);
  flaps.spin(reverse, 120, volt);
  end_roll.spin(reverse, 120, volt);
}


void setPose(double x, double y, double theta) {
  x_pos = x;
  y_pos = y;
  correct_angle = theta;
  inertial_sensor.setRotation(theta, degrees);
}

void Right_Rush(){
  
  hood.set(false);
  matchloader.set(true);
  moveToPoint(0, 33, 1, 800, false, 11.5);
  intake();
  turnToAngle(90, 700);
  moveToPoint(21, 33, 1, 700, false, 8);
  moveToPoint(24, 33.1, 1, 200, false, 12);
  wait(200, msec);
  moveToPoint(-25, 33.1, -1, 700, false, 6);
  hood.set(true);
  moveToPoint(-28, 33.1, -1, 200, false, 4);
  matchloader.set(false);
  wait(1300, msec);

  driveTo(-15, 700, false, 10);
}

void curve(){
  hood.set(false);
  matchloader.set(true);
  moveToPoint(0, 33, 1, 800, false, 11.5);
  intake();
  turnToAngle(90, 700);
  moveToPoint(20.8, 33, 1, 700, false, 8);
  moveToPoint(23, 33, 1, 200, false, 10);
  wait(250, msec);
  moveToPoint(-25, 33.1, -1, 700, false, 6);
  hood.set(true);
  moveToPoint(-28, 33.1, -1, 200, false, 4);
  matchloader.set(false);
  wait(1300, msec);
  setPose(0, 0, 0);
  curveCircle(-170, -6, 600, true, 8);
  driveTo(0.15, 100, false, 8);
  turnToAngle(0, 400);
  driveTo(-18, 600, true, 10);
}

void Right_Side7Wing() {
  //matchloader.set(true);
  hood.set(false);
  matchloader.set(true);
  moveToPoint(0, 33, 1, 800, false, 11.5);
  intake();
  turnToAngle(90, 700);
  moveToPoint(21, 33, 1, 700, false, 8);
  moveToPoint(24, 33.1, 1, 200, false, 12);
  wait(200, msec);
  moveToPoint(-25, 33.1, -1, 700, false, 6);
  hood.set(true);
  moveToPoint(-28, 33.1, -1, 200, false, 4);
  matchloader.set(false);
  wait(1300, msec);
  turnToAngle(180, 700);
  hood.set(false);
  setPose(0, 0, 0);
  driveTo(6, 600, true, 15);
  matchloader.set(true);
  turnToAngle(47, 600);
  matchloader.set(false);
  driveTo(9, 600, true, 9);
  outtake();
  wait(1250, msec);
  driveTo(-12.5, 700, true, 15);
  turnToAngle(90, 700);
  descorer.set(false);
  driveTo(14, 700, true, 15);
  //matchloader.set(true);
  //moveToPoint(-27, 0, 1, 800, true, 100);


}

void Right_SAWP() {
  //matchloader.set(true);
  hood.set(false);
  matchloader.set(true);
  moveToPoint(0, 32.5, 1, 850, false, 11.8);
  intake();
  turnToAngle(90, 700);
  moveToPoint(21.5, 33, 1, 850, false, 11.8);
  wait(400, msec);
  moveToPoint(-28, 31, -1, 700, false, 6.5);
  hood.set(true);
  matchloader.set(false);
  wait(1000, msec);
  turnToAngle(180, 700);
  hood.set(false);
  setPose(0, 0, 0);


  driveTo(6, 600, true, 15);
  hood.set(true);
  matchloader.set(true);
  wait(400, msec);
  matchloader.set(false);
  driveTo(4.2, 700, false, 8);
  wait(250, msec);
  matchloader.set(true);
  wait(400, msec);
  matchloader.set(false);
  turnToAngle(-60, 700);
  driveTo(7.3, 800, false);
  turnToAngle(-90, 700);
  driveTo(-6, 700, false, 5);
  hood.set(true);
  wait(900, msec);
  matchloader.set(true);
  driveTo(15.5, 700, false, 5);
  wait(400, msec);
  driveTo(-5, 700, false, 5);
  turnToAngle(-30, 700);
  middle.set(true);
  driveTo(-18, 700, false, 5);
  wait(400, msec);

  

  //matchloader.set(true);
  //moveToPoint(-27, 0, 1, 800, true, 100);


}

void autonSawp(){
  moveToPoint(0, 48, 1, 1000, true, 100);
  turnToAngle(90, 1000);
  
}
//double arm_pid_target = 0, arm_load_target = 60, arm_store_target = 250, arm_score_target = 470;

/*
 * armPID
 * Runs a single PID update for the arm motor to reach the specified target position.
 * - arm_target: Desired arm position (degrees).
 */

/*
void armPID(double arm_target) {
  PID pidarm = PID(0.1, 0, 0.5); // Initialize PID controller for arm
  pidarm.setTarget(arm_target);   // Set target position
  pidarm.setIntegralMax(0);  
  pidarm.setIntegralRange(1);
  pidarm.setSmallBigErrorTolerance(1, 1);
  pidarm.setSmallBigErrorDuration(0, 0);
  pidarm.setDerivativeTolerance(100);
  pidarm.setArrive(true);
  arm_motor.spin(fwd, pidarm.update(arm_motor.position(deg)), volt); // Apply PID output to arm motor
}
  */

/*
 * armPIDLoop
 * Continuously runs the arm PID control in a separate thread, keeping the arm at the target position.
 */

 /*
void armPIDLoop() {
  while(true) {
    armPID(arm_pid_target); // Continuously update arm position
    wait(10, msec);
  }
}
  */

/*
 * rushClamp
 * Waits until the clamp distance sensor detects an object within 85mm, then closes the claw and lowers the rush arm.
 * Used for quickly grabbing a mobile goal at the start of autonomous.
 */

 /*
void rushClamp() {
  while(clamp_distance.objectDistance(mm) > 85) { // Wait for object to be close enough
    wait(10, msec);
  }
  claw.set(true);        // Close the claw to grab the goal
  rush_arm.set(false);   // Lower the rush arm
}

/*
 * intakeThread
 * Runs the intake until an object is detected by the optical or distance sensor, then stops the intake.
 * Used for picking up rings or other objects during autonomous.
 */
 /*
void intakeThread(){
  optical_sensor.setLight(ledState::on);      // Turn on optical sensor light
  optical_sensor.setLightPower(100);          // Set light power to max
  while(!optical_sensor.isNearObject() && intake_distance.objectDistance(mm) > 50){
    wait(10, msec);                           // Wait until object is detected
  }
  intake_motor.stop(hold);                    // Stop intake motor and hold
}
*/
/*
 * redGoalRush
 * 2024-2025 World Championship runner-up(1698V) autonomous routine.
 * This routine executes a complex sequence to rush, grab, and score mobile goals and rings.
 * It uses multiple threads for simultaneous arm, clamp, and intake control.
 */

 /*
void redGoalRush() {
  arm_motor.setPosition(arm_load_target, deg);         // Set arm to load position
  correct_angle = inertial_sensor.rotation();          // Sync correct_angle with inertial sensor
  arm_pid_target = arm_store_target;                   // Set arm PID target to store position

  thread al = thread(armPIDLoop);                      // Start arm PID loop in a thread
  thread rc = thread(rushClamp);                       // Start clamp routine in a thread
  intake_motor.spin(fwd, 12, volt);                    // Start intake motor at full speed
  thread it = thread(intakeThread);                    // Start intake sensor thread
  rush_arm.set(true);                                  // Lower rush arm

  driveTo(33, 1100, true);                             // Drive forward to first goal
  moveToPoint(-2, 10, -1, 15000, false);               // Pull the goal back
  stopChassis(hold);                                   // Stop chassis and hold position

  rc.interrupt();                                      // Stop clamp thread (goal should be clamped)
  rush_arm.set(true);                                  // Lower rush arm again (ensure down)
  claw.set(false);                                     // Open claw to release goal
  wait(100, msec);                                     // Brief pause

  correct_angle = normalizeTarget(-20);                // Adjust target heading for next maneuver
  driveTo(3, 800, true, 8);                            // Drive forward slightly
  driveTo(-5, 1000, true);                             // Back up

  rush_arm.set(false);                                 // Raise rush arm
  wait(200, msec);                                     // Wait for arm to raise

  turnToAngle(-90, 800, false);                        // Turn to face the goal backwards
  moveToPoint(0, 26, -1, 2000, false, 6);              // Move backwards into the goal
  driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
  mogo_mech.set(true);                                 // Clamp mobile goal
  wait(100, msec);                                     // Wait for clamp

  it.interrupt();                                      // Stop intake thread (ring should be collected)
  intake_motor.spin(fwd, 12, volt);                    // Restart intake

  moveToPoint(1, 7, 1, 2000, true);                    // Move near corner to drop goal
  turnToAngle(-90, 350, true);                         // Turn to drop goal
  mogo_mech.set(false);                                // Release mobile goal clamp
  driveChassis(-4, 4);                                 // Turn a bit to align with next target
  wait(300, msec);                                     // Wait for spin

  intake_motor.spin(fwd, -12, volt);                   // Reverse intake to push disc in front away
  moveToPoint(-13, -4, 1, 1500, false, 10);            // Move forward to push disc out of the way
  turnToAngle(180, 800, false);                        // Turn to clamp goal
  intake_motor.spin(fwd, 0, volt);                     // Stop intake

  moveToPoint(-31, 26, -1, 2000, false, 6);            // Move backwards into the next goal
  driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
  mogo_mech.set(true);                                 // Clamp mobile goal
  wait(100, msec);                                     // Wait for clamp

  turnToAngle(145, 300, true);                         // Turn to face corner
  moveToPoint(-4, -3, 1, 2000, false);                 // Move to corner
  intake_motor.spin(fwd, 12, volt);                    // Start intake

  correct_angle = normalizeTarget(135);                // Update heading for next maneuver
  driveTo(1000, 1500, false, 4);                       // Drive forward infinitely until timeout
  driveTo(-13, 2000, true, 6);                         // Back up
  driveTo(10, 2500, true, 3);                          // Drive forward to intake second corner ring

  wait(200, msec);                                     // Brief wait for intake

  moveToPoint(-11, 6, -1, 2000, false, 10);            // Move backward out of the corner
  turnToAngle(45, 400, true);                          // Turn to align for wallstake

  al.interrupt();                                      // Stop arm PID thread
  arm_pid_target = arm_score_target - 100;             // Set arm to scoring position
  thread al2 = thread(armPIDLoop);                     // Start new arm PID thread

  moveToPoint(12, 34, 1, 1700, true, 8);               // Move forward to final wallstake scoring position

  al2.interrupt();                                     // Stop arm PID thread
  arm_motor.spin(fwd, 1, volt);                        // Spin arm forward slightly

  turnToAngle(40, 200);                                // Final turn for alignment
  driveChassis(1, 1);                                  // Slow drive forward
}

*/