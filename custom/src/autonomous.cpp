#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <iostream>
#include <thread>
#include <string.h>
#include <thread>


  
#include "../include/autonomous.h"
#include "motor-control.h"
#include "../include/lift.h"



double clamp(double value, double min, double max) {
    if (value < min)
        return min;

    if (value > max)
        return max;

    return value;
}

// void release(){
//   liftTo(1);
//   intake.spin(fwd,12,volt);
//   wait(1,sec);
//   intake.stop();

// }

// void postAutoReset(){
//   wrist.spin(fwd,12,volt);
//   lift.spin(reverse,12,volt);
//   wait(.5,sec);
//   liftHeight.setPosition(0,deg);
//   wristPosition.setPosition(216,deg);
// }

// void printText(const char* text){
//   std::cout << text <<"\n";
// }

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }
//
// NOTE: arm_motor (PORT16/17) is now driven by the CascadeLift controller
// (see custom/src/lift.cpp). Do NOT run armPIDLoop() or any direct arm_motor
// commands at the same time as the lift, or the two controllers will fight.
// To move the lift in autonomous, use the lift API instead, e.g.:
//   lift.setHeight(LiftHeight::HIGH);
//   lift.waitUntilAtTarget();
//   lift.setHeight(LiftHeight::LOW);
//   lift.waitUntilAtTarget();


// =============================================================================
// Robot Control
// =============================================================================


// =============================================================================
// Autos
// =============================================================================
void exampleAuton() {
  // Use this for tuning linear and turn pid
  y_pos=0;
  x_pos=0;
  // driveTo(12, 3000,true,10);
  // wait(1,sec);
  // std::cout << y_pos << "\n";
  // driveTo(24, 3000,true,10);
  // wait(1,sec);
  // std::cout << y_pos << "\n";
  // driveTo(-36, 5000,true,10);
  // wait(1,sec);
  // std::cout << y_pos << "\n";
  // double heading = inertial_sensor.heading();
  // std::cout << "heading=" << heading <<"\n";

    // turnToAngle(90, 2000);
  // wait(500,msec);
  // std::cout << getInertialHeading() << "\n";
  // turnToAngle(135, 2000);
  // wait(500,msec);
  // std::cout << getInertialHeading() << "\n";
  // turnToAngle(150, 2000);
  // wait(500,msec);
  // std::cout << getInertialHeading() << "\n";
  // std::cout << getInertialHeading() << "\n";
  // turnToAngle(0, 2000);
  // wait(500,msec);
  // std::cout << getInertialHeading() << "\n";


  boomerang(-24,24,1,-90,.3,10000);
  wait(500,msec);
  std::cout << getInertialHeading() << "\n";
  boomerang(0,48,-1,-135,.3,10000);
  wait(500,msec);
  std::cout << getInertialHeading() << "\n";


}

void exampleAuton2() {
  moveToPoint(24, 24, 1, 2000, false);
  moveToPoint(48, 48, 1, 2000, true);
  moveToPoint(24, 24, -1, 2000, true);
  moveToPoint(0, 0, 1, 2000, true);
  correct_angle = 0;
  driveTo(24, 2000, false, 8);
  turnToAngle(90, 800, false);
  turnToAngle(180, 800, true);
}

double arm_pid_target = 0, arm_load_target = 60, arm_store_target = 250, arm_score_target = 470;

/*
 * armPID
 * Runs a single PID update for the arm motor to reach the specified target position.
 * - arm_target: Desired arm position (degrees).
//  */
// void armPID(double arm_target) {
//   PID pidarm = PID(0.1, 0, 0.5); // Initialize PID controller for arm
//   pidarm.setTarget(arm_target);   // Set target position
//   pidarm.setIntegralMax(0);  
//   pidarm.setIntegralRange(1);
//   pidarm.setSmallBigErrorTolerance(1, 1);
//   pidarm.setSmallBigErrorDuration(0, 0);
//   pidarm.setDerivativeTolerance(100);
//   pidarm.setArrive(true);
//   arm_motor.spin(fwd, pidarm.update(arm_motor.position(deg)), volt); // Apply PID output to arm motor
// }

/*
 * armPIDLoop
 * Continuously runs the arm PID control in a separate thread, keeping the arm at the target position.
//  */
// void armPIDLoop() {
//   while(true) {
//     armPID(arm_pid_target); // Continuously update arm position
//     wait(10, msec);
//   }
// }

/*
 * rushClamp
 * Waits until the clamp distance sensor detects an object within 85mm, then closes the claw and lowers the rush arm.
 * Used for quickly grabbing a mobile goal at the start of autonomous.
//  */
// void rushClamp() {
//   while(clamp_distance.objectDistance(mm) > 85) { // Wait for object to be close enough
//     wait(10, msec);
//   }
//   claw.set(true);        // Close the claw to grab the goal
//   rush_arm.set(false);   // Lower the rush arm
// }

// /*
//  * intakeThread
//  * Runs the intake until an object is detected by the optical or distance sensor, then stops the intake.
//  * Used for picking up rings or other objects during autonomous.
//  */
// void intakeThread(){
//   optical_sensor.setLight(ledState::on);      // Turn on optical sensor light
//   optical_sensor.setLightPower(100);          // Set light power to max
//   while(!optical_sensor.isNearObject() && intake_distance.objectDistance(mm) > 50){
//     wait(10, msec);                           // Wait until object is detected
//   }
//   intake_motor.stop(hold);                    // Stop intake motor and hold
// }

// /*
//  * redGoalRush
//  * 2024-2025 World Championship runner-up(1698V) autonomous routine.
//  * This routine executes a complex sequence to rush, grab, and score mobile goals and rings.
//  * It uses multiple threads for simultaneous arm, clamp, and intake control.
//  */
// void redGoalRush() {
//   arm_motor.setPosition(arm_load_target, deg);         // Set arm to load position
//   correct_angle = inertial_sensor.rotation();          // Sync correct_angle with inertial sensor
//   arm_pid_target = arm_store_target;                   // Set arm PID target to store position

//   thread al = thread(armPIDLoop);                      // Start arm PID loop in a thread
//   thread rc = thread(rushClamp);                       // Start clamp routine in a thread
//   intake_motor.spin(fwd, 12, volt);                    // Start intake motor at full speed
//   thread it = thread(intakeThread);                    // Start intake sensor thread
//   rush_arm.set(true);                                  // Lower rush arm

//   driveTo(33, 1100, true);                             // Drive forward to first goal
//   moveToPoint(-2, 10, -1, 15000, false);               // Pull the goal back
//   stopChassis(hold);                                   // Stop chassis and hold position

//   rc.interrupt();                                      // Stop clamp thread (goal should be clamped)
//   rush_arm.set(true);                                  // Lower rush arm again (ensure down)
//   claw.set(false);                                     // Open claw to release goal
//   wait(100, msec);                                     // Brief pause

//   correct_angle = normalizeTarget(-20);                // Adjust target heading for next maneuver
//   driveTo(3, 800, true, 8);                            // Drive forward slightly
//   driveTo(-5, 1000, true);                             // Back up

//   rush_arm.set(false);                                 // Raise rush arm
//   wait(200, msec);                                     // Wait for arm to raise

//   turnToAngle(-90, 800, false);                        // Turn to face the goal backwards
//   moveToPoint(0, 26, -1, 2000, false, 6);              // Move backwards into the goal
//   driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
//   mogo_mech.set(true);                                 // Clamp mobile goal
//   wait(100, msec);                                     // Wait for clamp

//   it.interrupt();                                      // Stop intake thread (ring should be collected)
//   intake_motor.spin(fwd, 12, volt);                    // Restart intake

//   moveToPoint(1, 7, 1, 2000, true);                    // Move near corner to drop goal
//   turnToAngle(-90, 350, true);                         // Turn to drop goal
//   mogo_mech.set(false);                                // Release mobile goal clamp
//   driveChassis(-4, 4);                                 // Turn a bit to align with next target
//   wait(300, msec);                                     // Wait for spin

//   intake_motor.spin(fwd, -12, volt);                   // Reverse intake to push disc in front away
//   moveToPoint(-13, -4, 1, 1500, false, 10);            // Move forward to push disc out of the way
//   turnToAngle(180, 800, false);                        // Turn to clamp goal
//   intake_motor.spin(fwd, 0, volt);                     // Stop intake

//   moveToPoint(-31, 26, -1, 2000, false, 6);            // Move backwards into the next goal
//   driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
//   mogo_mech.set(true);                                 // Clamp mobile goal
//   wait(100, msec);                                     // Wait for clamp

//   turnToAngle(145, 300, true);                         // Turn to face corner
//   moveToPoint(-4, -3, 1, 2000, false);                 // Move to corner
//   intake_motor.spin(fwd, 12, volt);                    // Start intake

//   correct_angle = normalizeTarget(135);                // Update heading for next maneuver
//   driveTo(1000, 1500, false, 4);                       // Drive forward infinitely until timeout
//   driveTo(-13, 2000, true, 6);                         // Back up
//   driveTo(10, 2500, true, 3);                          // Drive forward to intake second corner ring

//   wait(200, msec);                                     // Brief wait for intake

//   moveToPoint(-11, 6, -1, 2000, false, 10);            // Move backward out of the corner
//   turnToAngle(45, 400, true);                          // Turn to align for wallstake

//   al.interrupt();                                      // Stop arm PID thread
//   arm_pid_target = arm_score_target - 100;             // Set arm to scoring position
//   thread al2 = thread(armPIDLoop);                     // Start new arm PID thread

//   moveToPoint(12, 34, 1, 1700, true, 8);               // Move forward to final wallstake scoring position

//   al2.interrupt();                                     // Stop arm PID thread
//   arm_motor.spin(fwd, 1, volt);                        // Spin arm forward slightly

//   turnToAngle(40, 200);                                // Final turn for alignment
//   driveChassis(1, 1);                                  // Slow drive forward
// }

//assume local point system
void autonOne(){
//change this to back up at an angle since not enough space
/*
  //beginning
  //swing to first cup
  swingToAngle(150, 1, "right", 5000, true, 10 );
  
  //put preload in cup todo
 */ 
  wait(1,sec);
  //backup to create space
  driveTo(-3,5000, true, 10);
  wait(50,msec);
  //go to first goal
  turnToPoint(12,12, -1, 5000);
  moveToPoint(12,12,-1,5000,true,10);
  
  //score todo
  
  //move away to create space
  driveTo(4,3000,true, 10);
  
  //go to point away from obstacles  
  turnToPoint(0,6, 1, 4000);
  moveToPoint(0,6,1,4000,true,10);
  
  //go to horizontal pin
  turnToPoint(12,24,1, 3000);
  moveToPoint(12,24,1,4000,true,10);
  
  //score todo
  
  //drive back if needed...
  //go to red goal
  turnToPoint(-12,12,-1, 3000);
  moveToPoint(-12,12,-1, 4000, true, 10);
  
  //score todo
  
  //move away to create space
  driveTo(3,3000,true,10);

  //go to vertical pin
  turnToPoint(-12,24, 1, 3000);
  moveToPoint(-12,24,1,3000,true, 10);
  
  //score todo
  //score/stack on red
  turnToPoint(-12,12, -1, 5000);
  moveToPoint(-12,12,-1, 4000, true,10);
  //finished
}
void tunePid(){
  x_pos = 0;
  y_pos = 0;
  boomerang(24,24,1,90,0.3,5000);
  wait(3,sec);
  boomerang(0,0,-1,0,0.3,5000);
 
  
  
}


// PID TUNING TESTS:
// x_pos=0 y_pos=12.0303 error_y=-0.0303364 - test 1
// x_pos=0 y_pos=12.0087 error_y=-0.00873792 - test 2
// x_pos=0 y_pos=11.8683 error_y=0.131652 - test 3


void liftPID_tuner(){
  // liftHeight.setPosition(0,deg);
  std::cout <<"Running... \n";
  liftToAngle(50,12);
  Brain.Screen.clearScreen();
  // wait(5,sec);
  // Brain.Screen.printAt(10,10,"%f",liftHeight.position(deg));
}

void fullLiftTest(){
  std::cout << "Running... \n";
  std::cout << getLiftHeight() << "\n";
  liftTo(true,true,true,5,"alliance");
  //lifting to 50 degrees
  wait(3,sec);
  
  //printing height to terminal

  
  
}

void brainD(){
  std::cout << "go";
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10,20,"(%f,%f, %f)", x_pos, y_pos, getInertialHeading());
    wait(10,msec);
  }
}

// // ============================================================================
// // Single Pin
// // ============================================================================




//    claw.spin(fwd,12,volt);
//   x_pos = 0;
//   y_pos = 0;
//   std::cout << "Running... \n";

//   wristPosition.setPosition(-290,deg);
//   release();
//   moveWristTo(15);
//   wrist.stop(hold);
//   lift.spin(reverse,12,volt);

//   driveTo(-6,400,true,8);
//   wait(.5,sec);
//   driveTo(8,400,false,4);
//   driveTo(-6,400,true,8);
//   wait(.5,sec);
//   driveTo(8,400,true,4);
//   stopChassis(coast);
  
//   moveToPoint(18,-16,-1,1000,true);
//   left_chassis.spin(fwd,-6,volt);
//   right_chassis.spin(fwd,-6,volt);
//   wait(500,msec);
//   stopChassis(coast);
//   claw.spin(fwd,-12,volt);

//   wait(1,sec);
//   liftPlusHeight(4); 

  
//   lift.stop();
//   driveTo(10,500,true);
//   turnToAngle(75,500);

//     postAutoReset();
//     moveWristTo(-85);
//     liftToAngle(5);
  
// }

// void A_1pin(){


//    claw.spin(fwd,12,volt);
//   x_pos = 0;
//   y_pos = 0;
//   std::cout << "Running... \n";
//   wristPosition.setPosition(-290,deg);
//   release();

//   moveWristTo(15);
//   wrist.stop(hold);
//   lift.spin(reverse,12,volt);
//   driveTo(-4,400,false,8);
//   driveTo(6,400,false,8);
//   driveTo(-4,400,false,8);
//   driveTo(6,400,true,8);
//   stopChassis(coast);
  
//   moveToPoint(-18,-16,-1,1000,true);
//   left_chassis.spin(fwd,-6,volt);
//   right_chassis.spin(fwd,-6,volt);
//   wait(500,msec);
//   stopChassis(coast);
//    claw.spin(fwd,-12,volt);
//   wait(1,sec);
//   lift.stop();
//   driveTo(10,500,true);
//   turnToAngle(75,500);
//   postAutoReset();
//   moveWristTo(-85);
//   liftToAngle(5);
  
// }

// // =============================================================================
// // General
// // =============================================================================

// void runIntake(){
//   claw.spin(fwd,12,volt);
//   intake.spin(fwd,12,volt);
// }

// void stopIntake(){
//   claw.stop(coast);
//   intake.stop(coast);

// }

// void score(){
//   claw.spin(reverse,12,volt);
//         lift.spin(fwd,12,volt);

//         wait(250,msec);
        
//         lift.stop(hold);


//         moveWristTo(136/4);

        
//         claw.stop();

// }



// double qual1_lift_step = 0;
// double qual1_wrist_step = 0;

// void qual1(){
//   lift.setStopping(hold);
//   thread lift_control(qual1_lift);
//   thread wrist_control(qual1_wrist);
//   std::cout << "go";
 
//   wristPosition.setPosition(-290,deg);
//   x_pos = 0;
//   y_pos = 0;

//   //toggles
//   release();
//   driveTo(3,200,false,12);
//   driveTo(-6,600,false,12);
//   driveTo(4,600,false,12);
//   driveTo(-5,600,true,12);

//   //cup then goal
  
//   runIntake();
//   qual1_lift_step=1;
//   moveToPoint(0,25.5,1,1500,true,6);

//   wait(250,msec);
//   driveTo(12,500,true,6);
  
//   wait(1.5,sec);



//   qual1_lift_step=2;
//   qual1_wrist_step=1;

//   moveToPoint(24,16,-1,1500,8);
//   lift.spin(reverse,12,volt);
//   wait(250,msec);
//   lift.stop();
//   score();
//   liftPlusHeight(8);

//   //pin then alliance


  
  




//   //moveToPoint(19,33,1,1000);
//   turnToAngle(15,300);
//   qual1_lift_step=3;
//   qual1_wrist_step=2;
//   wait(0.5,sec);
//   driveTo(18,500,true,6);
//   wait(0.5,sec);

  
//       qual1_wrist_step = 3;

//   moveToPoint(0,17,-1,2000,false,10);

//   moveToPoint(-12,15,-1,2000,true,4);
//   wait(.5,sec);
//   claw.spin(reverse,12,volt);


//   wait(.5,sec);

//   qual1_wrist_step = 4;
//   claw.spin(fwd,12,volt);

//   postAutoReset();

//   //1st pc
  
//   // moveToPoint(-3,7,1,1000,false);
//   // turnToAngle(180,500,true);
//   // boomerang(-18,27,-1,-45,.3,2500);

//   // qual1_lift_step = 4;
//   //   qual1_wrist_step = 5;

//   // wait(.5,sec);

//   //alliance

//   //moveToPoint(-22,17,-1,1000);

//   // wait(.5,sec);

//   // driveTo(-5,500,false);
//   // turnToAngle(45,500,false);
//   // moveToPoint(-20,24,-1,750);

//   /*

//   //2nd pc

//   moveToPoint(-30,18,1,1000,false);
//   moveToPoint(-36,14,-1,1000);

//   wait(.5,sec);

//   //alliance 2

//   turnToAngle(-180,750);
//   moveToPoint(-36,36,-1,1000);

//   */
  
//   stopChassis(coast);
// }

// void qual1_lift(){
//   while(qual1_lift_step==0) wait(10,msec);
//     liftTo(0,12,12);
//   while(qual1_lift_step==1) wait(10,msec);
  
//   liftTo(true,true,true,0,"neutral",12,0,5);
//   while(qual1_lift_step==2) wait(10,msec);
//   wait(0.5,sec);
//   liftTo(.5,12,12);
    
//   while(qual1_lift_step==3) wait(10,msec);

//   while(qual1_lift_step==4) wait(10,msec);

//   while(qual1_lift_step==5) wait(10,msec);
// }

// void qual1_wrist(){
//   while(qual1_wrist_step==0) wait(10,msec);
//   wait(250,msec);
//   moveWristTo(15);
//   while(qual1_wrist_step==1) wait(10,msec);

//   moveWristTo(-85);
//   while(qual1_wrist_step==2) wait(10,msec);
//   wait(850,msec);
//   moveWristTo(15);
//   while(qual1_wrist_step==3) wait(10,msec);
//   moveWristTo(60);
//   while(qual1_wrist_step==4) wait(10,msec);
//   moveWristTo(-25);
//   while(qual1_wrist_step==5) wait(10,msec);


// }

// // void skills_Q1_Matchloading(){
// //   //toggle
// //   moveToPoint(0,8,1,3000,true,10);
// //   moveToPoint(0,0,-1,3000,true,10);
// //   boomerang(22,23,1,90,.3,3000);
// //   //add score preload
// //   moveToPoint(18,23,-1,3000,true,10);
  
// //   turnToPoint(22,46,1,3000);   
// //   moveToPoint(22,46,1,3000,true,10);
// //   //add collect lava lamp
// //   turnToPoint(22,23,1,3000);
// //   moveToPoint(22,23,1,3000,true,10);
// //   //add score lava lamp
// //   turnToPoint(22,26,-1,3000);
// //   moveToPoint(22,26,-1,3000,true,10);
// //   turnToPoint(48,23,1,3000);
// //   moveToPoint(48,23,1,3000,true,10);
// //   for(int i=0;i<3;i++){
// //     moveToPoint(60,23,-1,3000,true,10);
// //     turnToPoint(60,0,1,3000);
// //     moveToPoint(60,0,1,3000,true,10);
// //     //add matchload
// //     wait(2,sec);
// //     //funnel
// //     moveToPoint(60,0,-1,3000,true,10);
// //     turnToPoint(22,23,1,3000);
// //     moveToPoint(22,23,1,3000,true,10);
// //     //score funnel
// //   }
// //     moveToPoint(48,23,-1,3000,true,10);      //going backwards after scoring 3x and at the goal to go to other side
// //     turnToPoint(22,46,1,3000); //placeholder for 1st lava lamp
// //     moveToPoint(22,46,1,3000,true,10);
// //     turnToPoint(-22,46,1,3000); 
// //     moveToPoint(-22,46,1,3000,true,10);
// //     //collect petal
// //     turnToPoint(-22,23,1,3000);
// //     moveToPoint(-22,23,1,3000,true,10);
// //     //score petal

// //     //back up collect petal
// //     moveToPoint(-18,30,-1,3000,true,10);
// //     turnToPoint(-46,23,1,3000,true,10);
// //     moveToPoint(-46,23,1,3000,true,10);
// //      //score
// //     turnToPoint(-22,23,1,3000,true,10);
// //     moveToPoint(-22,23,1,3000,true,10); //run thr point
// //     turnToPoint(-60,23,-1,3000,true,10); 
// //     moveToPoint(-60,23,-1,3000,true,10);
// //     turnToPoint(-60,0,1,3000,true,10);
// //     moveToPoint(-60,0,1,3000,true,10);
// //     //matchload
// //     for(int i=0;i<4;i++){
// //     moveToPoint(-60,23,-1,3000,true,10);
// //     turnToPoint(-60,23,1,3000,true,10);
// //     moveToPoint(-60,23,1,3000,true,10); //score matchload
// //     turnToPoint(-60,23,-1,3000,true,10); // run thr point(s)
// //     moveToPoint(-60,23,-1,3000,true,10);
// //     turnToPoint(-60,0,1,3000,true,10);
// //     moveToPoint(-60,0,1,3000,true,10);
// //     //add matchload
// //     wait(2,sec);
// //     }
// //     //park in midfield
// //     moveToPoint(0,67,1,3000,true,10); // 36 could be possible y value for midfield - mason
// // }



// //void bottom

// // //each sqr = about 24
// // //scores two pins in each quadrant for two quadrants
// // void randomAhAuton(){
// // moveToPoint(0,48, 1, 3000,true, 100);
// // //action
// // turnToPoint(-24,24,1, 3000,true, 100);
// // moveToPoint(-24,24,1,3000,100);
// // //pin
// // turnToPoint(-24,48, 1, 3000,true, 100);
// // moveToPoint(-24,48,1, 3000, 200);
// // //score
// // turnToPoint(24,24, 1, 3000, true, 100);
// // moveToPoint(24,24,1, 3000, true, 100);        
// // //action
// // turnToPoint(24,48, 1,3000, true, 100);
// // moveToPoint(24,48, 1,3000,true,100);
// // turnToPoint(36, 45,1 , 3000,true, 100);
// // moveToPoint(36,45,1 ,3000,true,100);
// // turnToPoint(24,24,1, 3000,true,100);
// // moveToPoint(24,24,1, 3000,true,100);
// // //action
// // turnToPoint(22,22,1, 3000,true,100);
// // moveToPoint(22,22,1, 3000,true,100);
// // turnToPoint(48,22,1, 3000,true,100);
// // moveToPoint(28,22,1, 3000,true,100);
// // //action
// // turnToPoint(48,48,1,3000,true, 100);
// // moveToPoint(48,48,1,3000,true, 100);
// // //action


// // //end of autonomous run //
// // }


// void skills(){

//   double universal_speed_cap = 8.0;

//   lift.setStopping(hold);
//   thread lift_control(qual1_lift);
//   thread wrist_control(qual1_wrist);
//   std::cout << "go";
 
//   wristPosition.setPosition(-290,deg);
//   x_pos = 0;
//   y_pos = 0;


//   moveWristTo(15);
//   wrist.stop(hold);
//   lift.spin(reverse,12,volt);
//   driveTo(-3,400,false,8);
//   driveTo(5,400,false,8);
//   driveTo(-4,400,false,8);
//   driveTo(6,400,true,8);
//   stopChassis(coast);
  
//   moveToPoint(-18,-16,-1,1000,true);
//   left_chassis.spin(fwd,-6,volt);
//   right_chassis.spin(fwd,-6,volt);
//   wait(500,msec);
//   stopChassis(coast);
//    claw.spin(fwd,-12,volt);
//   wait(1,sec);
//   lift.stop();
//   driveTo(10,500,true);
//   turnToAngle(30,500);


//   moveToPoint(-26,-33,-1,1000,false,universal_speed_cap);

//   turnToAngle(-20,500,true,universal_speed_cap);

//   moveToPoint(-57,-17,1,1000,true,universal_speed_cap);

//   turnToAngle(0,500,true,universal_speed_cap);

//   driveTo(10,1000,true);

//   intake.spin(fwd,12,volt);
//   claw.spin(fwd,12,volt);

//   wait(3,sec);

//     driveTo(-10,1000,true);

//   moveToPoint(-50,22,-1,1000,false,universal_speed_cap);
//   moveToPoint(-37,22,-1,1000,false,universal_speed_cap);

//   wait(3,sec);

//   moveToPoint(-50,22,1,1000,false,universal_speed_cap);
//   moveToPoint(-57,-17,1,1000,true,universal_speed_cap);

//   turnToAngle(0,500,true,universal_speed_cap);

//   driveTo(10,1000,true);

//   intake.spin(fwd,12,volt);
//   claw.spin(fwd,12,volt);

//   wait(3,sec);
  

//   stopChassis(coast);
// }

// =============================================================================
// WPI
// =============================================================================
//Blue Auto

int step_BA_2pin = 0;

void lc_BA_2pin(){
  while(step_BA_2pin==0) wait(10,msec);
    lift.spin(fwd,12,volt);
    wait(450,msec);
    lift.stop(hold);
  while(step_BA_2pin==1) wait(10,msec);
    lift.spin(reverse,3,volt);
    wait(500,msec);
    lift.stop(hold);
  while(step_BA_2pin==2) wait(10,msec);
    lift.spin(reverse,12,volt);
    wait(500,msec);
    lift.stop();
  while(step_BA_2pin==3) wait(10,msec);
    lift.spin(fwd,12,volt);
    wait(750,msec);
    lift.stop();
}

void BA_2pin(){
  thread a(lc_BA_2pin);
  x_pos = 0;
  y_pos = 0;
  
  toggle.spin(fwd,12,volt);
  wait(350,msec);
  toggle.stop();
  
  step_BA_2pin++;

  moveToPoint(17.5,8,1,2500,true,10);

  step_BA_2pin++;
  
  wait(500,msec);

  claw.spin(reverse,12,volt);

  wait(1,sec);


  
  driveTo(-6,1000);
  step_BA_2pin++;
  claw.spin(fwd,12,volt);
  turnToPoint(28,-24,1,1000);
  driveTo(10,1000,true,3);
  wait(1000,msec);

  step_BA_2pin++;
  driveTo(-12,1000,10);
  moveToPoint(17.5,8,1,2500,true,10);
  left_chassis.spin(fwd,3,volt);
  right_chassis.spin(fwd,3,volt);
  //moveToPoint(16,3,1,2500,true,10);
  stopChassis(coast);


  


  

  
  //turnToPoint(17.5,16.742,1,2500);
  //moveToPoint(17.5,16.742,1,2500,false,10);
  //wait(.5,sec);
  
  //turnToPoint(17.5,16.742,1,2500);
  //moveToPoint(7.719,7.5 ,-1,2500,false,10);
  //wait(.5,sec);
  
  //turnToPoint(18.013,-5.208,1,2500);
  //moveToPoint(18.013,-5.208,1,2500,false,10);
  //wait(.5,sec);

  //turnToPoint(7.719,7.5 ,1,2500);
  //moveToPoint(7.719,7.5 ,-1,2500,false,10);
  //wait(.5,sec);

  //turnToPoint(17.37,16.749,1,2500);
  //moveToPoint(17.37,16.749,-1,2500,false,10);
  
}

/*void BA_3pin(){
  x_pos = 0;
  y_pos = 0;
  claw.spin(fwd,12,volt);

  turnToPoint(17.5,16.742,1,2500);
  moveToPoint(17.5,16.742,1,2500,false,10);
  wait(.5,sec);
  
  turnToPoint(17.5,16.742,1,2500);
  moveToPoint(7.719,7.5 ,-1,2500,false,10);
  wait(.5,sec);
  
  turnToPoint(18.013,-5.208,1,2500);
  moveToPoint(18.013,-5.208,1,2500,false,10);
  wait(.5,sec);

  turnToPoint(7.719,7.5 ,1,2500);
  moveToPoint(7.719,7.5 ,-1,2500,false,10);
  wait(.5,sec);

  turnToPoint(17.37,16.749,1,2500);
  moveToPoint(17.37,16.749,-1,2500,false,10);
  wait(.5,sec);
  //new compared to BA_2pin

  turnToPoint(17.37,16.749,1,2500);
  moveToPoint(-11.819,11.333,-1,2500,false,10);
  wait(.5,sec);

  turnToPoint(-29.561,-5.539,1,2500);
  moveToPoint(-29.561,-5.539,1,2500,false,10);
  wait(.5,sec);

  turnToPoint(-29.561,-5.539,1,2500);
  moveToPoint(-11.819,11.333,-1,2500,false,10);
  wait(.5,sec);

  turnToPoint(-29.387,16.551,1,2500);
  moveToPoint(-29.387,16.551,1,2500,false,10);
  
}

void BAL_3pin(){
  x_pos = 0;
  y_pos = 0;
  claw.spin(fwd,12,volt);

  turnToPoint(-18.194,19.145,1,2500);
  moveToPoint(-18.194,19.145,1,2500,false,10);
  wait(.5,sec);

  turnToPoint(-18.194,19.145,1,2500);
  moveToPoint(-18.194,3.231,-1,2500,false,10);
  wait(.5,sec);

  turnToPoint(-42.383,19.145,1,2500);
  moveToPoint(-42.383,19.145,1,2500,false,10);
  wait(.5,sec);

  turnToPoint(-18.035,18.986,1,2500);
  moveToPoint(-18.035,18.986,1,2500,false,10);
  wait(.5,sec);

  turnToPoint(-18.035,18.986,1,2500);
  moveToPoint(-32.994,19.304,-1,2500,false,10);
  wait(.5,sec);

  turnToPoint(-18.512,43.016,1,2500);
  moveToPoint(-18.512,43.016,1,2500,false,10);
  wait(.5,sec);

  turnToPoint(-18.193,18.986,1,2500);
  moveToPoint(-18.193,18.986,1,2500,false,10); 
  
}
*/





