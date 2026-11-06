#include "vex.h"
#include "motor-control.h"
#include "../include/autonomous.h"
#include "../include/robot-config.h"
#include "../include/lift.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <thread>
#include <string>
#include "pid.h"
#include "utils.h"
#include "pid.h"
#include <ctime>



// Modify autonomous, driver, or pre-auton code below

// =============================================================================
// Driver Control
// =============================================================================

float deadband(float input, float width){
  if (std::fabs(input)<width){
    return(0);
  }
  return(input);
}

volatile int requestedStage = 1;
bool manualMode = false;


void controlNormalized() {

    double forward = deadband(controller_1.Axis3.value(), 10);
    double turn = deadband(controller_1.Axis1.value(), 10);
    double normal = fabs(turn) + fabs(forward);
    double fResult = 0, tResult = 0;
    if (normal != 0) {
      fResult = forward / normal, tResult = turn / normal;
      if (fabs(turn) > fabs(forward)) {
        fResult *= fabs(turn);
        tResult *= fabs(turn);
      } else {
        fResult *= fabs(forward);
        tResult *= fabs(forward);
      }
    }
    left_chassis.spin(fwd, (fResult + tResult) * 0.12, volt);
    right_chassis.spin(fwd, (fResult - tResult) * 0.12, volt);
    wait(10, msec);

}


void liftControl() {
bool wasManual = false;

  while (true) {
    bool r1Pressed = controller_1.ButtonR1.pressing();
    bool r2Pressed = controller_1.ButtonR2.pressing();

if (r1Pressed) {
  lift.spin(fwd, 12, volt);
  wasManual = true;
} else if (r2Pressed) {
    lift.spin(reverse, 12, volt);
    wasManual = true;
} else {
// When manual input stops, sync requestedStage to current physical position
    if (wasManual) {
      double currentDegrees = -liftHeight.position(deg);

  if (currentDegrees >= 39.1) {
    requestedStage = 3;
} else if (currentDegrees >= 16.2) {
    requestedStage = 2;
} else {
    requestedStage = 1;
}
wasManual = false;
}

// Target stage positioning logic
double targetRotation = 0;
  if (requestedStage == 0 || requestedStage == 1) {
    targetRotation = 0;
} else if (requestedStage == 2) {
    targetRotation = -16.2;
} else if (requestedStage == 3) {
    targetRotation = -39.1;
}

double error = targetRotation - (-liftHeight.position(deg));

if (std::fabs(error) <= 1.0) {
  lift.stop(hold);
} else {
    double voltage = std::fmin(std::fabs(error) * 0.5, 12.0);
if (error < 0) {
  lift.spin(fwd, voltage, volt);
} else {
    lift.spin(reverse, voltage, volt);
      }
    }
  }
  wait(20, msec);
}
}




// void liftControl(){
//   while(1){
//     if(controller_1.ButtonR1.pressing()){
//       lift.spin(fwd,12,volt);
//     }else if(controller_1.ButtonR2.pressing()){
//       lift.spin(reverse,12,volt);
//     }else{
//       lift.stop(hold);
//     }
//   }
// }

bool clawOverride = false;


void moveClawTo(const char* position){
  if(!clawOverride){
    if(position=="open"){
      //0
      while(claw.position(deg)>2.5){
        claw.spin(fwd,-12,volt);
      }
      claw.stop(hold);
    }else if(position=="close"){
      while(claw.position(deg)<50){
        claw.spin(fwd,12,volt);
      }
      claw.stop(hold);
    }else if(position=="matchload"){
      if(claw.position(deg)>35){
        claw.spin(fwd,-12,volt);
      }else if(claw.position(deg)<32.5){
        claw.spin(fwd,12,volt);
      }
      claw.stop(hold);
    }
  }
}

void clawControl(){
  bool inClaw = false;
  while(1){
    if(controller_1.ButtonL1.pressing()){
      clawOverride = false;
      clawRollers.spin(fwd,12,volt);
      claw.spin(fwd,12,volt);
      if(clawRollers.power(watt)>4)inClaw=true;

    }else if(controller_1.ButtonL2.pressing()){
      clawOverride = true;
      clawOverride = true;
      inClaw=false;
        claw.spin(fwd,-12,volt);
      claw.setPosition(0,deg);
    }else{
      clawOverride = false;
      clawRollers.stop(coast);
      if(inClaw){
        claw.spin(fwd,12,volt);
      }else{
        claw.stop(hold);
      }
    }
    wait(20, msec);
  }
}


void clawControlPranoy(){
  while(1){
    if(controller_1.ButtonL1.pressing()){
      clawOverride = true;
      claw.spin(fwd, 12, volt);
      clawRollers.spin(fwd, 12, volt);
    }
    else if(controller_1.ButtonL2.pressing()){
      clawRollers.stop(coast);
      claw.stop(coast);
      clawOverride = false;
    }
    wait(20, msec);
  }
}

void toggleControl(){
  bool toggled=false;
  while(1){
    if(controller_1.ButtonA.pressing()&&!toggled){
      toggle.spin(fwd,12,volt);
      toggled=true;
    }
    else{
      toggle.stop();
      toggled=false;
    }
    
    wait(20, msec);
  } 
}

//stage 0 = 0 (placeholder, same as stage 1)
//stage 1 = 0
//stage 2 = -16.2
//stage 3 = -39.1

//
void rotationMeasure() {
  while (true) {
    double rotationDegrees = -(liftHeight.position(deg));
   int stage = 1;
if (rotationDegrees >= 39.1) {
  stage = 3;
} else if (rotationDegrees >= 16.2) {
  stage = 2;
} else if (rotationDegrees >= 0) {
  stage = 1;
}

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Stack stage: %d", stage);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Rotation: %.1f deg", rotationDegrees);
    wait(100, msec);
  }
}


// Up arrow raises the stage by one, down arrow lowers it by one.
const int MIN_STAGE = 0;
const int MAX_STAGE = 3;

void stageSetter() {
  bool upWasPressed = false;
  bool downWasPressed = false;
  bool rightWasPressed = false;
  bool manualBtnWasPressed = false;

  while (true) {
    bool upPressed = controller_1.ButtonUp.pressing();
    bool downPressed = controller_1.ButtonDown.pressing();
    bool rightPressed = controller_1.ButtonRight.pressing();
    bool manualBtnPressed = controller_1.ButtonLeft.pressing();

// Toggle manual mode
if (manualBtnPressed && !manualBtnWasPressed) {
  manualMode = !manualMode;
}
  manualBtnWasPressed = manualBtnPressed;

// Stage controls (updates requestedStage without fighting motors)
if (upPressed && !upWasPressed && requestedStage < MAX_STAGE) {
  requestedStage++;
} else if (downPressed && !downWasPressed && requestedStage > MIN_STAGE) {
  requestedStage--;
} else if (rightPressed && !rightWasPressed) {
  requestedStage = 0;
}

upWasPressed = upPressed;
downWasPressed = downPressed;
rightWasPressed = rightPressed;
wait(20, msec);
}
}



void conD(){
  while(1){
    controller_1.Screen.clearLine(1);
    controller_1.Screen.setCursor(1,1);
    controller_1.Screen.print("%f",horizontal_tracker.position(deg));
    controller_1.Screen.clearLine(2);
    controller_1.Screen.setCursor(2,1);
        controller_1.Screen.print("%f",vertical_tracker.position(deg));
        controller_1.Screen.clearLine(3);
    controller_1.Screen.setCursor(3,1);
        controller_1.Screen.print("%f",getInertialHeading());
    wait(100, msec);
  }
}


// =============================================================================
// RW Stuff
// =============================================================================

void runAutonomous() {
  int auton_selected = 1;
  thread a(brainD);
  switch(auton_selected) {
    case 1:
      tunePid();
      break;
    case 2:
     
      break;  
    case 3:
      
      break;
    case 4: 
      
      break; 
    case 5:
     
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
  }
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;

// --- Lift driver-control edge detection (previous-button states) ---
bool lift_up_prev = false, lift_down_prev = false;
bool lift_low_prev = false, lift_med_prev = false;
bool lift_high_prev = false, lift_top_prev = false;
bool lift_home_prev = false;

void runDriver() {

  horizontal_tracker.setPosition(0,deg);
  vertical_tracker.setPosition(0,deg);
  stopChassis(coast);
  heading_correction = false;
  claw.setPosition(0,deg);
  liftHeight.setPosition(0, deg);

  // wait(3,sec);

  // controller_1.rumble("---");

  thread lift_control_thread(liftControl);
  thread claw_control_thread(clawControl);
  thread controller_display(conD);
  thread rotation_display(rotationMeasure);
  thread stage_setter_thread(stageSetter);
  thread toggle_thread(toggleControl);
  

    resetChassis();
  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }

  while (true) {
      // [-100, 100] for controller stick axis values
      ch1 = controller_1.Axis1.value();
      ch2 = controller_1.Axis2.value();
      ch3 = controller_1.Axis3.value();
      ch4 = controller_1.Axis4.value();

      // true/false for controller button presses
      l1 = controller_1.ButtonL1.pressing();
      l2 = controller_1.ButtonL2.pressing();
      r1 = controller_1.ButtonR1.pressing();
      r2 = controller_1.ButtonR2.pressing();
      button_a = controller_1.ButtonA.pressing();
      button_b = controller_1.ButtonB.pressing();
      button_x = controller_1.ButtonX.pressing();
      button_y = controller_1.ButtonY.pressing();
      button_up_arrow = controller_1.ButtonUp.pressing();
      button_down_arrow = controller_1.ButtonDown.pressing();
      button_left_arrow = controller_1.ButtonLeft.pressing();
      button_right_arrow = controller_1.ButtonRight.pressing();

      
      controlNormalized();

      wait(10, msec);
    }
}

void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);
  
  // odom tracking
  resetChassis();
  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }



}