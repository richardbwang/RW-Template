#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/robot-config.h"

// Modify autonomous, driver, or pre-auton code below

const char* TEAM_NAME = "98601C JINX";

void drawTeamGUI() {
  Brain.Screen.clearScreen(black);

  Brain.Screen.setPenColor(purple);

  Brain.Screen.drawRectangle(8, 8, 120, 120);
  Brain.Screen.printAt(150, 30, TEAM_NAME);

  Brain.Screen.printAt(130, 60,  "Theta:");
  Brain.Screen.printAt(130, 90,  "X:");
  Brain.Screen.printAt(130, 120, "Y:");
  Brain.Screen.printAt(130, 150, "H Track:");
  Brain.Screen.printAt(130, 180, "V Track:");
  Brain.Screen.printAt(130, 210, "Opt:");
}

void update_Pos(){
  Brain.Screen.drawRectangle(180, 45, 280, 160);

  Brain.Screen.setPenColor(white);

  Brain.Screen.printAt(180, 60,  " %7.2f deg", inertial_sensor.heading());
  Brain.Screen.printAt(180, 90,  " %7.2f", x_pos);
  Brain.Screen.printAt(180, 110, " %7.2f", y_pos);
  Brain.Screen.printAt(180, 150, " %7.2f", (horizontal_tracker.position(degrees) / 360.0) * horizontal_tracker_diameter * M_PI);
  Brain.Screen.printAt(180, 190, " %7.2f", (vertical_tracker.position(degrees) / 360.0) * vertical_tracker_diameter * M_PI);

}

void runAutonomous() {
  int auton_selected = 1;
  switch(auton_selected) {
    case 1:
      exampleAuton();
      break;
    case 2:
      exampleAuton2();
      break;  
    case 3:
      //redGoalRush();
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

void runDriver() {

  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  
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
  drawTeamGUI();
  stopChassis(coast);
  heading_correction = false;
  bool index_toggle = false;
  bool last_l1 = false;
  bool last_l2 = false;
  bool last_up = false;
  bool last_a = false;
  bool last_down = false;
  bool last_left = false;

  int gui_timer = 0;


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
    if (gui_timer >= 5) {   // 5 * 10ms = 50ms
      update_Pos();
      gui_timer = 0;
    }
    gui_timer++;

    if(button_up_arrow && !last_up) {
      hood.set(!hood.value());
    }
    last_up = button_up_arrow;

    if(button_a && !last_a) {
      matchloader.set(!matchloader.value());
    }
    last_a = button_a;

    if(button_down_arrow && !last_down) {
      descorer.set(!descorer.value());
    }
    last_down = button_down_arrow;

    if (l2 && !last_l2) {
      index_toggle = !index_toggle;
    }
    last_l2 = l2;

    if (r1) {
      first_intake.spin(fwd, 120, volt);
      flaps.spin(fwd, 120, volt);
      if (index_toggle) {
        end_roll.spin(fwd, 120, volt);
      } else {
        end_roll.spin(reverse, 120, volt);
      }
    } 
    else if (r2) {
      first_intake.spin(reverse, 120, volt);
      flaps.spin(reverse, 120, volt);
      if (index_toggle) {
        end_roll.spin(reverse, 120, volt);
      } else {
        end_roll.stop(coast);
      }
    } 
    else {
      first_intake.stop(coast);
      flaps.stop(coast);
      end_roll.stop(coast);
    }

    // default tank drive or replace it with your preferred driver code here: 

    double forward = ch3 * 0.3;
    double turn = ch1 * 0.35;   // stronger turn response than forward


    double left = forward + turn;
    double right = forward - turn;

    driveChassis(left, right);
  

    wait(8, msec); 
  }
}

int guiTask() {
  drawTeamGUI();   // draw static labels once

  while (true) {
    update_Pos();  // only update values
    wait(50, msec);
  }
  return 0;
}

void resetOdom() {
  x_pos = 0;
  y_pos = 0;
  left_chassis.setPosition(0, degrees);
  right_chassis.setPosition(0, degrees);
  horizontal_tracker.setPosition(0, degrees);
  vertical_tracker.setPosition(0, degrees);
  inertial_sensor.setRotation(0, degrees);
  correct_angle = 0;
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

  resetOdom();
  
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

  thread gui = thread(guiTask);
}