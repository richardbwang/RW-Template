#include "vex.h"
#include <string.h>
#include "motor-control.h"
#include "auton-selector.h"
#include "rw-logo.h"

extern vex::competition Competition; // Defined in src/main.cpp

// ============================================================================
// INTERNAL STATE (DO NOT CHANGE)
// ============================================================================
const int max_autons = 12;
const char* auton_names[max_autons];
void (*auton_routines[max_autons])();
int auton_count = 0;
int selected_auton = 0;
bool selector_running = false;
const char* selection_file = "rw_auton.txt";

// Button grid: 3 columns x 4 rows under the header
const int button_width = 150, button_height = 42;

int buttonX(int index) { return 5 + (index % 3) * 158; }
int buttonY(int index) { return 50 + (index / 3) * 47; }

/*
 * addAuton
 * Adds an autonomous routine to the selector. Call it once per routine in
 * runPreAutonomous(), before startAutonSelector(). Up to 12 routines, shown in order.
 * - name: Name shown on the Brain and controller screens (about 14 characters fit).
 * - routine: The autonomous function to run.
 */
void addAuton(const char* name, void (*routine)()) {
  if (auton_count >= max_autons) {
    printf("addAuton: the selector holds %d autons, so \"%s\" wasn't added\n", max_autons, name);
    return;
  }
  auton_names[auton_count] = name;
  auton_routines[auton_count] = routine;
  auton_count++;
}

/*
 * getSelectedAuton
 * Returns the index of the selected routine (0 = the first one added).
 */
int getSelectedAuton() {
  return selected_auton;
}

/*
 * saveSelection / loadSelection
 * Keep the selected routine's name on the SD card (if one is inserted),
 * so the choice survives a program restart between matches.
 */
void saveSelection() {
  if (!Brain.SDcard.isInserted()) {
    return;
  }
  const char* name = auton_names[selected_auton];
  Brain.SDcard.savefile(selection_file, (uint8_t*)name, strlen(name));
}

void loadSelection() {
  if (!Brain.SDcard.isInserted() || !Brain.SDcard.exists(selection_file)) {
    return;
  }
  char saved_name[64] = {0};
  int length = Brain.SDcard.loadfile(selection_file, (uint8_t*)saved_name, sizeof(saved_name) - 1);
  if (length <= 0) {
    return;
  }
  saved_name[length] = 0;
  for (int i = 0; i < auton_count; i++) {
    if (strcmp(saved_name, auton_names[i]) == 0) {
      selected_auton = i;
    }
  }
}

/*
 * drawSelector
 * Draws the header with the chosen routine, then one button per routine.
 */
void drawSelector() {
  Brain.Screen.clearScreen(black);
  Brain.Screen.drawImageFromBuffer((uint8_t*)rw_logo_png, 442, 2, rw_logo_png_size);
  Brain.Screen.setFont(mono20);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(black);
  if (auton_count == 0) {
    Brain.Screen.printAt(8, 22, "No autons added (see addAuton)");
    return;
  }
  Brain.Screen.printAt(8, 22, true, "Auton: %s", auton_names[selected_auton]);

  Brain.Screen.setFont(mono15);
  for (int i = 0; i < auton_count; i++) {
    Brain.Screen.setFillColor(i == selected_auton ? "#1E8C3A" : "#3C3C3C");
    Brain.Screen.drawRectangle(buttonX(i), buttonY(i), button_width, button_height);
    Brain.Screen.printAt(buttonX(i) + 8, buttonY(i) + 26, false, "%s", auton_names[i]);
  }
}

/*
 * autonSelectorThread
 * Handles Brain screen taps and controller arrows until autonomous starts.
 * The controller arrows only work while the robot is disabled, so they never
 * clash with driver controls.
 */
void autonSelectorThread() {
  bool was_touching = false, was_left = false, was_right = false;
  int shown = -1;
  double last_pose_draw = 0;

  while (selector_running) {
    // Tap a button on the Brain screen
    bool touching = Brain.Screen.pressing();
    if (touching && !was_touching) {
      int x = Brain.Screen.xPosition(), y = Brain.Screen.yPosition();
      for (int i = 0; i < auton_count; i++) {
        if (x >= buttonX(i) && x <= buttonX(i) + button_width && y >= buttonY(i) && y <= buttonY(i) + button_height) {
          selected_auton = i;
        }
      }
    }
    was_touching = touching;

    // Left/right arrows on the controller
    bool left = controller_1.ButtonLeft.pressing(), right = controller_1.ButtonRight.pressing();
    if (!Competition.isEnabled() && auton_count > 0) {
      if (left && !was_left) selected_auton = (selected_auton + auton_count - 1) % auton_count;
      if (right && !was_right) selected_auton = (selected_auton + 1) % auton_count;
    }
    was_left = left;
    was_right = right;

    if (selected_auton != shown) {
      if (shown != -1) {
        saveSelection();
      }
      shown = selected_auton;
      drawSelector();
      if (auton_count > 0) {
        controller_1.Screen.setCursor(1, 1);
        controller_1.Screen.print("%-19s", auton_names[selected_auton]);
      }
    }

    // Live position and heading, handy for checking odometry before a match
    if (Brain.timer(msec) - last_pose_draw > 200) {
      last_pose_draw = Brain.timer(msec);
      Brain.Screen.setFont(mono15);
      Brain.Screen.setPenColor(white);
      Brain.Screen.setFillColor(black);
      Brain.Screen.printAt(8, 42, "x %6.1f  y %6.1f  h %6.1f   ", x_pos, y_pos, getInertialHeading());
      if (mcl_running) {
        Brain.Screen.printAt(276, 42, "MCL %.1fin %.1fms ", mcl_spread, mcl_cycle_msec);
      }
    }

    wait(20, msec);
  }
}

/*
 * startAutonSelector
 * Shows the selector on the Brain screen. Call it at the end of runPreAutonomous(),
 * after adding routines with addAuton().
 */
void startAutonSelector() {
  loadSelection();
  selector_running = true;
  vex::thread selector = vex::thread(autonSelectorThread);
}

/*
 * runSelectedAuton
 * Stops the selector and runs the chosen routine. Call it from runAutonomous().
 */
void runSelectedAuton() {
  selector_running = false;
  Brain.Screen.clearScreen();
  if (auton_count > 0) {
    auton_routines[selected_auton]();
  }
}
