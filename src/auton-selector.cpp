#include "hardware.h"
#include <stdio.h>
#include <string.h>
#include "motor-control.h"
#include "auton-selector.h"

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
const uint32_t white = 0xFFFFFF, black = 0x000000, selected_color = 0x1E8C3A, button_color = 0x3C3C3C;

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
  const char* name = auton_names[selected_auton];
  hw::sdSave(selection_file, name, strlen(name));
}

void loadSelection() {
  char saved_name[64] = {0};
  int length = hw::sdLoad(selection_file, saved_name, sizeof(saved_name) - 1);
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
  hw::screenClear(black);
  hw::screenLogo(442, 2);
  if (auton_count == 0) {
    hw::screenText(8, 22, true, white, black, "No autons added (see addAuton)");
    return;
  }
  char header[64];
  snprintf(header, sizeof(header), "Auton: %s", auton_names[selected_auton]);
  hw::screenText(8, 22, true, white, black, header);

  for (int i = 0; i < auton_count; i++) {
    hw::screenRect(buttonX(i), buttonY(i), button_width, button_height, i == selected_auton ? selected_color : button_color, white);
    hw::screenText(buttonX(i) + 8, buttonY(i) + 26, false, white, hw::no_background, auton_names[i]);
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
    int x = 0, y = 0;
    bool touching = hw::screenTouch(x, y);
    if (touching && !was_touching) {
      for (int i = 0; i < auton_count; i++) {
        if (x >= buttonX(i) && x <= buttonX(i) + button_width && y >= buttonY(i) && y <= buttonY(i) + button_height) {
          selected_auton = i;
        }
      }
    }
    was_touching = touching;

    // Left/right arrows on the controller
    bool left = hw::controllerLeft(), right = hw::controllerRight();
    if (!hw::robotEnabled() && auton_count > 0) {
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
        char line[32];
        snprintf(line, sizeof(line), "%-19s", auton_names[selected_auton]);
        hw::controllerPrint(line);
      }
    }

    // Live position and heading, handy for checking odometry before a match
    if (hw::timeMsec() - last_pose_draw > 200) {
      last_pose_draw = hw::timeMsec();
      char text[64];
      snprintf(text, sizeof(text), "x %6.1f  y %6.1f  h %6.1f   ", x_pos, y_pos, getInertialHeading());
      hw::screenText(8, 42, false, white, black, text);
      if (mcl_running) {
        snprintf(text, sizeof(text), "MCL %.1fin %.1fms ", mcl_spread, mcl_cycle_msec);
        hw::screenText(276, 42, false, white, black, text);
      }
    }

    hw::sleepMsec(20);
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
  hw::startThread(autonSelectorThread);
}

/*
 * runSelectedAuton
 * Stops the selector and runs the chosen routine. Call it from runAutonomous().
 */
void runSelectedAuton() {
  selector_running = false;
  hw::screenClear(black);
  if (auton_count > 0) {
    auton_routines[selected_auton]();
  }
}
