#include "hardware.h"
#include <stdio.h>
#include <string.h>
#include "motor-control.h"
#include "drive-log.h"

// ============================================================================
// DRIVE CALIBRATION FOR THE SIMULATOR
// Runs a fixed driving pattern while recording the drive's commands and sensors. The simulator's
// fitter (sh sim/fit-profile.sh rw_drive_log.csv) turns the recording into a robot profile so
// the simulator drives like this robot. Optional: the simulator works without it.
// ============================================================================

// One step of the pattern: what to command and for how long
struct CalibrationStep {
  double seconds;
  int mode;               // 0 = volts, 1 = brake, 2 = coast
  double left_start, right_start, left_end, right_end;  // Volts at the start and end (ramps when different)
};

const CalibrationStep calibration_steps[] = {
  {2.0, 0, 0, 0, 6, 6},          // Slow ramp forward: friction and speed per volt
  {1.0, 2, 0, 0, 0, 0},          // Coast to a stop
  {2.0, 0, 0, 0, -6, -6},        // Slow ramp backward
  {1.0, 2, 0, 0, 0, 0},
  {0.45, 0, 12, 12, 12, 12},     // Full-power launch: acceleration and wheel slip
  {0.6, 1, 0, 0, 0, 0},          // Hard brake
  {0.45, 0, -12, -12, -12, -12},
  {0.6, 1, 0, 0, 0, 0},
  {0.9, 0, 8, -8, 8, -8},        // Turn in place: turning scrub
  {0.5, 1, 0, 0, 0, 0},
  {0.9, 0, -8, 8, -8, 8},
  {0.5, 1, 0, 0, 0, 0},
  {1.0, 0, 9, 4, 9, 4},          // Arc: sideways skid
  {0.8, 2, 0, 0, 0, 0},
  {1.0, 0, -9, -4, -9, -4},      // Arc back
  {0.8, 2, 0, 0, 0, 0},
};

/*
 * calibrateDrive
 * Records the drive for the simulator's fitter. Put the robot in open space with about 4 feet
 * clear on every side, then run it (e.g. from the auton selector). Takes about 15 seconds.
 * Saves rw_drive_log.csv to the SD card, or prints it to the terminal if there's no card.
 */
void calibrateDrive() {
  const int max_rows = 2000;
  static char log_text[max_rows * 72 + 512];
  int length = 0;
  length += snprintf(log_text + length, sizeof(log_text) - length,
                     "# RW drive log: wheel_distance_in=%.4f track=%.4f horizontal=%d vertical=%d "
                     "horizontal_offset=%.4f vertical_offset=%.4f horizontal_diameter=%.4f vertical_diameter=%.4f\n"
                     "t,mode,left_volts,right_volts,left_deg,right_deg,imu_deg,horizontal_deg,vertical_deg\n",
                     wheel_distance_in, distance_between_wheels, using_horizontal_tracker ? 1 : 0, using_vertical_tracker ? 1 : 0,
                     horizontal_tracker_dist_from_center, vertical_tracker_dist_from_center,
                     horizontal_tracker_diameter, vertical_tracker_diameter);

  hw::screenClear(0x000000);
  hw::screenRow(1, "Calibrating the drive for the simulator...");
  double start = hw::timeMsec();
  int rows = 0;
  for (const CalibrationStep& step : calibration_steps) {
    double step_start = hw::timeMsec();
    while (hw::timeMsec() - step_start < step.seconds * 1000 && rows < max_rows) {
      double f = (hw::timeMsec() - step_start) / (step.seconds * 1000);
      double left = step.left_start + (step.left_end - step.left_start) * f;
      double right = step.right_start + (step.right_end - step.right_start) * f;
      if (step.mode == 0) driveChassis(left, right);
      else hw::stopDrive(step.mode == 1 ? hw::Stop::brake : hw::Stop::coast);
      length += snprintf(log_text + length, sizeof(log_text) - length, "%.3f,%d,%.3f,%.3f,%.2f,%.2f,%.3f,%.2f,%.2f\n",
                         (hw::timeMsec() - start) / 1000.0, step.mode, step.mode == 0 ? left : 0, step.mode == 0 ? right : 0,
                         getLeftRotationDegree(), getRightRotationDegree(), hw::imuDegrees(),
                         hw::horizontalTrackerDegrees(), hw::verticalTrackerDegrees());
      rows++;
      hw::sleepMsec(10);
    }
  }
  hw::stopDrive(hw::Stop::brake);

  if (hw::sdSave("rw_drive_log.csv", log_text, length)) {
    hw::screenRow(2, "Saved rw_drive_log.csv to the SD card (%d rows)", rows);
  } else {
    hw::screenRow(2, "No SD card: printing the log to the terminal");
    printf("%s", log_text);
  }
}
