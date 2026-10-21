#pragma once
#include <stdint.h>

// ============================================================================
// HARDWARE LAYER
// Everything RW asks of the robot goes through these functions, so the rest of RW is the same
// on VEXcode and PROS. src/vexcode/hardware.cpp is the VEXcode version and pros/rw/hardware.cpp
// the PROS one. Both use your devices by name (left_chassis, inertial_sensor, front_sensor, ...).
// ============================================================================
namespace hw {

// --- Drivetrain ---
// Positions and speeds come from the first motor in each motor group
enum class Stop { coast, brake, hold };
void driveVolts(double left_volts, double right_volts);
void leftVolts(double volts);
void rightVolts(double volts);
void stopDrive(Stop mode);
void stopLeft(Stop mode);
void stopRight(Stop mode);
double leftDegrees();
double rightDegrees();
double leftRpm();
double rightRpm();
void resetDriveDegrees();

// --- Holonomic drives (X-drive, mecanum) and H-drives ---
// After useHolonomicDrive(), the drivetrain functions above use the four corner motors (left = both left
// corners, positions and speeds averaged). useHDrive() adds a strafe motor to a tank drive.
bool holonomicDrive();
bool hDrive();
void cornerVolts(double left_front, double right_front, double left_back, double right_back);
void cornerDegrees(double& left_front, double& right_front, double& left_back, double& right_back);
void strafeVolts(double volts);
double strafeDegrees();

// --- Inertial sensor ---
// Rotation in degrees (clockwise positive, keeps counting past 360), without imu_scale
double imuDegrees();
void setImuDegrees(double rotation_deg);

// --- Tracking wheels (rotation sensors), in degrees ---
double horizontalTrackerDegrees();
double verticalTrackerDegrees();

// --- Distance sensors for resets and MCL, in the order MCL goes around the robot ---
enum DistanceSensor { front_distance, right_distance, back_distance, left_distance };
bool distanceInstalled(DistanceSensor sensor);
bool distanceDetected(DistanceSensor sensor);       // false when the sensor sees nothing
double distanceInches(DistanceSensor sensor);
uint32_t distanceTimestamp(DistanceSensor sensor);  // changes when the sensor has a new reading

// --- Time, threads and competition ---
double timeMsec();      // Time since the program started
uint64_t timeMicros();  // The same in microseconds, for timing short things
void sleepMsec(double duration);
void startThread(void (*function)());
intptr_t threadId();    // Tells threads apart
bool robotEnabled();    // false while the robot is disabled

// --- Controller ---
bool controllerLeft();   // Left arrow button
bool controllerRight();  // Right arrow button
void controllerPrint(const char* text);  // Top line of the controller screen

// --- Brain screen (480 x 240) ---
// Colors are 0xRRGGBB. Text y is the bottom of the text, like VEXcode's printAt
const uint32_t no_background = 0xFFFFFFFF;
void screenClear(uint32_t color);
void screenLine(int x1, int y1, int x2, int y2, uint32_t color);
void screenRect(int x, int y, int width, int height, uint32_t fill, uint32_t outline);
void screenText(int x, int y, bool big, uint32_t color, uint32_t background, const char* text);
void screenRow(int row, const char* format, ...);  // printf-style text on a row (1 = top)
void screenLogo(int x, int y);
bool screenTouch(int& x, int& y);

// --- SD card ---
bool sdSave(const char* name, const char* data, int length);
int sdLoad(const char* name, char* buffer, int max_length);  // Bytes read, or -1 without the card or file

}  // namespace hw
