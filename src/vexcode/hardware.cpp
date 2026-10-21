#include "vex.h"
#include <stdarg.h>
#include <stdio.h>
#include "hardware.h"
#include "motor-control.h"
#include "rw-logo.h"

// ============================================================================
// VEXCODE HARDWARE LAYER
// RW's calls on the devices from custom/src/robot-config.cpp (see include/hardware.h).
// The PROS version is pros/rw/hardware.cpp.
// ============================================================================

extern vex::competition Competition; // Defined in src/vexcode/main.cpp

namespace {

vex::brakeType toBrakeType(hw::Stop mode) {
  if (mode == hw::Stop::coast) return vex::coast;
  if (mode == hw::Stop::brake) return vex::brake;
  return vex::hold;
}

vex::distance& distanceSensor(hw::DistanceSensor sensor) {
  switch (sensor) {
    case hw::front_distance: return front_sensor;
    case hw::right_distance: return right_sensor;
    case hw::back_distance: return back_sensor;
    default: return left_sensor;
  }
}

// A holonomic drive's corner motors (left front, right front, left back, right back) and an
// H-drive's strafe motor, from useHolonomicDrive() and useHDrive()
vex::motor* holonomic_corners[4] = {nullptr, nullptr, nullptr, nullptr};
vex::motor* h_drive_motor = nullptr;
namespace corner { enum { left_front, right_front, left_back, right_back }; }

void spinVolts(vex::motor* motor, double volts) { motor->spin(vex::fwd, volts, vex::voltageUnits::volt); }

}  // namespace

namespace hw {

// --- Drivetrain ---
void driveVolts(double left_volts, double right_volts) {
  if (holonomicDrive()) {
    cornerVolts(left_volts, right_volts, left_volts, right_volts);
    return;
  }
  left_chassis.spin(vex::fwd, left_volts, vex::voltageUnits::volt);
  right_chassis.spin(vex::fwd, right_volts, vex::voltageUnits::volt);
}

void leftVolts(double volts) {
  if (holonomicDrive()) {
    spinVolts(holonomic_corners[corner::left_front], volts);
    spinVolts(holonomic_corners[corner::left_back], volts);
    return;
  }
  left_chassis.spin(vex::fwd, volts, vex::voltageUnits::volt);
}

void rightVolts(double volts) {
  if (holonomicDrive()) {
    spinVolts(holonomic_corners[corner::right_front], volts);
    spinVolts(holonomic_corners[corner::right_back], volts);
    return;
  }
  right_chassis.spin(vex::fwd, volts, vex::voltageUnits::volt);
}

void stopLeft(Stop mode) {
  if (holonomicDrive()) {
    holonomic_corners[corner::left_front]->stop(toBrakeType(mode));
    holonomic_corners[corner::left_back]->stop(toBrakeType(mode));
    return;
  }
  left_chassis.stop(toBrakeType(mode));
}

void stopRight(Stop mode) {
  if (holonomicDrive()) {
    holonomic_corners[corner::right_front]->stop(toBrakeType(mode));
    holonomic_corners[corner::right_back]->stop(toBrakeType(mode));
    return;
  }
  right_chassis.stop(toBrakeType(mode));
}

void stopDrive(Stop mode) {
  stopLeft(mode);
  stopRight(mode);
  if (h_drive_motor) h_drive_motor->stop(toBrakeType(mode));
}

double leftDegrees() {
  if (holonomicDrive()) return (holonomic_corners[corner::left_front]->position(vex::degrees) + holonomic_corners[corner::left_back]->position(vex::degrees)) / 2;
  return left_chassis.position(vex::degrees);
}

double rightDegrees() {
  if (holonomicDrive()) return (holonomic_corners[corner::right_front]->position(vex::degrees) + holonomic_corners[corner::right_back]->position(vex::degrees)) / 2;
  return right_chassis.position(vex::degrees);
}

double leftRpm() {
  if (holonomicDrive()) return (holonomic_corners[corner::left_front]->velocity(vex::rpm) + holonomic_corners[corner::left_back]->velocity(vex::rpm)) / 2;
  return left_chassis.velocity(vex::rpm);
}

double rightRpm() {
  if (holonomicDrive()) return (holonomic_corners[corner::right_front]->velocity(vex::rpm) + holonomic_corners[corner::right_back]->velocity(vex::rpm)) / 2;
  return right_chassis.velocity(vex::rpm);
}

void resetDriveDegrees() {
  if (holonomicDrive()) {
    for (vex::motor* motor : holonomic_corners) motor->setPosition(0, vex::degrees);
  } else {
    left_chassis.setPosition(0, vex::degrees);
    right_chassis.setPosition(0, vex::degrees);
  }
  if (h_drive_motor) h_drive_motor->setPosition(0, vex::degrees);
}

// --- Holonomic drives and H-drives ---
bool holonomicDrive() { return holonomic_corners[corner::left_front] != nullptr; }
bool hDrive() { return h_drive_motor != nullptr; }

void cornerVolts(double left_front_volts, double right_front_volts, double left_back_volts, double right_back_volts) {
  if (!holonomicDrive()) {
    driveVolts((left_front_volts + left_back_volts) / 2, (right_front_volts + right_back_volts) / 2);
    return;
  }
  spinVolts(holonomic_corners[corner::left_front], left_front_volts);
  spinVolts(holonomic_corners[corner::right_front], right_front_volts);
  spinVolts(holonomic_corners[corner::left_back], left_back_volts);
  spinVolts(holonomic_corners[corner::right_back], right_back_volts);
}

void cornerDegrees(double& left_front_deg, double& right_front_deg, double& left_back_deg, double& right_back_deg) {
  if (!holonomicDrive()) {
    left_front_deg = left_back_deg = leftDegrees();
    right_front_deg = right_back_deg = rightDegrees();
    return;
  }
  left_front_deg = holonomic_corners[corner::left_front]->position(vex::degrees);
  right_front_deg = holonomic_corners[corner::right_front]->position(vex::degrees);
  left_back_deg = holonomic_corners[corner::left_back]->position(vex::degrees);
  right_back_deg = holonomic_corners[corner::right_back]->position(vex::degrees);
}

void strafeVolts(double volts) {
  if (h_drive_motor) spinVolts(h_drive_motor, volts);
}

double strafeDegrees() { return h_drive_motor ? h_drive_motor->position(vex::degrees) : 0; }

// --- Inertial sensor and tracking wheels ---
double imuDegrees() { return inertial_sensor.rotation(vex::degrees); }
void setImuDegrees(double rotation_deg) { inertial_sensor.setRotation(rotation_deg, vex::degrees); }
double horizontalTrackerDegrees() { return horizontal_tracker.position(vex::degrees); }
double verticalTrackerDegrees() { return vertical_tracker.position(vex::degrees); }

// --- Distance sensors ---
bool distanceInstalled(DistanceSensor sensor) { return distanceSensor(sensor).installed(); }
bool distanceDetected(DistanceSensor sensor) { return distanceSensor(sensor).isObjectDetected(); }
double distanceInches(DistanceSensor sensor) { return distanceSensor(sensor).objectDistance(vex::inches); }
uint32_t distanceTimestamp(DistanceSensor sensor) { return distanceSensor(sensor).timestamp(); }

// --- Time, threads and competition ---
double timeMsec() { return Brain.timer(vex::msec); }
uint64_t timeMicros() { return vex::timer::systemHighResolution(); }
void sleepMsec(double duration) { vex::wait(duration, vex::msec); }
void startThread(void (*function)()) {
  vex::thread started = vex::thread(function);
  (void)started;  // The thread keeps running after this object goes away
}
intptr_t threadId() { return vex::this_thread::get_id(); }
bool robotEnabled() { return Competition.isEnabled(); }

// --- Controller ---
bool controllerLeft() { return controller_1.ButtonLeft.pressing(); }
bool controllerRight() { return controller_1.ButtonRight.pressing(); }

void controllerPrint(const char* text) {
  controller_1.Screen.setCursor(1, 1);
  controller_1.Screen.print("%s", text);
}

// --- Brain screen ---
void screenClear(uint32_t color) { Brain.Screen.clearScreen(color); }

void screenLine(int x1, int y1, int x2, int y2, uint32_t color) {
  Brain.Screen.setPenColor(color);
  Brain.Screen.drawLine(x1, y1, x2, y2);
}

void screenRect(int x, int y, int width, int height, uint32_t fill, uint32_t outline) {
  Brain.Screen.setPenColor(outline);
  Brain.Screen.setFillColor(fill);
  Brain.Screen.drawRectangle(x, y, width, height);
}

void screenText(int x, int y, bool big, uint32_t color, uint32_t background, const char* text) {
  Brain.Screen.setFont(big ? vex::mono20 : vex::mono15);
  Brain.Screen.setPenColor(color);
  if (background == no_background) {
    Brain.Screen.printAt(x, y, false, "%s", text);
  } else {
    Brain.Screen.setFillColor(background);
    Brain.Screen.printAt(x, y, true, "%s", text);
  }
}

void screenRow(int row, const char* format, ...) {
  char text[128];
  va_list args;
  va_start(args, format);
  vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  Brain.Screen.setCursor(row, 1);
  Brain.Screen.print("%s", text);
}

void screenLogo(int x, int y) { Brain.Screen.drawImageFromBuffer((uint8_t*)rw_logo_png, x, y, rw_logo_png_size); }

bool screenTouch(int& x, int& y) {
  x = Brain.Screen.xPosition();
  y = Brain.Screen.yPosition();
  return Brain.Screen.pressing();
}

// --- SD card ---
bool sdSave(const char* name, const char* data, int length) {
  return Brain.SDcard.isInserted() && Brain.SDcard.savefile(name, (uint8_t*)data, length) > 0;
}

int sdLoad(const char* name, char* buffer, int max_length) {
  if (!Brain.SDcard.isInserted() || !Brain.SDcard.exists(name)) {
    return -1;
  }
  return Brain.SDcard.loadfile(name, (uint8_t*)buffer, max_length);
}

}  // namespace hw

// ============================================================================
// RW FUNCTIONS THAT TAKE VEXCODE TYPES (declared in include/vex.h)
// ============================================================================

/*
 * Stops both chassis motors with the specified brake type.
 * - type: Brake mode (coast, brake, or hold).
 */
void stopChassis(vex::brakeType type) {
  if (hw::holonomicDrive()) {
    for (vex::motor* motor : holonomic_corners) motor->stop(type);
  } else {
    left_chassis.stop(type);
    right_chassis.stop(type);
  }
  if (h_drive_motor) h_drive_motor->stop(type);
}

/*
 * useHolonomicDrive
 * Sets up an X-drive or mecanum drive: the four corner motors, each reversed so it drives the robot
 * forward at positive volts. Call it in runPreAutonomous() before startOdometry(). Every RW motion then
 * works (driving like a tank drive), and strafeToPose(), strafe() and driverHolonomic() can strafe.
 */
void useHolonomicDrive(vex::motor& left_front_motor, vex::motor& right_front_motor, vex::motor& left_back_motor, vex::motor& right_back_motor) {
  holonomic_corners[corner::left_front] = &left_front_motor;
  holonomic_corners[corner::right_front] = &right_front_motor;
  holonomic_corners[corner::left_back] = &left_back_motor;
  holonomic_corners[corner::right_back] = &right_back_motor;
}

/*
 * useHDrive
 * Adds an H-drive's strafe wheel to the tank drive in left_chassis and right_chassis. Its motor should
 * move the robot right at positive volts. Call it in runPreAutonomous() before startOdometry().
 */
void useHDrive(vex::motor& motor) {
  h_drive_motor = &motor;
}

// For the simulator (sim/sim-main.cpp): the motors from useHolonomicDrive() and useHDrive(), or nullptr
vex::motor* holonomicMotor(int index) { return index >= 0 && index < 4 ? holonomic_corners[index] : nullptr; }
vex::motor* hDriveMotor() { return h_drive_motor; }

/*
 * resetPositionWithSensor
 * Resets X or Y using one distance sensor (see resetPositionWithReading() in src/motor-control.cpp).
 * Use resetPositionFront() / Back() / Left() / Right() for the four reset sensors.
 */
bool resetPositionWithSensor(vex::distance& sensor, double s_x, double s_y, double sensor_angle_offset, double field_half_size) {
  if (!sensor.isObjectDetected()) {
    return false;
  }
  return resetPositionWithReading(sensor.objectDistance(vex::inches), s_x, s_y, sensor_angle_offset, field_half_size);
}
