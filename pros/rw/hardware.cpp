#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "pros/error.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "rw/api.h"
#include "hardware.h"

// ============================================================================
// PROS HARDWARE LAYER
// RW's calls on the devices from src/rw-config.cpp (see include/rw/hardware.h).
// The VEXcode version is src/vexcode/hardware.cpp in the RW-Template repository.
// ============================================================================

namespace {

pros::MotorBrake toBrake(hw::Stop mode) {
  if (mode == hw::Stop::coast) return pros::MotorBrake::coast;
  if (mode == hw::Stop::brake) return pros::MotorBrake::brake;
  return pros::MotorBrake::hold;
}

pros::Distance& distanceSensor(hw::DistanceSensor sensor) {
  switch (sensor) {
    case hw::front_distance: return front_sensor;
    case hw::right_distance: return right_sensor;
    case hw::back_distance: return back_sensor;
    default: return left_sensor;
  }
}

void stopMotors(pros::MotorGroup& motors, pros::MotorBrake mode) {
  motors.set_brake_mode_all(mode);
  motors.brake();
}

void stopMotor(pros::Motor* motor, pros::MotorBrake mode) {
  motor->set_brake_mode(mode);
  motor->brake();
}

void moveVolts(pros::Motor* motor, double volts) { motor->move_voltage((int32_t)(volts * 1000)); }

// A holonomic drive's corner motors (left front, right front, left back, right back) and an
// H-drive's strafe motor, from useHolonomicDrive() and useHDrive()
pros::Motor* holonomic_corners[4] = {nullptr, nullptr, nullptr, nullptr};
pros::Motor* h_drive_motor = nullptr;
namespace corner { enum { left_front, right_front, left_back, right_back }; }

// RW reads the drive encoders in degrees, whatever units the motor groups were made with
void useDegrees() {
  static bool done = false;
  if (!done) {
    left_chassis.set_encoder_units_all(pros::MotorEncoderUnits::degrees);
    right_chassis.set_encoder_units_all(pros::MotorEncoderUnits::degrees);
    done = true;
  }
}

// PROS gives PROS_ERR_F (infinity) while a device is unplugged or starting up (like the IMU while it
// calibrates). Keep the last good reading then, so odometry doesn't jump
double last_left = 0, last_right = 0, last_imu = 0, last_horizontal = 0, last_vertical = 0;
double last_corner[4] = {0, 0, 0, 0}, last_strafe = 0;

double goodOr(double reading, double& last) {
  if (isfinite(reading)) last = reading;
  return last;
}

double cornerDegree(int index) { return goodOr(holonomic_corners[index]->get_position(), last_corner[index]); }

double cornerRpm(int index) {
  double rpm = holonomic_corners[index]->get_actual_velocity();
  return isfinite(rpm) ? rpm : 0;
}

// Rotation sensors read in centidegrees, or PROS_ERR
double trackerDegrees(pros::Rotation& sensor, double& last) {
  int32_t centidegrees = sensor.get_position();
  if (centidegrees != PROS_ERR) last = centidegrees / 100.0;
  return last;
}

// The distance sensor reads 9999 mm when it sees nothing, or PROS_ERR
bool seesObject(int32_t mm) { return mm != PROS_ERR && mm > 0 && mm < 9999; }

}  // namespace

namespace hw {

// --- Drivetrain ---
void driveVolts(double left_volts, double right_volts) {
  if (holonomicDrive()) {
    cornerVolts(left_volts, right_volts, left_volts, right_volts);
    return;
  }
  left_chassis.move_voltage((int32_t)(left_volts * 1000));
  right_chassis.move_voltage((int32_t)(right_volts * 1000));
}

void leftVolts(double volts) {
  if (holonomicDrive()) {
    moveVolts(holonomic_corners[corner::left_front], volts);
    moveVolts(holonomic_corners[corner::left_back], volts);
    return;
  }
  left_chassis.move_voltage((int32_t)(volts * 1000));
}

void rightVolts(double volts) {
  if (holonomicDrive()) {
    moveVolts(holonomic_corners[corner::right_front], volts);
    moveVolts(holonomic_corners[corner::right_back], volts);
    return;
  }
  right_chassis.move_voltage((int32_t)(volts * 1000));
}

void stopLeft(Stop mode) {
  if (holonomicDrive()) {
    stopMotor(holonomic_corners[corner::left_front], toBrake(mode));
    stopMotor(holonomic_corners[corner::left_back], toBrake(mode));
    return;
  }
  stopMotors(left_chassis, toBrake(mode));
}

void stopRight(Stop mode) {
  if (holonomicDrive()) {
    stopMotor(holonomic_corners[corner::right_front], toBrake(mode));
    stopMotor(holonomic_corners[corner::right_back], toBrake(mode));
    return;
  }
  stopMotors(right_chassis, toBrake(mode));
}

void stopDrive(Stop mode) {
  stopLeft(mode);
  stopRight(mode);
  if (h_drive_motor) stopMotor(h_drive_motor, toBrake(mode));
}

double leftDegrees() {
  if (holonomicDrive()) return (cornerDegree(corner::left_front) + cornerDegree(corner::left_back)) / 2;
  useDegrees();
  return goodOr(left_chassis.get_position(), last_left);
}

double rightDegrees() {
  if (holonomicDrive()) return (cornerDegree(corner::right_front) + cornerDegree(corner::right_back)) / 2;
  useDegrees();
  return goodOr(right_chassis.get_position(), last_right);
}

double leftRpm() {
  if (holonomicDrive()) return (cornerRpm(corner::left_front) + cornerRpm(corner::left_back)) / 2;
  double rpm = left_chassis.get_actual_velocity();
  return isfinite(rpm) ? rpm : 0;
}

double rightRpm() {
  if (holonomicDrive()) return (cornerRpm(corner::right_front) + cornerRpm(corner::right_back)) / 2;
  double rpm = right_chassis.get_actual_velocity();
  return isfinite(rpm) ? rpm : 0;
}

void resetDriveDegrees() {
  if (holonomicDrive()) {
    for (int i = 0; i < 4; i++) {
      holonomic_corners[i]->tare_position();
      last_corner[i] = 0;
    }
  } else {
    useDegrees();
    left_chassis.tare_position_all();
    right_chassis.tare_position_all();
    last_left = 0;
    last_right = 0;
  }
  if (h_drive_motor) {
    h_drive_motor->tare_position();
    last_strafe = 0;
  }
}

// --- Holonomic drives and H-drives ---
bool holonomicDrive() { return holonomic_corners[corner::left_front] != nullptr; }
bool hDrive() { return h_drive_motor != nullptr; }

void cornerVolts(double left_front_volts, double right_front_volts, double left_back_volts, double right_back_volts) {
  if (!holonomicDrive()) {
    driveVolts((left_front_volts + left_back_volts) / 2, (right_front_volts + right_back_volts) / 2);
    return;
  }
  moveVolts(holonomic_corners[corner::left_front], left_front_volts);
  moveVolts(holonomic_corners[corner::right_front], right_front_volts);
  moveVolts(holonomic_corners[corner::left_back], left_back_volts);
  moveVolts(holonomic_corners[corner::right_back], right_back_volts);
}

void cornerDegrees(double& left_front_deg, double& right_front_deg, double& left_back_deg, double& right_back_deg) {
  if (!holonomicDrive()) {
    left_front_deg = left_back_deg = leftDegrees();
    right_front_deg = right_back_deg = rightDegrees();
    return;
  }
  left_front_deg = cornerDegree(corner::left_front);
  right_front_deg = cornerDegree(corner::right_front);
  left_back_deg = cornerDegree(corner::left_back);
  right_back_deg = cornerDegree(corner::right_back);
}

void strafeVolts(double volts) {
  if (h_drive_motor) moveVolts(h_drive_motor, volts);
}

double strafeDegrees() { return h_drive_motor ? goodOr(h_drive_motor->get_position(), last_strafe) : 0; }

// --- Inertial sensor and tracking wheels ---
double imuDegrees() { return goodOr(inertial_sensor.get_rotation(), last_imu); }

void setImuDegrees(double rotation_deg) {
  inertial_sensor.set_rotation(rotation_deg);
  last_imu = rotation_deg;
}

double horizontalTrackerDegrees() { return trackerDegrees(horizontal_tracker, last_horizontal); }
double verticalTrackerDegrees() { return trackerDegrees(vertical_tracker, last_vertical); }

// --- Distance sensors ---
bool distanceInstalled(DistanceSensor sensor) { return distanceSensor(sensor).is_installed(); }
bool distanceDetected(DistanceSensor sensor) { return seesObject(distanceSensor(sensor).get()); }
double distanceInches(DistanceSensor sensor) { return distanceSensor(sensor).get() / 25.4; }

// PROS doesn't say when a distance reading is new, so this always changes. MCL still skips a reading
// that's exactly the same as the last one, which is what a sensor that hasn't updated gives
uint32_t distanceTimestamp(DistanceSensor sensor) {
  (void)sensor;
  return pros::millis();
}

// --- Time, threads and competition ---
double timeMsec() { return pros::micros() / 1000.0; }
uint64_t timeMicros() { return pros::micros(); }
void sleepMsec(double duration) { pros::delay(duration > 0 ? (uint32_t)duration : 0); }
void startThread(void (*function)()) { pros::Task started(function); }
intptr_t threadId() { return (intptr_t)pros::c::task_get_current(); }
bool robotEnabled() { return !pros::competition::is_disabled(); }

// --- Controller ---
bool controllerLeft() { return controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT); }
bool controllerRight() { return controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT); }
void controllerPrint(const char* text) { controller_1.print(0, 0, "%s", text); }

// --- Brain screen ---
void screenClear(uint32_t color) {
  pros::screen::set_eraser(color);
  pros::screen::erase();
}

void screenLine(int x1, int y1, int x2, int y2, uint32_t color) {
  pros::screen::set_pen(color);
  pros::screen::draw_line(x1, y1, x2, y2);
}

void screenRect(int x, int y, int width, int height, uint32_t fill, uint32_t outline) {
  pros::screen::set_pen(fill);
  pros::screen::fill_rect(x, y, x + width, y + height);
  pros::screen::set_pen(outline);
  pros::screen::draw_rect(x, y, x + width, y + height);
}

// RW gives the bottom of the text, like VEXcode's printAt; PROS wants the top
void screenText(int x, int y, bool big, uint32_t color, uint32_t background, const char* text) {
  int height = big ? 18 : 14, width = big ? 10 : 8;
  int top = y - height;
  if (background != no_background) {
    pros::screen::set_pen(background);
    pros::screen::fill_rect(x, top, x + width * (int)strlen(text), y + 2);
  }
  pros::screen::set_pen(color);
  pros::screen::print(big ? pros::E_TEXT_MEDIUM : pros::E_TEXT_SMALL, x, top, "%s", text);
}

void screenRow(int row, const char* format, ...) {
  char text[128];
  va_list args;
  va_start(args, format);
  vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  pros::screen::set_pen(0xFFFFFF);
  pros::screen::print(pros::E_TEXT_MEDIUM, row - 1, "%s", text);
}

// PROS can't draw the PNG logo, so this draws the letters RW in the logo's colors
void screenLogo(int x, int y) {
  screenRect(x, y, 36, 36, 0x16181D, 0xEF4444);
  screenText(x + 8, y + 25, true, 0xFFFFFF, no_background, "RW");
}

bool screenTouch(int& x, int& y) {
  pros::screen_touch_status_s_t status = pros::screen::touch_status();
  x = status.x;
  y = status.y;
  return status.touch_status == pros::E_TOUCH_PRESSED || status.touch_status == pros::E_TOUCH_HELD;
}

// --- SD card (its files are under /usd/ in PROS) ---
bool sdSave(const char* name, const char* data, int length) {
  if (!pros::usd::is_installed()) {
    return false;
  }
  char path[96];
  snprintf(path, sizeof(path), "/usd/%s", name);
  FILE* file = fopen(path, "wb");
  if (!file) {
    return false;
  }
  size_t written = fwrite(data, 1, length, file);
  fclose(file);
  return written > 0;
}

int sdLoad(const char* name, char* buffer, int max_length) {
  if (!pros::usd::is_installed()) {
    return -1;
  }
  char path[96];
  snprintf(path, sizeof(path), "/usd/%s", name);
  FILE* file = fopen(path, "rb");
  if (!file) {
    return -1;
  }
  int length = (int)fread(buffer, 1, max_length, file);
  fclose(file);
  return length;
}

}  // namespace hw

// ============================================================================
// RW FUNCTIONS THAT TAKE PROS TYPES (declared in include/rw/api.h)
// ============================================================================

/*
 * Stops both chassis motors with the specified brake mode.
 * - type: Brake mode (pros::MotorBrake::coast, brake, or hold).
 */
void stopChassis(pros::MotorBrake type) {
  if (hw::holonomicDrive()) {
    for (pros::Motor* motor : holonomic_corners) stopMotor(motor, type);
  } else {
    stopMotors(left_chassis, type);
    stopMotors(right_chassis, type);
  }
  if (h_drive_motor) stopMotor(h_drive_motor, type);
}

/*
 * useHolonomicDrive
 * Sets up an X-drive or mecanum drive: the four corner motors, each reversed (negative port) so it drives
 * the robot forward at positive volts. Call it in runPreAutonomous() before startOdometry(). Every RW
 * motion then works (driving like a tank drive), and strafeToPose(), strafe() and driverHolonomic() can strafe.
 */
void useHolonomicDrive(pros::Motor& left_front_motor, pros::Motor& right_front_motor, pros::Motor& left_back_motor, pros::Motor& right_back_motor) {
  holonomic_corners[corner::left_front] = &left_front_motor;
  holonomic_corners[corner::right_front] = &right_front_motor;
  holonomic_corners[corner::left_back] = &left_back_motor;
  holonomic_corners[corner::right_back] = &right_back_motor;
  for (pros::Motor* motor : holonomic_corners) motor->set_encoder_units(pros::MotorEncoderUnits::degrees);
}

/*
 * useHDrive
 * Adds an H-drive's strafe wheel to the tank drive in left_chassis and right_chassis. Its motor should
 * move the robot right at positive volts. Call it in runPreAutonomous() before startOdometry().
 */
void useHDrive(pros::Motor& motor) {
  h_drive_motor = &motor;
  h_drive_motor->set_encoder_units(pros::MotorEncoderUnits::degrees);
}

// For the simulator (sim/sim-main.cpp): the motors from useHolonomicDrive() and useHDrive(), or nullptr
pros::Motor* holonomicMotor(int index) { return index >= 0 && index < 4 ? holonomic_corners[index] : nullptr; }
pros::Motor* hDriveMotor() { return h_drive_motor; }

/*
 * resetPositionWithSensor
 * Resets X or Y using one distance sensor (see resetPositionWithReading() in RW's motor-control.cpp).
 * Use resetPositionFront() / Back() / Left() / Right() for the four reset sensors.
 */
bool resetPositionWithSensor(pros::Distance& sensor, double s_x, double s_y, double sensor_angle_offset, double field_half_size) {
  int32_t mm = sensor.get();
  if (!seesObject(mm)) {
    return false;
  }
  return resetPositionWithReading(mm / 25.4, s_x, s_y, sensor_angle_offset, field_half_size);
}
