// Simulator stand-in for the PROS 4 API (only used by the computer simulator in sim/). It covers what
// RW-Template and typical robot code use, with the simulated robot in sim/sim-engine.cpp behind it.
// sim/run-sim.sh points every PROS header (api.h, pros/motors.hpp, ...) here. If your code uses
// something missing, the simulator's build says so; add it below the same way.
#pragma once
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "../sim-hooks.h"
// PROS's error values and color names (sim/run-sim.sh copies these from your project's include/pros)
#include "pros/error.h"
#include "pros/colors.h"
#include "pros/colors.hpp"

#ifndef HIGH
#define HIGH 1
#define LOW 0
#endif

namespace rwsim {
// PROS ports are 1-21 (negative = reversed); the simulator numbers them from 0, like VEXcode's PORT1.
// Like the VEXcode stand-in, the simulator takes every motor as already turning the right way
inline int32_t prosPort(int32_t port) { return std::abs(port) - 1; }
}  // namespace rwsim

namespace pros {

// ============================================================================
// TIME AND TASKS
// ============================================================================
namespace c {
typedef void* task_t;
inline uint32_t millis() { return (uint32_t)rwsim::nowMs(); }
inline uint64_t micros() { return (uint64_t)(rwsim::nowMs() * 1000.0); }
inline void delay(uint32_t milliseconds) { rwsim::sleepMs(milliseconds); }
inline void task_delay(uint32_t milliseconds) { rwsim::sleepMs(milliseconds); }
inline task_t task_get_current() { return (task_t)(intptr_t)(rwsim::currentThreadId() + 1); }
}  // namespace c
using c::delay;
using c::micros;
using c::millis;

typedef void (*task_fn_t)(void*);
const uint32_t TASK_PRIORITY_MAX = 16, TASK_PRIORITY_DEFAULT = 8, TASK_PRIORITY_MIN = 1;
const uint16_t TASK_STACK_DEPTH_DEFAULT = 0x2000, TASK_STACK_DEPTH_MIN = 0x200;

class Task {
  int32_t id = -1;
  struct Existing {};
  Task(int32_t task_id, Existing) : id(task_id) {}

 public:
  Task(task_fn_t function, void* parameters = nullptr, uint32_t prio = TASK_PRIORITY_DEFAULT,
       uint16_t stack_depth = TASK_STACK_DEPTH_DEFAULT, const char* name = "") {
    (void)prio; (void)stack_depth; (void)name;
    id = rwsim::spawnThread([function, parameters] { function(parameters); });
  }
  Task(task_fn_t function, void* parameters, const char* name)
      : Task(function, parameters, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, name) {}
  template <class F, class = typename std::enable_if<!std::is_same<typename std::decay<F>::type, Task>::value>::type>
  explicit Task(F&& function, uint32_t prio = TASK_PRIORITY_DEFAULT, uint16_t stack_depth = TASK_STACK_DEPTH_DEFAULT, const char* name = "") {
    (void)prio; (void)stack_depth; (void)name;
    id = rwsim::spawnThread(std::function<void()>(std::forward<F>(function)));
  }
  template <class F, class = typename std::enable_if<!std::is_same<typename std::decay<F>::type, Task>::value>::type>
  Task(F&& function, const char* name) : Task(std::forward<F>(function), TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, name) {}

  static Task current() { return Task(rwsim::currentThreadId(), Existing()); }
  static void delay(uint32_t milliseconds) { rwsim::sleepMs(milliseconds); }
  static void delay_until(uint32_t* prev_time, uint32_t delta) {
    uint32_t target = *prev_time + delta;
    double wait = (double)target - rwsim::nowMs();
    rwsim::sleepMs(wait > 0 ? wait : 0);
    *prev_time = target;
  }
  void remove() { rwsim::interruptThread(id); }
  void join() { while (rwsim::threadRunning(id)) rwsim::sleepMs(10); }
  uint32_t notify() { return 1; }
  uint32_t notify_take(bool clear_on_exit, uint32_t timeout) { (void)clear_on_exit; (void)timeout; return 1; }
  const char* get_name() { return "task"; }
  uint32_t get_priority() { return TASK_PRIORITY_DEFAULT; }
  void set_priority(uint32_t prio) { (void)prio; }
  void suspend() {}
  void resume() {}
};

// Only one simulated task runs at a time, so a lock never has to wait
class Mutex {
 public:
  bool take() { return true; }
  bool take(uint32_t timeout) { (void)timeout; return true; }
  bool give() { return true; }
  void lock() {}
  void unlock() {}
  bool try_lock() { return true; }
};

// ============================================================================
// MOTORS
// ============================================================================
inline namespace v5 {
enum class MotorBrake { coast = 0, brake = 1, hold = 2, invalid = INT32_MAX };
enum class MotorEncoderUnits { degrees = 0, deg = 0, rotations = 1, counts = 2, invalid = INT32_MAX };
using MotorUnits = MotorEncoderUnits;
enum class MotorGears {
  ratio_36_to_1 = 0, red = 0, rpm_100 = 0,
  ratio_18_to_1 = 1, green = 1, rpm_200 = 1,
  ratio_6_to_1 = 2, blue = 2, rpm_600 = 2,
  invalid = INT32_MAX
};
using MotorGearset = MotorGears;
using MotorCart = MotorGears;
using MotorCartridge = MotorGears;
using MotorGear = MotorGears;
}  // namespace v5

typedef enum motor_brake_mode_e { E_MOTOR_BRAKE_COAST = 0, E_MOTOR_BRAKE_BRAKE = 1, E_MOTOR_BRAKE_HOLD = 2, E_MOTOR_BRAKE_INVALID = INT32_MAX } motor_brake_mode_e_t;
typedef enum motor_encoder_units_e { E_MOTOR_ENCODER_DEGREES = 0, E_MOTOR_ENCODER_ROTATIONS = 1, E_MOTOR_ENCODER_COUNTS = 2, E_MOTOR_ENCODER_INVALID = INT32_MAX } motor_encoder_units_e_t;
typedef enum motor_gearset_e {
  E_MOTOR_GEARSET_36 = 0, E_MOTOR_GEAR_RED = 0, E_MOTOR_GEAR_100 = 0,
  E_MOTOR_GEARSET_18 = 1, E_MOTOR_GEAR_GREEN = 1, E_MOTOR_GEAR_200 = 1,
  E_MOTOR_GEARSET_06 = 2, E_MOTOR_GEAR_BLUE = 2, E_MOTOR_GEAR_600 = 2,
  E_MOTOR_GEARSET_INVALID = INT32_MAX
} motor_gearset_e_t;

}  // namespace pros

// What pros::Motor and pros::MotorGroup do to one simulator port
namespace rwsim {
namespace pros_motor {
inline double freeRpm(pros::MotorGears gears) {
  if (gears == pros::MotorGears::red) return 100;
  if (gears == pros::MotorGears::green) return 200;
  if (gears == pros::MotorGears::blue) return 600;
  return 0;  // Keeps the cartridge the port already has
}
inline int brakeMode(pros::MotorBrake mode) { return mode == pros::MotorBrake::coast ? 0 : (mode == pros::MotorBrake::brake ? 1 : 2); }
// Encoder counts per degree, for counts units (1800, 900 or 300 per turn for red, green, blue)
inline double countsPerDegree(int32_t port) {
  double rpm = motorFreeRpm(port);
  return (rpm <= 100 ? 1800 : (rpm <= 200 ? 900 : 300)) / 360.0;
}
inline double toUnits(int32_t port, double deg, pros::MotorUnits units) {
  if (units == pros::MotorUnits::rotations) return deg / 360.0;
  if (units == pros::MotorUnits::counts) return deg * countsPerDegree(port);
  return deg;
}
inline double fromUnits(int32_t port, double value, pros::MotorUnits units) {
  if (units == pros::MotorUnits::rotations) return value * 360.0;
  if (units == pros::MotorUnits::counts) return value / countsPerDegree(port);
  return value;
}
// Each command replaces the one before, like on a real motor (see moveTo())
inline std::map<int32_t, int>& commands() {
  static std::map<int32_t, int> counts;
  return counts;
}
inline int newCommand(int32_t port) { return ++commands()[port]; }
inline void voltage(int32_t port, int32_t millivolts) {
  newCommand(port);
  motorVoltage(port, millivolts / 1000.0);
}
inline void velocity(int32_t port, int32_t rpm) {
  newCommand(port);
  double free = motorFreeRpm(port);
  motorPercent(port, free > 0 ? rpm / free * 100.0 : 0);
}
inline void brake(int32_t port) {
  newCommand(port);
  motorStop(port, -1);  // The port's brake mode
}
// move_absolute() and move_relative(): like the motor's own position control, a background task
// drives the motor to the target and holds it there, until another command replaces it
inline void moveTo(int32_t port, double target_deg, int32_t max_rpm) {
  int command = newCommand(port);
  double free = motorFreeRpm(port);
  double limit = free > 0 ? std::fmin(100.0, std::fabs((double)max_rpm) / free * 100.0) : 50;
  spawnThread([port, target_deg, limit, command] {
    while (commands()[port] == command) {
      double error = target_deg - motorPositionDeg(port);
      if (std::fabs(error) < 2) {
        motorStop(port, 2);
      } else {
        motorPercent(port, (error > 0 ? 1 : -1) * std::fmin(limit, std::fabs(error) * 0.5 + 5));
      }
      sleepMs(10);
    }
  });
}
}  // namespace pros_motor
}  // namespace rwsim

namespace pros {
inline namespace v5 {

class Motor {
  int32_t port_;  // The PROS port (negative = reversed)
  int32_t sim_;   // The simulator's port
  MotorUnits units_ = MotorUnits::degrees;
  MotorBrake brake_ = MotorBrake::coast;

 public:
  explicit Motor(int8_t port, MotorGears gearset = MotorGears::invalid, MotorUnits encoder_units = MotorUnits::invalid)
      : port_(port), sim_(rwsim::prosPort(port)) {
    rwsim::motorRegister(sim_, rwsim::pros_motor::freeRpm(gearset));
    if (encoder_units != MotorUnits::invalid) units_ = encoder_units;
  }

  int32_t move(int32_t voltage) { rwsim::pros_motor::voltage(sim_, (int32_t)(voltage / 127.0 * 12000)); return 1; }
  int32_t move_voltage(int32_t voltage) { rwsim::pros_motor::voltage(sim_, voltage); return 1; }
  int32_t move_velocity(int32_t velocity) { rwsim::pros_motor::velocity(sim_, velocity); return 1; }
  int32_t move_absolute(double position, int32_t velocity) {
    rwsim::pros_motor::moveTo(sim_, rwsim::pros_motor::fromUnits(sim_, position, units_), velocity);
    return 1;
  }
  int32_t move_relative(double position, int32_t velocity) {
    rwsim::pros_motor::moveTo(sim_, rwsim::motorPositionDeg(sim_) + rwsim::pros_motor::fromUnits(sim_, position, units_), velocity);
    return 1;
  }
  int32_t brake() { rwsim::pros_motor::brake(sim_); return 1; }
  int32_t set_brake_mode(MotorBrake mode, uint8_t index = 0) {
    (void)index;
    brake_ = mode;
    rwsim::motorSetStopping(sim_, rwsim::pros_motor::brakeMode(mode));
    return 1;
  }
  int32_t set_brake_mode(motor_brake_mode_e_t mode, uint8_t index = 0) { return set_brake_mode((MotorBrake)mode, index); }
  int32_t set_brake_mode_all(MotorBrake mode) { return set_brake_mode(mode); }
  MotorBrake get_brake_mode(uint8_t index = 0) const { (void)index; return brake_; }
  int32_t set_encoder_units(MotorUnits units, uint8_t index = 0) { (void)index; units_ = units; return 1; }
  int32_t set_encoder_units(motor_encoder_units_e_t units, uint8_t index = 0) { return set_encoder_units((MotorUnits)units, index); }
  int32_t set_encoder_units_all(MotorUnits units) { return set_encoder_units(units); }
  MotorUnits get_encoder_units(uint8_t index = 0) const { (void)index; return units_; }
  int32_t set_gearing(MotorGears gearset, uint8_t index = 0) { (void)index; rwsim::motorRegister(sim_, rwsim::pros_motor::freeRpm(gearset)); return 1; }

  double get_position(uint8_t index = 0) const { (void)index; return rwsim::pros_motor::toUnits(sim_, rwsim::motorPositionDeg(sim_), units_); }
  int32_t tare_position(uint8_t index = 0) { (void)index; rwsim::motorSetPositionDeg(sim_, 0); return 1; }
  int32_t tare_position_all() { return tare_position(); }
  int32_t set_zero_position(double position, uint8_t index = 0) {
    (void)index;
    rwsim::motorSetPositionDeg(sim_, rwsim::motorPositionDeg(sim_) - rwsim::pros_motor::fromUnits(sim_, position, units_));
    return 1;
  }
  double get_actual_velocity(uint8_t index = 0) const { (void)index; return rwsim::motorRpm(sim_); }
  double get_temperature(uint8_t index = 0) const { (void)index; return 30; }
  int32_t get_current_draw(uint8_t index = 0) const { (void)index; return 500; }
  int32_t get_voltage(uint8_t index = 0) const { (void)index; return 12000; }
  double get_power(uint8_t index = 0) const { (void)index; return 0; }
  double get_torque(uint8_t index = 0) const { (void)index; return 0; }
  double get_efficiency(uint8_t index = 0) const { (void)index; return 50; }
  int32_t is_over_temp(uint8_t index = 0) const { (void)index; return 0; }
  int32_t is_reversed(uint8_t index = 0) const { (void)index; return port_ < 0; }
  int32_t set_reversed(bool reverse, uint8_t index = 0) { (void)index; port_ = reverse ? -std::abs(port_) : std::abs(port_); return 1; }
  int8_t get_port(uint8_t index = 0) const { (void)index; return (int8_t)port_; }
  bool is_installed() { return true; }
  int32_t index() const { return sim_; }  // The simulator's port number
};

class MotorGroup {
  std::vector<int8_t> ports_;
  std::vector<int32_t> sims_;
  std::vector<MotorUnits> units_;
  std::vector<MotorBrake> brakes_;

  void setUp(MotorGears gearset, MotorUnits encoder_units) {
    for (int8_t port : ports_) {
      sims_.push_back(rwsim::prosPort(port));
      rwsim::motorRegister(sims_.back(), rwsim::pros_motor::freeRpm(gearset));
    }
    units_.assign(ports_.size(), encoder_units == MotorUnits::invalid ? MotorUnits::degrees : encoder_units);
    brakes_.assign(ports_.size(), MotorBrake::coast);
  }
  double fromUnits(size_t i, double value) const { return rwsim::pros_motor::fromUnits(sims_[i], value, units_[i]); }

 public:
  MotorGroup(std::initializer_list<int8_t> ports, MotorGears gearset = MotorGears::invalid, MotorUnits encoder_units = MotorUnits::invalid)
      : ports_(ports) { setUp(gearset, encoder_units); }
  explicit MotorGroup(const std::vector<int8_t>& ports, MotorGears gearset = MotorGears::invalid, MotorUnits encoder_units = MotorUnits::invalid)
      : ports_(ports) { setUp(gearset, encoder_units); }

  int32_t move(int32_t voltage) { for (int32_t p : sims_) rwsim::pros_motor::voltage(p, (int32_t)(voltage / 127.0 * 12000)); return 1; }
  int32_t move_voltage(int32_t voltage) { for (int32_t p : sims_) rwsim::pros_motor::voltage(p, voltage); return 1; }
  int32_t move_velocity(int32_t velocity) { for (int32_t p : sims_) rwsim::pros_motor::velocity(p, velocity); return 1; }
  int32_t move_absolute(double position, int32_t velocity) {
    for (size_t i = 0; i < sims_.size(); i++) rwsim::pros_motor::moveTo(sims_[i], fromUnits(i, position), velocity);
    return 1;
  }
  int32_t move_relative(double position, int32_t velocity) {
    for (size_t i = 0; i < sims_.size(); i++) rwsim::pros_motor::moveTo(sims_[i], rwsim::motorPositionDeg(sims_[i]) + fromUnits(i, position), velocity);
    return 1;
  }
  int32_t brake() { for (int32_t p : sims_) rwsim::pros_motor::brake(p); return 1; }
  int32_t set_brake_mode(MotorBrake mode, uint8_t index = 0) {
    if (index >= sims_.size()) return PROS_ERR;
    brakes_[index] = mode;
    rwsim::motorSetStopping(sims_[index], rwsim::pros_motor::brakeMode(mode));
    return 1;
  }
  int32_t set_brake_mode(motor_brake_mode_e_t mode, uint8_t index = 0) { return set_brake_mode((MotorBrake)mode, index); }
  int32_t set_brake_mode_all(MotorBrake mode) { for (size_t i = 0; i < sims_.size(); i++) set_brake_mode(mode, (uint8_t)i); return 1; }
  int32_t set_brake_mode_all(motor_brake_mode_e_t mode) { return set_brake_mode_all((MotorBrake)mode); }
  MotorBrake get_brake_mode(uint8_t index = 0) const { return index < brakes_.size() ? brakes_[index] : MotorBrake::invalid; }
  int32_t set_encoder_units(MotorUnits units, uint8_t index = 0) { if (index < units_.size()) units_[index] = units; return 1; }
  int32_t set_encoder_units_all(MotorUnits units) { units_.assign(sims_.size(), units); return 1; }
  int32_t set_encoder_units_all(motor_encoder_units_e_t units) { return set_encoder_units_all((MotorUnits)units); }
  int32_t set_gearing_all(MotorGears gearset) { for (int32_t p : sims_) rwsim::motorRegister(p, rwsim::pros_motor::freeRpm(gearset)); return 1; }

  double get_position(uint8_t index = 0) const {
    if (index >= sims_.size()) return PROS_ERR_F;
    return rwsim::pros_motor::toUnits(sims_[index], rwsim::motorPositionDeg(sims_[index]), units_[index]);
  }
  std::vector<double> get_position_all() const {
    std::vector<double> positions;
    for (size_t i = 0; i < sims_.size(); i++) positions.push_back(get_position((uint8_t)i));
    return positions;
  }
  int32_t tare_position(uint8_t index = 0) { if (index < sims_.size()) rwsim::motorSetPositionDeg(sims_[index], 0); return 1; }
  int32_t tare_position_all() { for (int32_t p : sims_) rwsim::motorSetPositionDeg(p, 0); return 1; }
  double get_actual_velocity(uint8_t index = 0) const { return index < sims_.size() ? rwsim::motorRpm(sims_[index]) : PROS_ERR_F; }
  std::vector<double> get_actual_velocity_all() const {
    std::vector<double> velocities;
    for (int32_t p : sims_) velocities.push_back(rwsim::motorRpm(p));
    return velocities;
  }
  double get_temperature(uint8_t index = 0) const { (void)index; return 30; }
  int32_t get_current_draw(uint8_t index = 0) const { (void)index; return 500; }
  int32_t get_voltage(uint8_t index = 0) const { (void)index; return 12000; }
  int32_t is_over_temp(uint8_t index = 0) const { (void)index; return 0; }
  int8_t get_port(uint8_t index = 0) const { return index < ports_.size() ? ports_[index] : 0; }
  std::vector<int8_t> get_port_all() const { return ports_; }
  int8_t size() const { return (int8_t)ports_.size(); }
  Motor operator[](int index) const { return Motor(ports_[index]); }
  const std::vector<int32_t>& simPorts() const { return sims_; }  // The simulator's port numbers
};

// ============================================================================
// SENSORS
// ============================================================================
class Imu {
  int32_t sim_;

 public:
  explicit Imu(uint8_t port) : sim_(rwsim::prosPort(port)) {}
  int32_t reset(bool blocking = false) {
    rwsim::imuCalibrate(sim_);
    while (blocking && rwsim::imuCalibrating(sim_)) rwsim::sleepMs(10);
    return 1;
  }
  bool is_calibrating() { return rwsim::imuCalibrating(sim_); }
  // Like the real IMU, it reads PROS_ERR_F until it's calibrated
  double get_rotation() { return is_calibrating() ? PROS_ERR_F : rwsim::imuRotationDeg(sim_); }
  double get_heading() {
    if (is_calibrating()) return PROS_ERR_F;
    double heading = std::fmod(rwsim::imuRotationDeg(sim_), 360.0);
    return heading < 0 ? heading + 360.0 : heading;
  }
  double get_yaw() { double heading = get_heading(); return heading > 180 ? heading - 360 : heading; }
  double get_pitch() { return 0; }
  double get_roll() { return 0; }
  int32_t set_rotation(double target) { rwsim::imuSetRotationDeg(sim_, target); return 1; }
  int32_t tare_rotation() { return set_rotation(0); }
  int32_t set_heading(double target) {
    double rotation = rwsim::imuRotationDeg(sim_);
    double heading = std::fmod(rotation, 360.0);
    if (heading < 0) heading += 360.0;
    rwsim::imuSetRotationDeg(sim_, rotation - heading + target);
    return 1;
  }
  int32_t tare_heading() { return set_heading(0); }
  int32_t tare() { return tare_rotation(); }
  bool is_installed() { return true; }
  int32_t index() const { return sim_; }  // The simulator's port number
};

class Rotation {
  int32_t port_, sim_;

 public:
  explicit Rotation(int8_t port) : port_(port), sim_(rwsim::prosPort(port)) {}
  int32_t get_position() { return (int32_t)std::lround(rwsim::rotationSensorDeg(sim_) * 100); }  // Centidegrees
  int32_t get_angle() {
    double angle = std::fmod(rwsim::rotationSensorDeg(sim_), 360.0);
    return (int32_t)std::lround((angle < 0 ? angle + 360.0 : angle) * 100);
  }
  int32_t get_velocity() { return (int32_t)std::lround(rwsim::rotationSensorRpm(sim_) * 6.0 * 100); }  // Centidegrees per second
  int32_t set_position(int32_t position) { rwsim::rotationSensorSetDeg(sim_, position / 100.0); return 1; }
  int32_t reset_position() { return set_position(0); }
  int32_t reset() { return reset_position(); }
  int32_t set_reversed(bool value) { (void)value; return 1; }
  int32_t reverse() { return 1; }
  int32_t get_reversed() { return port_ < 0; }
  bool is_installed() { return true; }
  int32_t index() const { return sim_; }  // The simulator's port number
};

class Distance {
  int32_t sim_;

 public:
  explicit Distance(uint8_t port) : sim_(rwsim::prosPort(port)) {}
  int32_t get() {  // mm, 9999 when it sees nothing
    double mm = rwsim::distanceMm(sim_);
    return mm >= 9999 ? 9999 : (int32_t)std::lround(mm);
  }
  int32_t get_distance() { return get(); }
  int32_t get_confidence() { return get() < 9999 ? 63 : 0; }
  int32_t get_object_size() { return get() < 9999 ? 200 : 0; }
  double get_object_velocity() { return 0; }
  bool is_installed() { return true; }
  int32_t index() const { return sim_; }  // The simulator's port number
};

// Sees nothing in the simulator
class Optical {
 public:
  explicit Optical(uint8_t port) { (void)port; }
  double get_hue() { return 0; }
  double get_saturation() { return 0; }
  double get_brightness() { return 0; }
  int32_t get_proximity() { return 0; }
  int32_t set_led_pwm(uint8_t value) { (void)value; return 1; }
  int32_t get_led_pwm() { return 0; }
  int32_t disable_gesture() { return 1; }
  int32_t enable_gesture() { return 1; }
  bool is_installed() { return true; }
};
}  // namespace v5

// The three-wire ports: outputs remember what they were set to, inputs read as untouched
namespace adi {
class DigitalOut {
  bool value_;

 public:
  explicit DigitalOut(uint8_t port, bool init_state = LOW) : value_(init_state) { (void)port; }
  int32_t set_value(bool value) { value_ = value; return 1; }
  bool get_value() const { return value_; }
};

class DigitalIn {
 public:
  explicit DigitalIn(uint8_t port) { (void)port; }
  int32_t get_value() { return 0; }
  int32_t get_new_press() { return 0; }
};

class AnalogIn {
 public:
  explicit AnalogIn(uint8_t port) { (void)port; }
  int32_t calibrate() { return 0; }
  int32_t get_value() { return 0; }
  int32_t get_value_calibrated() { return 0; }
};

class Pneumatics {
  bool extended_;

 public:
  explicit Pneumatics(uint8_t port, bool start_extended, bool extended_is_low = false) : extended_(start_extended) {
    (void)port;
    (void)extended_is_low;
  }
  int32_t extend() { extended_ = true; return 1; }
  int32_t retract() { extended_ = false; return 1; }
  int32_t toggle() { extended_ = !extended_; return 1; }
  bool is_extended() const { return extended_; }
};
}  // namespace adi
// PROS 3 names, which PROS 4 still takes
using ADIDigitalOut = adi::DigitalOut;
using ADIDigitalIn = adi::DigitalIn;
using ADIAnalogIn = adi::AnalogIn;
using ADIPneumatics = adi::Pneumatics;

// ============================================================================
// CONTROLLER, COMPETITION, SD CARD, BATTERY
// ============================================================================
typedef enum { E_CONTROLLER_MASTER = 0, E_CONTROLLER_PARTNER } controller_id_e_t;
typedef enum { E_CONTROLLER_ANALOG_LEFT_X = 0, E_CONTROLLER_ANALOG_LEFT_Y, E_CONTROLLER_ANALOG_RIGHT_X, E_CONTROLLER_ANALOG_RIGHT_Y } controller_analog_e_t;
typedef enum {
  E_CONTROLLER_DIGITAL_L1 = 6, E_CONTROLLER_DIGITAL_L2, E_CONTROLLER_DIGITAL_R1, E_CONTROLLER_DIGITAL_R2,
  E_CONTROLLER_DIGITAL_UP, E_CONTROLLER_DIGITAL_DOWN, E_CONTROLLER_DIGITAL_LEFT, E_CONTROLLER_DIGITAL_RIGHT,
  E_CONTROLLER_DIGITAL_X, E_CONTROLLER_DIGITAL_B, E_CONTROLLER_DIGITAL_Y, E_CONTROLLER_DIGITAL_A,
  E_CONTROLLER_DIGITAL_POWER
} controller_digital_e_t;

inline namespace v5 {
// Nobody holds the controller in the simulator: the sticks are centered and no button is pressed
class Controller {
 public:
  explicit Controller(controller_id_e_t id) { (void)id; }
  int32_t get_analog(controller_analog_e_t channel) { (void)channel; return 0; }
  int32_t get_digital(controller_digital_e_t button) { (void)button; return 0; }
  int32_t get_digital_new_press(controller_digital_e_t button) { (void)button; return 0; }
  int32_t get_digital_new_release(controller_digital_e_t button) { (void)button; return 0; }
  template <class... Params>
  int32_t print(uint8_t line, uint8_t col, const char* fmt, Params... args) { (void)line; (void)col; (void)fmt; ((void)args, ...); return 1; }
  int32_t set_text(uint8_t line, uint8_t col, const char* str) { (void)line; (void)col; (void)str; return 1; }
  int32_t set_text(uint8_t line, uint8_t col, const std::string& str) { return set_text(line, col, str.c_str()); }
  int32_t clear() { return 1; }
  int32_t clear_line(uint8_t line) { (void)line; return 1; }
  int32_t rumble(const char* pattern) { (void)pattern; return 1; }
  int32_t is_connected() { return 1; }
  int32_t get_battery_level() { return 100; }
  int32_t get_battery_capacity() { return 100; }
};
}  // namespace v5

namespace competition {
inline uint8_t is_disabled() { return !rwsim::competitionEnabled(); }
inline uint8_t is_autonomous() { return rwsim::competitionAutonomous(); }
inline uint8_t is_connected() { return 1; }
inline uint8_t get_status() { return (is_disabled() ? 1 : 0) | (is_autonomous() ? 2 : 0) | 4; }
}  // namespace competition

// The simulator has no SD card (RW's auton selector and drive log work without one)
namespace usd {
inline int32_t is_installed() { return 0; }
}  // namespace usd

namespace battery {
inline int32_t get_capacity() { return 100; }
inline int32_t get_voltage() { return 12600; }
inline int32_t get_current() { return 2000; }
inline double get_temperature() { return 25; }
}  // namespace battery

// ============================================================================
// BRAIN SCREEN AND LLEMU (they show nothing in the simulator, and nobody taps them)
// ============================================================================
typedef enum { E_TEXT_SMALL = 0, E_TEXT_MEDIUM, E_TEXT_LARGE, E_TEXT_MEDIUM_CENTER, E_TEXT_LARGE_CENTER } text_format_e_t;
typedef enum { E_TOUCH_RELEASED = 0, E_TOUCH_PRESSED, E_TOUCH_HELD, E_TOUCH_ERROR } last_touch_e_t;
typedef struct screen_touch_status_s {
  last_touch_e_t touch_status;
  int16_t x;
  int16_t y;
  int32_t press_count;
  int32_t release_count;
} screen_touch_status_s_t;

namespace screen {
inline uint32_t set_pen(uint32_t color) { (void)color; return 1; }
inline uint32_t set_pen(Color color) { (void)color; return 1; }
inline uint32_t set_eraser(uint32_t color) { (void)color; return 1; }
inline uint32_t set_eraser(Color color) { (void)color; return 1; }
inline uint32_t erase() { return 1; }
inline uint32_t draw_pixel(int16_t x, int16_t y) { (void)x; (void)y; return 1; }
inline uint32_t draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1) { (void)x0; (void)y0; (void)x1; (void)y1; return 1; }
inline uint32_t draw_rect(int16_t x0, int16_t y0, int16_t x1, int16_t y1) { (void)x0; (void)y0; (void)x1; (void)y1; return 1; }
inline uint32_t fill_rect(int16_t x0, int16_t y0, int16_t x1, int16_t y1) { (void)x0; (void)y0; (void)x1; (void)y1; return 1; }
inline uint32_t erase_rect(int16_t x0, int16_t y0, int16_t x1, int16_t y1) { (void)x0; (void)y0; (void)x1; (void)y1; return 1; }
inline uint32_t draw_circle(int16_t x, int16_t y, int16_t radius) { (void)x; (void)y; (void)radius; return 1; }
inline uint32_t fill_circle(int16_t x, int16_t y, int16_t radius) { (void)x; (void)y; (void)radius; return 1; }
template <class... Params>
void print(text_format_e_t txt_fmt, int16_t line, const char* text, Params... args) { (void)txt_fmt; (void)line; (void)text; ((void)args, ...); }
template <class... Params>
void print(text_format_e_t txt_fmt, int16_t x, int16_t y, const char* text, Params... args) { (void)txt_fmt; (void)x; (void)y; (void)text; ((void)args, ...); }
inline screen_touch_status_s_t touch_status() { return screen_touch_status_s_t{E_TOUCH_RELEASED, 0, 0, 0, 0}; }
}  // namespace screen

#ifndef LCD_BTN_LEFT
#define LCD_BTN_LEFT 4
#define LCD_BTN_CENTER 2
#define LCD_BTN_RIGHT 1
#endif

namespace lcd {
using lcd_btn_cb_fn_t = void (*)(void);
enum class Text_Align { LEFT = 0, CENTER, RIGHT };
inline bool initialize() { return true; }
inline bool is_initialized() { return true; }
inline bool shutdown() { return true; }
template <class... Params>
bool print(int16_t line, const char* fmt, Params... args) { (void)line; (void)fmt; ((void)args, ...); return true; }
inline bool set_text(int16_t line, const std::string& text) { (void)line; (void)text; return true; }
inline bool clear() { return true; }
inline bool clear_line(int16_t line) { (void)line; return true; }
inline void set_text_align(Text_Align alignment) { (void)alignment; }
inline uint8_t read_buttons() { return 0; }
inline void register_btn0_cb(lcd_btn_cb_fn_t cb) { (void)cb; }
inline void register_btn1_cb(lcd_btn_cb_fn_t cb) { (void)cb; }
inline void register_btn2_cb(lcd_btn_cb_fn_t cb) { (void)cb; }
}  // namespace lcd

}  // namespace pros

// The short names that main.h turns on with PROS_USE_SIMPLE_NAMES
#ifdef PROS_USE_SIMPLE_NAMES
#define CONTROLLER_MASTER pros::E_CONTROLLER_MASTER
#define CONTROLLER_PARTNER pros::E_CONTROLLER_PARTNER
#define ANALOG_LEFT_X pros::E_CONTROLLER_ANALOG_LEFT_X
#define ANALOG_LEFT_Y pros::E_CONTROLLER_ANALOG_LEFT_Y
#define ANALOG_RIGHT_X pros::E_CONTROLLER_ANALOG_RIGHT_X
#define ANALOG_RIGHT_Y pros::E_CONTROLLER_ANALOG_RIGHT_Y
#define DIGITAL_L1 pros::E_CONTROLLER_DIGITAL_L1
#define DIGITAL_L2 pros::E_CONTROLLER_DIGITAL_L2
#define DIGITAL_R1 pros::E_CONTROLLER_DIGITAL_R1
#define DIGITAL_R2 pros::E_CONTROLLER_DIGITAL_R2
#define DIGITAL_UP pros::E_CONTROLLER_DIGITAL_UP
#define DIGITAL_DOWN pros::E_CONTROLLER_DIGITAL_DOWN
#define DIGITAL_LEFT pros::E_CONTROLLER_DIGITAL_LEFT
#define DIGITAL_RIGHT pros::E_CONTROLLER_DIGITAL_RIGHT
#define DIGITAL_X pros::E_CONTROLLER_DIGITAL_X
#define DIGITAL_B pros::E_CONTROLLER_DIGITAL_B
#define DIGITAL_Y pros::E_CONTROLLER_DIGITAL_Y
#define DIGITAL_A pros::E_CONTROLLER_DIGITAL_A
#define DIGITAL_POWER pros::E_CONTROLLER_DIGITAL_POWER
#define MOTOR_BRAKE_COAST pros::E_MOTOR_BRAKE_COAST
#define MOTOR_BRAKE_BRAKE pros::E_MOTOR_BRAKE_BRAKE
#define MOTOR_BRAKE_HOLD pros::E_MOTOR_BRAKE_HOLD
#define MOTOR_ENCODER_DEGREES pros::E_MOTOR_ENCODER_DEGREES
#define MOTOR_ENCODER_ROTATIONS pros::E_MOTOR_ENCODER_ROTATIONS
#define MOTOR_ENCODER_COUNTS pros::E_MOTOR_ENCODER_COUNTS
#define TEXT_SMALL pros::E_TEXT_SMALL
#define TEXT_MEDIUM pros::E_TEXT_MEDIUM
#define TEXT_LARGE pros::E_TEXT_LARGE
#define TOUCH_RELEASED pros::E_TOUCH_RELEASED
#define TOUCH_PRESSED pros::E_TOUCH_PRESSED
#define TOUCH_HELD pros::E_TOUCH_HELD
#endif
