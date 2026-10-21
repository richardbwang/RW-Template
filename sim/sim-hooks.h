// The simulator engine's hooks (implemented in sim/sim-engine.cpp). The stand-in VEX and PROS APIs
// (sim/vex/v5_vcs.h, sim/pros/pros-sim.h) talk to the simulated robot through these.
#pragma once
#include <cstdint>
#include <functional>

namespace rwsim {
  double nowMs();
  void sleepMs(double ms);
  int32_t spawnThread(std::function<void()> body);
  void interruptThread(int32_t id);
  bool threadRunning(int32_t id);
  int32_t currentThreadId();

  // Motors: one state per port, so two motor objects on the same port share it
  void motorRegister(int32_t port, double free_rpm);
  void motorVoltage(int32_t port, double volts);
  void motorPercent(int32_t port, double percent);
  void motorStop(int32_t port, int brake_mode);
  void motorSetStopping(int32_t port, int brake_mode);
  void motorSetVelocity(int32_t port, double percent);
  double motorVelocitySetting(int32_t port);
  double motorPositionDeg(int32_t port);
  void motorSetPositionDeg(int32_t port, double deg);
  double motorRpm(int32_t port);
  double motorFreeRpm(int32_t port);

  // Sensors
  double imuRotationDeg(int32_t port);
  void imuSetRotationDeg(int32_t port, double deg);
  void imuCalibrate(int32_t port);
  bool imuCalibrating(int32_t port);
  double imuRateDps(int32_t port);
  double rotationSensorDeg(int32_t port);
  void rotationSensorSetDeg(int32_t port, double deg);
  double rotationSensorRpm(int32_t port);
  double distanceMm(int32_t port);
  uint32_t distanceTimestamp(int32_t port);

  // Competition
  bool competitionEnabled();
  bool competitionAutonomous();
}
