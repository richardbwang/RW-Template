// Drivetrain physics shared by the simulator (sim-engine.cpp) and the calibration fitter
// (fit-profile.cpp), so a fitted robot profile drives the simulator exactly the same way.
#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace rwsim {

// How one drivetrain behaves. The defaults are a typical VEX drive; a robot profile
// (sim/robot-profile.txt, made by fit-profile) replaces them with a real robot's numbers.
struct DriveModel {
  double kS = 0.4;              // Volts lost to friction before the wheels turn
  double kV = 0.169;            // Volts per in/s of wheel speed
  double kA = 0.022;            // Volts per in/s^2 of wheel acceleration (motor lag)
  double max_accel = 300;       // Fastest the motors can speed the wheels up (in/s^2, motor current limit)
  double traction_accel = 350;  // Fastest the robot itself can speed up before the wheels slip (in/s^2)
  double brake_decel = 450;     // How fast brake/hold stops the wheels (in/s^2)
  double coast_decel = 60;      // How fast the wheels slow down when coasting (in/s^2)
  double tread_slip = 0.985;    // Fraction of wheel travel that actually moves the robot
  double turn_scrub = 0.965;    // Fraction of the wheel speed difference that actually turns the robot
  double lateral_skid = 0.012;  // Sideways skid in fast turns (in/s per in/s per rad/s)
};

// One side of the drive: how fast the wheels spin, and how fast that side of the robot moves
// over the tiles (lower than the wheels while slipping)
struct DriveSide {
  double wheel = 0, ground = 0;
};

enum DriveMode { DRIVE_VOLTS = 0, DRIVE_BRAKE = 1, DRIVE_COAST = 2 };

inline double clampValue(double value, double low, double high) { return value < low ? low : (value > high ? high : value); }

// Advances one side by dt seconds
inline void stepDriveSide(const DriveModel& m, DriveSide& side, int mode, double volts, double dt) {
  double accel = 0;
  if (mode == DRIVE_VOLTS) {
    // Friction opposes motion; when stopped, it holds against up to kS volts
    double friction = side.wheel > 0.05 ? m.kS : (side.wheel < -0.05 ? -m.kS : clampValue(volts, -m.kS, m.kS));
    accel = clampValue((volts - friction - m.kV * side.wheel) / m.kA, -m.max_accel, m.max_accel);
    side.wheel += accel * dt;
  } else {
    double decel = (mode == DRIVE_BRAKE ? m.brake_decel : m.coast_decel) * dt;
    side.wheel = std::fabs(side.wheel) <= decel ? 0 : side.wheel - (side.wheel > 0 ? decel : -decel);
  }
  // The robot follows the wheels, but can only speed up or slow down so fast before they slip
  double grip = m.traction_accel * dt;
  side.ground += clampValue(side.wheel - side.ground, -grip, grip);
}

// Robot speed (in/s), turn rate (rad/s, clockwise) and sideways skid (in/s, + = right) from both sides
inline void driveMotion(const DriveModel& m, const DriveSide& left, const DriveSide& right, double track_width,
                        double& speed, double& turn_rate, double& skid) {
  speed = (left.ground + right.ground) / 2.0 * m.tread_slip;
  turn_rate = (left.ground - right.ground) / track_width * m.turn_scrub;
  skid = -m.lateral_skid * speed * turn_rate;
}

// Reads "name = value" lines into a model. Returns false if the file can't be opened.
inline bool loadDriveModel(const char* path, DriveModel& m) {
  FILE* file = std::fopen(path, "r");
  if (!file) return false;
  char line[256];
  while (std::fgets(line, sizeof(line), file)) {
    char name[64];
    double value = 0;
    if (line[0] == '#' || std::sscanf(line, " %63[A-Za-z_] = %lf", name, &value) != 2) continue;
    if (!std::strcmp(name, "kS")) m.kS = value;
    else if (!std::strcmp(name, "kV")) m.kV = value;
    else if (!std::strcmp(name, "kA")) m.kA = value;
    else if (!std::strcmp(name, "max_accel")) m.max_accel = value;
    else if (!std::strcmp(name, "traction_accel")) m.traction_accel = value;
    else if (!std::strcmp(name, "brake_decel")) m.brake_decel = value;
    else if (!std::strcmp(name, "coast_decel")) m.coast_decel = value;
    else if (!std::strcmp(name, "tread_slip")) m.tread_slip = value;
    else if (!std::strcmp(name, "turn_scrub")) m.turn_scrub = value;
    else if (!std::strcmp(name, "lateral_skid")) m.lateral_skid = value;
  }
  std::fclose(file);
  return true;
}

inline bool saveDriveModel(const char* path, const DriveModel& m, const std::string& note) {
  FILE* file = std::fopen(path, "w");
  if (!file) return false;
  std::fprintf(file, "# RW-Template simulator robot profile%s%s\n", note.empty() ? "" : ": ", note.c_str());
  std::fprintf(file, "# The simulator uses this automatically. Delete it to go back to the default robot.\n");
  std::fprintf(file, "kS = %.4f\nkV = %.5f\nkA = %.5f\nmax_accel = %.1f\ntraction_accel = %.1f\n", m.kS, m.kV, m.kA, m.max_accel, m.traction_accel);
  std::fprintf(file, "brake_decel = %.1f\ncoast_decel = %.1f\ntread_slip = %.4f\nturn_scrub = %.4f\nlateral_skid = %.4f\n",
               m.brake_decel, m.coast_decel, m.tread_slip, m.turn_scrub, m.lateral_skid);
  std::fclose(file);
  return true;
}

}  // namespace rwsim
