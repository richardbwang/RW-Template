// RW-Template simulator engine, runner side: sim-main.cpp sets it up and runs it.
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include "drive-model.h"

namespace rwsim {
  // A distance sensor on the robot, used to reset or localize against the field
  struct DistanceMount {
    int32_t port;
    double x, y;       // Sensor face position on the robot (inches, +x = right, +y = forward)
    double angle_deg;  // Direction it faces on the robot (0 = front, 90 = right)
  };

  // The simulated robot, read from custom/src/robot-config.cpp
  struct RobotSetup {
    std::vector<int32_t> left_ports, right_ports;
    double track_width_in = 12;
    double inches_per_motor_rev = 7;
    double robot_size_in = 15;
    int32_t horizontal_tracker_port = -1, vertical_tracker_port = -1;
    double horizontal_offset_in = 0, vertical_offset_in = 0; // + = behind center / + = right of center
    double horizontal_diameter_in = 2, vertical_diameter_in = 2;
    std::vector<DistanceMount> distance_sensors;
    DriveModel drive;            // Drivetrain physics
    bool drive_from_profile = false;  // false = estimate the default drive from the motors and wheels
  };

  // The simulated field and how realistic the robot is
  struct WorldSetup {
    double start_x = -35, start_y = -60, start_heading_deg = 0; // Field-centered inches, compass heading
    double field_half_in = 70.25;
    bool override_field = true; // Override goals, loaders and wall stacks (otherwise walls only)
    bool realistic = true;      // Wheel slip, IMU error, sensor noise and delay
    unsigned seed = 1;
    double time_limit_s = 15;
  };

  // Where to read what RW thinks (pointers to RW's own globals)
  struct BeliefSource {
    double* x = nullptr;
    double* y = nullptr;
    double (*heading)() = nullptr;
    bool* motion_running = nullptr;
    bool* mcl_running = nullptr;
    double* mcl_x = nullptr;
    double* mcl_y = nullptr;
  };

  void adoptMainThread();
  void setup(const RobotSetup& robot, const WorldSetup& world, const BeliefSource& belief);
  void setCompetition(bool enabled, bool autonomous);
  void startRecording(const std::string& name, const std::string& viewer_path, const std::string& output_path);
  [[noreturn]] void finishRun(const std::string& reason);
}
