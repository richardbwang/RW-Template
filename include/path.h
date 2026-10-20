#pragma once
#include <vector>

// Pure math for motion profiles, paths, pure pursuit and RAMSETE (no VEX calls, so it can be
// tested on a computer). The robot side is in src/path-following.cpp.
// Headings are compass style (degrees, 0 = +Y, clockwise positive), distances in inches.

// ============================================================================
// MOTION PROFILES
// ============================================================================

// A trapezoidal speed plan for moving a set distance: speed up, cruise, slow down
struct MotionProfile {
  double distance, max_speed, max_accel, start_speed, end_speed;
  double peak_speed, accel_time, cruise_time, decel_time, total_time;
};

MotionProfile makeProfile(double distance, double max_speed, double max_accel, double start_speed = 0, double end_speed = 0);
void profileAt(const MotionProfile& profile, double t, double& position, double& speed, double& accel);

// ============================================================================
// PATHS
// ============================================================================

// A point the path goes through, with an optional speed limit there (0-1 of max speed)
struct Waypoint {
  double x, y, speed;
  Waypoint(double x_in, double y_in, double speed_fraction = 1.0) : x(x_in), y(y_in), speed(speed_fraction) {}
};

// The path, sampled about every inch
struct PathSample {
  double x, y, heading_deg, curvature, distance, speed_limit;
};
struct Path {
  std::vector<PathSample> samples;
  double length;
  Path() : length(0) {}
};

Path pathFromPoints(const std::vector<Waypoint>& waypoints);
Path pathFromJerryIO(const char* text);

// Speed plan along a path (in/s at each sample): slow for curves, speed up and slow down at max_accel
std::vector<double> planPathSpeeds(const Path& path, double max_speed, double max_accel, double max_lateral_accel,
                                   double start_speed, double end_speed, bool limit_start);
// Time stamps for a speed plan (seconds at each sample)
std::vector<double> planPathTimes(const Path& path, const std::vector<double>& speeds);

// ============================================================================
// FOLLOWING
// ============================================================================

int closestSample(const Path& path, double x, double y, int start_index);
bool lookaheadPoint(const Path& path, double x, double y, double lookahead, int from_index, double& target_x, double& target_y);
double curvatureToPoint(double x, double y, double heading_deg, double target_x, double target_y);

void ramsete(double x, double y, double heading_deg, double ref_x, double ref_y, double ref_heading_deg,
             double ref_speed, double ref_turn_rate, double& speed, double& turn_rate);

double feedforwardVolts(double speed, double accel, double kS, double kV, double kA);
