#pragma once
#include "path.h"

// Settings (defaults in path-following.cpp, override in custom/src/robot-config.cpp)
extern double drive_kS, drive_kV, drive_kA, drive_motor_rpm, drive_velocity_kP;
extern double profile_max_speed, profile_max_accel, profile_turn_speed, profile_turn_accel;
extern double path_lookahead, path_max_lateral_accel;

void driveWheelSpeeds(double left_speed, double right_speed, double left_accel, double right_accel);
void driveProfiled(double distance_in, double time_limit_msec, bool exit = true, double max_speed = 0);
void turnProfiled(double angle_deg, double time_limit_msec, bool exit = true, double max_speed = 0);
void followPath(const Path& path, double time_limit_msec, bool reverse = false, bool exit = true, double max_speed = 0, double lookahead = 0);
void followPathRamsete(const Path& path, double time_limit_msec, bool reverse = false, bool exit = true, double max_speed = 0);
void measureDriveFeedforward();
