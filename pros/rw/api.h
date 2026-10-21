#pragma once
// RW-Template for PROS: include this wherever you use RW (include/rw-config.h includes it, so
// your own files can include that instead). The functions and settings are the same as on VEXcode.
#include "devices.h"
#include "motor-control.h"
#include "path-following.h"
#include "auton-selector.h"
#include "driver-control.h"
#include "drive-log.h"
#include "pid.h"
#include "path.h"
#include "utils.h"

// RW functions that take PROS types (in rw/hardware.cpp; the VEXcode versions are in include/vex.h)
void stopChassis(pros::MotorBrake type = pros::MotorBrake::brake);
bool resetPositionWithSensor(pros::Distance& sensor, double s_x, double s_y, double sensor_angle_offset, double field_half_size);
void useHolonomicDrive(pros::Motor& left_front, pros::Motor& right_front, pros::Motor& left_back, pros::Motor& right_back);
void useHDrive(pros::Motor& strafe_motor);
