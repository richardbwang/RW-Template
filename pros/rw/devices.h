#pragma once
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

// The devices RW uses, by name. Define every one of them in src/rw-config.cpp, even the
// ones your robot doesn't have (give those any free port), the same way as on VEXcode.
extern pros::MotorGroup left_chassis;
extern pros::MotorGroup right_chassis;
extern pros::Imu inertial_sensor;
extern pros::Rotation horizontal_tracker;
extern pros::Rotation vertical_tracker;
extern pros::Distance front_sensor;
extern pros::Distance left_sensor;
extern pros::Distance right_sensor;
extern pros::Distance back_sensor;
extern pros::Controller controller_1;
