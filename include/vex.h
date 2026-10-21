/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "../custom/include/robot-config.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)

// RW functions that take VEXcode types (in src/vexcode/hardware.cpp; the PROS versions are in pros/rw/api.h)
void stopChassis(vex::brakeType type = vex::brake);
bool resetPositionWithSensor(vex::distance& sensor, double s_x, double s_y, double sensor_angle_offset, double field_half_size);
void useHolonomicDrive(vex::motor& left_front, vex::motor& right_front, vex::motor& left_back, vex::motor& right_back);
void useHDrive(vex::motor& strafe_motor);