#pragma once

// Driver control settings (defaults in driver-control.cpp, override in custom/src/robot-config.cpp)
extern double driver_curve;
extern double driver_deadband;

extern double driver_forward_heading;

double driverCurve(double stick);
void driverTank(double left_stick, double right_stick);
void driverArcade(double throttle_stick, double turn_stick);
void driverHolonomic(double forward_stick, double strafe_stick, double turn_stick);
void driverFieldCentric(double forward_stick, double strafe_stick, double turn_stick);
