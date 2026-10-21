#pragma once
#include <string>
#include <cmath>

// Marks a setting's default value. Defining the same variable in custom/src/robot-config.cpp
// replaces the default, and custom folders from older versions keep building without it.
#define RW_DEFAULT __attribute__((weak))

// --- Global Variables (snake_case) ---
extern bool is_turning;

extern double x_pos, y_pos;
extern double correct_angle;

// Odometry extras and motion state (read only)
extern double odom_traveled_in, odom_speed_ips, odom_turn_rate_dps;
extern bool motion_running;
extern bool mcl_running;
extern double mcl_x, mcl_y, mcl_spread, mcl_cycle_msec;

// Settings from custom/src/robot-config.cpp (src/rw-config.cpp on PROS)
extern double distance_between_wheels;
extern double wheel_distance_in;
extern double distance_kp, distance_ki, distance_kd;
extern double turn_kp, turn_ki, turn_kd;
extern double heading_correction_kp, heading_correction_ki, heading_correction_kd;
extern bool using_horizontal_tracker;
extern bool using_vertical_tracker;
extern double horizontal_tracker_dist_from_center;
extern double vertical_tracker_dist_from_center;
extern double horizontal_tracker_diameter;
extern double vertical_tracker_diameter;
extern double front_sensor_offsetX, front_sensor_offsetY;
extern double left_sensor_offsetX, left_sensor_offsetY;
extern double right_sensor_offsetX, right_sensor_offsetY;
extern double back_sensor_offsetX, back_sensor_offsetY;
extern bool heading_correction;
extern bool dir_change_start;
extern bool dir_change_end;
extern double min_output;
extern double max_slew_accel_fwd, max_slew_decel_fwd, max_slew_accel_rev, max_slew_decel_rev;
extern double chase_power;

// Newer settings (defaults in motor-control.cpp, override in custom/src/robot-config.cpp)
extern double imu_scale;
extern double stall_exit_msec;
extern double distance_reset_max_angle;
extern double distance_reset_max_jump;
extern double distance_reset_corner_margin;
extern int mcl_particles;
extern double mcl_max_range;
extern double distance_sensor_delay_msec;
extern double distance_sensor_scale;

// Which way turnToAngle() and turnProfiled() turn. They turn the short way unless you pass
// clockwise (right) or counterclockwise (left), like turnToAngle(180, clockwise, 1000)
enum class TurnDirection { counterclockwise = -1, shortest = 0, clockwise = 1 };
const TurnDirection clockwise = TurnDirection::clockwise;
const TurnDirection counterclockwise = TurnDirection::counterclockwise;

// --- Function Declarations (lowerCamelCase) ---
void driveChassis(double left_power, double right_power);

double getInertialHeading();
double normalizeTarget(double angle);

void turnToAngle(double turn_angle, double time_limit_msec, bool exit = true, double max_output = 12);
void turnToAngle(double turn_angle, TurnDirection direction, double time_limit_msec, bool exit = true, double max_output = 12);
void driveTo(double distance_in, double time_limit_msec, bool exit = true, double max_output = 12);
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit = true, double max_output = 12);

// stopChassis() takes a VEXcode or PROS brake type, so it's declared in include/vex.h (pros/rw/api.h on PROS)
void resetChassis();
double getLeftRotationDegree();
double getRightRotationDegree();
void correctHeading();
void runOdometry(bool use_horizontal, bool use_vertical);
void startOdometry();
void trackNoOdomWheel();
void trackXYOdomWheel();
void trackXOdomWheel();
void trackYOdomWheel();
void turnToPoint(double x, double y, int dir, double time_limit_msec);
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);

// Holonomic drives (X-drive, mecanum) and H-drives, in src/holonomic.cpp. Set one up with
// useHolonomicDrive() or useHDrive() (include/vex.h, or rw/api.h on PROS)
extern double strafe_distance_in;
void driveHolonomic(double forward_volts, double strafe_volts, double turn_volts);
void strafeToPose(double x, double y, double heading_deg, double time_limit_msec, bool exit = true, double max_output = 12);
void strafe(double distance_in, double time_limit_msec, bool exit = true, double max_output = 12);
bool driveCanStrafe();
double strafeEncoderDegrees();

// Motion lifecycle, async motions, and wait helpers
void beginMotion();
void endMotion();
bool motionActive(double start_time, double time_limit_msec);
void runAsync(void (*motion)());
void waitForMotion();
void waitUntilTraveled(double distance_in);
void waitUntilTurned(double angle_deg);
void waitUntilNear(double x, double y, double radius_in);
void cancelMotion();
bool isMotionRunning();
void setPose(double x, double y, double heading_deg);
void measureImuScale(int turns = 10);

void runMcl();
void startMcl();

// resetPositionWithSensor() takes a VEXcode or PROS distance sensor, so it's declared with stopChassis()
bool resetPositionWithReading(double raw_reading_in, double s_x, double s_y, double sensor_angle_offset, double field_half_size);
bool resetPositionFront();
bool resetPositionBack();
bool resetPositionLeft();
bool resetPositionRight();
