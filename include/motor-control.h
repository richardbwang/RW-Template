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

// --- Function Declarations (lowerCamelCase) ---
void driveChassis(double left_power, double right_power);

double getInertialHeading();
double normalizeTarget(double angle);

void turnToAngle(double turn_angle, double time_limit_msec, bool exit = true, double max_output = 12);
void driveTo(double distance_in, double time_limit_msec, bool exit = true, double max_output = 12);
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 12);
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit = true, double max_output = 12);

void stopChassis(vex::brakeType type = vex::brake);
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

bool resetPositionWithSensor(vex::distance& sensor, double s_x, double s_y, double sensor_angle_offset, double field_half_size);
bool resetPositionFront();
bool resetPositionBack();
bool resetPositionLeft();
bool resetPositionRight();
