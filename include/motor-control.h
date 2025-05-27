#include <string>
#include <cmath>

// --- Global Variables (snake_case) ---
extern bool is_turning;
extern bool heading_correction;
extern bool dir_change_start;
extern bool dir_change_end;
extern bool using_tracking_wheels;

extern double xpos, ypos;
extern double correct_angle;
extern double distance_between_wheels;
extern double chase_power;

// --- Function Declarations (lowerCamelCase) ---
void driveChassis(double left_power, double right_power);

double getInertialHeading(bool normalize = false);
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
void trackOdom();
void trackOdomWheel();
void turnToPoint(double x, double y, int dir, double time_limit_msec);
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit = true, double max_output = 12, bool overturn = false);