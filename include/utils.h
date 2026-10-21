#ifndef __UTILS__
#define __UTILS__

//declarations for utilities such as graphing classes and math functions.

#include <vector>
#include <string>
#include <iostream>
#include <climits>
#include <algorithm>

double degToRad(double deg);

double radToDeg(double rad);

double turnTarget(double angle_deg, double heading_deg, int direction);

double getRadius(double x, double y, double x1, double y1, double angle);

double arcToChord(double wheel_delta_in, double delta_heading_rad, double offset_in);

void localToGlobal(double local_x, double local_y, double heading_rad, double& global_x, double& global_y);

void odomStep(double prev_heading_deg, double heading_deg, double delta_left_in, double delta_right_in,
              double delta_horizontal_in, double delta_vertical_in, bool use_horizontal, bool use_vertical,
              double track_width, double horizontal_offset, double vertical_offset, double& delta_x, double& delta_y);

double beamToWall(double sensor_x, double sensor_y, double beam_x, double beam_y, double field_half_size, bool& hits_x_wall, double& other_wall_gap);

int distanceResetPosition(double robot_x, double robot_y, double heading_deg, double s_x, double s_y, double sensor_angle_offset,
                          double reading_in, double field_half_size, double max_angle_deg, double corner_margin, double& new_pos);

#endif
