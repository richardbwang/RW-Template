#pragma once

// One distance sensor reading for MCL
struct MclReading {
  double s_x, s_y;      // Position of the sensor's face on the robot (inches, +x = right, +y = forward)
  double angle_deg;     // Direction the sensor faces on the robot (0 = front, 90 = right, 180 = back, 270 = left)
  double reading_in;    // Distance the sensor measured (inches)
};

void mclInit(int particle_count, double x, double y, double spread_in);
void mclPredict(double delta_x, double delta_y, double delta_heading_deg);
bool mclCorrect(double heading_deg, const MclReading* readings, int reading_count, double field_half_size,
                double extra_sigma_in, double shift_x = 0, double shift_y = 0);
void mclEstimate(double& x, double& y, double& spread_x, double& spread_y);
void mclNudge(double& x, double& y, double est_x, double est_y, double spread_x, double spread_y);
void mclSeed(unsigned int seed);
