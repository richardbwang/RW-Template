// Pure math helpers. No VEX calls here, so this file can also be compiled and tested on a computer.
#include <math.h>
#include "utils.h"

double degToRad(double deg) {
  return (deg * M_PI / 180);
}

double radToDeg(double rad) {
  return (rad * 180 / M_PI);
}

double getRadius(double x, double y, double x1, double y1, double angle) {
  double delta_x = x1 - x;
  double delta_y = y1 - y;
  if((2 * delta_y * sin(degToRad(90 - angle))) == 0) {
    return 999;
  }
  return (delta_x * delta_x + delta_y * delta_y) / (2 * delta_y * sin(degToRad(90 - angle)));
}

/*
 * arcToChord
 * Converts how far one wheel rolled during an odometry step into how far the robot's center
 * moved along that wheel's rolling direction (the chord of the arc the robot drove).
 * - wheel_delta_in: Distance the wheel rolled (in inches).
 * - delta_heading_rad: Change in heading during the step (in radians, clockwise positive).
 * - offset_in: Wheel's offset from the robot's center, perpendicular to its rolling direction
 *   (in inches). For a forward-rolling wheel, positive = right of center.
 *   For a sideways-rolling wheel, positive = behind center.
 */
double arcToChord(double wheel_delta_in, double delta_heading_rad, double offset_in) {
  if (fabs(delta_heading_rad) < 1e-6) {
    return wheel_delta_in;
  }
  return 2.0 * sin(delta_heading_rad / 2.0) * (wheel_delta_in / delta_heading_rad + offset_in);
}

/*
 * localToGlobal
 * Rotates a robot-relative movement (x = right, y = forward) into field coordinates.
 * Heading is compass style: 0 faces +Y and it increases clockwise.
 */
void localToGlobal(double local_x, double local_y, double heading_rad, double& global_x, double& global_y) {
  global_x = local_x * cos(heading_rad) + local_y * sin(heading_rad);
  global_y = -local_x * sin(heading_rad) + local_y * cos(heading_rad);
}

/*
 * odomStep
 * One odometry update: turns wheel travel and heading change into field movement.
 * - prev_heading_deg, heading_deg: Heading at the start and end of the step (in degrees).
 * - delta_left_in, delta_right_in: Drivetrain wheel travel (in inches), used without a vertical tracker.
 * - delta_horizontal_in, delta_vertical_in: Tracking wheel travel (in inches).
 * - use_horizontal, use_vertical: Which tracking wheels are installed.
 * - track_width: Distance between the left and right wheels (in inches).
 * - horizontal_offset: Horizontal tracker distance behind the center (in inches).
 * - vertical_offset: Vertical tracker distance right of the center (in inches).
 * - delta_x, delta_y: Set to the movement in field coordinates (in inches).
 */
void odomStep(double prev_heading_deg, double heading_deg, double delta_left_in, double delta_right_in,
              double delta_horizontal_in, double delta_vertical_in, bool use_horizontal, bool use_vertical,
              double track_width, double horizontal_offset, double vertical_offset, double& delta_x, double& delta_y) {
  double delta_heading_rad = degToRad(heading_deg - prev_heading_deg);

  // Local X (sideways) movement, only measurable with a horizontal tracking wheel
  double delta_local_x = use_horizontal ? arcToChord(delta_horizontal_in, delta_heading_rad, horizontal_offset) : 0;

  // Local Y (forward) movement, from the vertical tracking wheel or the drivetrain encoders.
  // The left wheels sit left of center (negative offset), the right wheels right of center.
  double delta_local_y = use_vertical
    ? arcToChord(delta_vertical_in, delta_heading_rad, vertical_offset)
    : (arcToChord(delta_left_in, delta_heading_rad, -track_width / 2.0) +
       arcToChord(delta_right_in, delta_heading_rad, track_width / 2.0)) / 2.0;

  // Rotate into field coordinates using the average heading over this step
  localToGlobal(delta_local_x, delta_local_y, degToRad(prev_heading_deg) + delta_heading_rad / 2.0, delta_x, delta_y);
}

/*
 * beamToWall
 * Distance along a sensor beam to the field wall it hits.
 * - sensor_x, sensor_y: Sensor position on the field (in inches).
 * - beam_x, beam_y: Unit vector the beam points along.
 * - field_half_size: Distance from the field center to each wall (in inches).
 * - hits_x_wall: Set to true if the beam hits an east/west wall (x = +/-half), false for north/south.
 * - other_wall_gap: Set to how much farther the other wall is (small = the beam points into a corner).
 */
double beamToWall(double sensor_x, double sensor_y, double beam_x, double beam_y, double field_half_size, bool& hits_x_wall, double& other_wall_gap) {
  double wall_x = beam_x > 0 ? field_half_size : -field_half_size;
  double wall_y = beam_y > 0 ? field_half_size : -field_half_size;
  double to_wall_x = fabs(beam_x) > 1e-9 ? (wall_x - sensor_x) / beam_x : 1e9;
  double to_wall_y = fabs(beam_y) > 1e-9 ? (wall_y - sensor_y) / beam_y : 1e9;
  hits_x_wall = to_wall_x < to_wall_y;
  other_wall_gap = fabs(to_wall_x - to_wall_y);
  return hits_x_wall ? to_wall_x : to_wall_y;
}

/*
 * distanceResetPosition
 * Works out the robot's X or Y from one distance sensor reading against the field wall.
 * Returns 1 if it found a new X, 2 if it found a new Y, or 0 if the reading should be skipped
 * (beam points into a corner, hits too close to a corner, or hits the wall too steeply).
 * - robot_x, robot_y, heading_deg: Current pose estimate (in inches and degrees).
 * - s_x, s_y: Position of the sensor's face on the robot (in inches, +x = right, +y = forward).
 * - sensor_angle_offset: Direction the sensor faces on the robot (in degrees, 0 = front, 90 = right).
 * - reading_in: Distance the sensor measured (in inches).
 * - field_half_size: Distance from the field center to each wall (in inches).
 * - max_angle_deg: Steepest beam angle allowed, measured from hitting the wall straight on (in degrees).
 * - corner_margin: Skip readings that hit the wall this close to a corner (in inches, 0 = off).
 * - new_pos: Set to the new X or Y.
 */
int distanceResetPosition(double robot_x, double robot_y, double heading_deg, double s_x, double s_y, double sensor_angle_offset,
                          double reading_in, double field_half_size, double max_angle_deg, double corner_margin, double& new_pos) {
  double beam_rad = degToRad(heading_deg + sensor_angle_offset);
  double beam_x = sin(beam_rad), beam_y = cos(beam_rad);

  // Where the sensor is on the field (kept inside the walls so the beam math stays valid)
  double offset_x = 0, offset_y = 0;
  localToGlobal(s_x, s_y, degToRad(heading_deg), offset_x, offset_y);
  double sensor_x = fmax(-field_half_size + 0.1, fmin(field_half_size - 0.1, robot_x + offset_x));
  double sensor_y = fmax(-field_half_size + 0.1, fmin(field_half_size - 0.1, robot_y + offset_y));

  bool hits_x_wall = false;
  double other_wall_gap = 0;
  double beam_length = beamToWall(sensor_x, sensor_y, beam_x, beam_y, field_half_size, hits_x_wall, other_wall_gap);

  // Too close to call which wall the beam hits
  if (other_wall_gap < 2) {
    return 0;
  }
  // Also too close to call if being a little off (normal odometry drift, up to 2" in X and Y)
  // would change which wall the beam hits. Near a corner at a shallow angle, that's possible.
  const double drift = 2;
  for (int corner = 0; corner < 4; corner++) {
    double check_x = fmax(-field_half_size + 0.1, fmin(field_half_size - 0.1, sensor_x + (corner & 1 ? drift : -drift)));
    double check_y = fmax(-field_half_size + 0.1, fmin(field_half_size - 0.1, sensor_y + (corner & 2 ? drift : -drift)));
    bool hits_x_there = false;
    double gap_there = 0;
    beamToWall(check_x, check_y, beam_x, beam_y, field_half_size, hits_x_there, gap_there);
    if (hits_x_there != hits_x_wall) {
      return 0;
    }
  }
  // Steep angles give unreliable readings
  double facing_wall = hits_x_wall ? fabs(beam_x) : fabs(beam_y);
  if (facing_wall < cos(degToRad(max_angle_deg))) {
    return 0;
  }
  // Corners can hold field elements (e.g. loaders) in front of the wall
  double hit_along_wall = hits_x_wall ? sensor_y + beam_length * beam_y : sensor_x + beam_length * beam_x;
  if (corner_margin > 0 && fabs(hit_along_wall) > field_half_size - corner_margin) {
    return 0;
  }

  // Wall position, minus how far the beam traveled, minus where the sensor sits on the robot
  if (hits_x_wall) {
    new_pos = (beam_x > 0 ? field_half_size : -field_half_size) - reading_in * beam_x - offset_x;
    return 1;
  }
  new_pos = (beam_y > 0 ? field_half_size : -field_half_size) - reading_in * beam_y - offset_y;
  return 2;
}
