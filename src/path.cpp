// Motion profiles, paths, pure pursuit and RAMSETE. Pure math with no VEX calls, so it can be
// tested on a computer. The robot side is in src/path-following.cpp.
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "utils.h"
#include "path.h"

// ============================================================================
// MOTION PROFILES
// ============================================================================

/*
 * makeProfile
 * Plans a trapezoidal move: speed up at max_accel, cruise at max_speed if there's room,
 * then slow down to end_speed right at the end.
 * - distance: How far to move (negative = backward). Any unit (inches, degrees).
 * - max_speed, max_accel: Limits (units per second, units per second squared).
 * - start_speed, end_speed: Speed at the start and end (units per second, 0 or more).
 */
MotionProfile makeProfile(double distance, double max_speed, double max_accel, double start_speed, double end_speed) {
  MotionProfile p;
  p.distance = distance;
  p.max_speed = fabs(max_speed);
  p.max_accel = fmax(fabs(max_accel), 1e-6);
  double d = fabs(distance), a = p.max_accel;
  double v0 = fmin(fabs(start_speed), p.max_speed), v1 = fmin(fabs(end_speed), p.max_speed);
  // Too short to slow down to end_speed (or to speed up to it): end at the closest speed possible
  if (v0 * v0 - 2 * a * d > v1 * v1) v1 = sqrt(v0 * v0 - 2 * a * d);
  if (v1 * v1 - 2 * a * d > v0 * v0) v1 = sqrt(v0 * v0 + 2 * a * d);
  // Highest speed reachable while still slowing to end_speed in time
  double peak = fmin(p.max_speed, sqrt((2 * a * d + v0 * v0 + v1 * v1) / 2));
  peak = fmax(peak, fmax(v0, v1));
  p.start_speed = v0;
  p.end_speed = v1;
  p.peak_speed = peak;
  p.accel_time = (peak - v0) / a;
  p.decel_time = (peak - v1) / a;
  double accel_distance = (peak * peak - v0 * v0) / (2 * a);
  double decel_distance = (peak * peak - v1 * v1) / (2 * a);
  double cruise_distance = fmax(0, d - accel_distance - decel_distance);
  p.cruise_time = peak > 1e-9 ? cruise_distance / peak : 0;
  p.total_time = p.accel_time + p.cruise_time + p.decel_time;
  return p;
}

/*
 * profileAt
 * Where a profile says to be at time t.
 * - t: Seconds since the move started.
 * - position, speed, accel: Set to the planned position, speed and acceleration
 *   (negative when the move is backward).
 */
void profileAt(const MotionProfile& p, double t, double& position, double& speed, double& accel) {
  double sign = p.distance < 0 ? -1 : 1;
  double a = p.max_accel, v0 = p.start_speed, peak = p.peak_speed;
  double accel_distance = (peak * peak - v0 * v0) / (2 * a);
  double cruise_distance = peak * p.cruise_time;
  if (t <= 0) {
    position = 0; speed = v0; accel = p.accel_time > 0 ? a : 0;
  } else if (t < p.accel_time) {
    position = v0 * t + 0.5 * a * t * t; speed = v0 + a * t; accel = a;
  } else if (t < p.accel_time + p.cruise_time) {
    position = accel_distance + peak * (t - p.accel_time); speed = peak; accel = 0;
  } else if (t < p.total_time) {
    double into = t - p.accel_time - p.cruise_time;
    position = accel_distance + cruise_distance + peak * into - 0.5 * a * into * into;
    speed = peak - a * into;
    accel = -a;
  } else {
    position = fabs(p.distance); speed = p.end_speed; accel = 0;
  }
  position *= sign;
  speed *= sign;
  accel *= sign;
}

// ============================================================================
// PATHS
// ============================================================================

// Point on a centripetal Catmull-Rom curve between p1 and p2 (u from 0 to 1). Centripetal
// curves go through every point without loops or sharp cusps.
static void catmullRom(const Waypoint& p0, const Waypoint& p1, const Waypoint& p2, const Waypoint& p3,
                       double u, double& x, double& y) {
  double t0 = 0;
  double t1 = t0 + fmax(sqrt(hypot(p1.x - p0.x, p1.y - p0.y)), 1e-4);
  double t2 = t1 + fmax(sqrt(hypot(p2.x - p1.x, p2.y - p1.y)), 1e-4);
  double t3 = t2 + fmax(sqrt(hypot(p3.x - p2.x, p3.y - p2.y)), 1e-4);
  double t = t1 + u * (t2 - t1);
  double a1x = (t1 - t) / (t1 - t0) * p0.x + (t - t0) / (t1 - t0) * p1.x;
  double a1y = (t1 - t) / (t1 - t0) * p0.y + (t - t0) / (t1 - t0) * p1.y;
  double a2x = (t2 - t) / (t2 - t1) * p1.x + (t - t1) / (t2 - t1) * p2.x;
  double a2y = (t2 - t) / (t2 - t1) * p1.y + (t - t1) / (t2 - t1) * p2.y;
  double a3x = (t3 - t) / (t3 - t2) * p2.x + (t - t2) / (t3 - t2) * p3.x;
  double a3y = (t3 - t) / (t3 - t2) * p2.y + (t - t2) / (t3 - t2) * p3.y;
  double b1x = (t2 - t) / (t2 - t0) * a1x + (t - t0) / (t2 - t0) * a2x;
  double b1y = (t2 - t) / (t2 - t0) * a1y + (t - t0) / (t2 - t0) * a2y;
  double b2x = (t3 - t) / (t3 - t1) * a2x + (t - t1) / (t3 - t1) * a3x;
  double b2y = (t3 - t) / (t3 - t1) * a2y + (t - t1) / (t3 - t1) * a3y;
  x = (t2 - t) / (t2 - t1) * b1x + (t - t1) / (t2 - t1) * b2x;
  y = (t2 - t) / (t2 - t1) * b1y + (t - t1) / (t2 - t1) * b2y;
}

// Turns a dense list of points into samples about every inch, with headings and curvature
static Path resamplePath(const std::vector<Waypoint>& dense) {
  Path path;
  if (dense.empty()) {
    return path;
  }
  const double spacing = 1.0;
  PathSample first = {dense[0].x, dense[0].y, 0, 0, 0, dense[0].speed};
  path.samples.push_back(first);
  double carried = 0;  // Distance walked since the last sample
  for (size_t i = 1; i < dense.size(); i++) {
    double x0 = dense[i - 1].x, y0 = dense[i - 1].y;
    double seg = hypot(dense[i].x - x0, dense[i].y - y0);
    double along = spacing - carried;
    while (along <= seg) {
      double f = along / seg;
      PathSample s = {x0 + (dense[i].x - x0) * f, y0 + (dense[i].y - y0) * f, 0, 0, 0,
                      dense[i - 1].speed + (dense[i].speed - dense[i - 1].speed) * f};
      path.samples.push_back(s);
      along += spacing;
    }
    carried = seg - (along - spacing);
  }
  // Always end exactly on the last point
  const Waypoint& last = dense.back();
  PathSample& end = path.samples.back();
  if (hypot(end.x - last.x, end.y - last.y) > 0.05) {
    PathSample s = {last.x, last.y, 0, 0, 0, last.speed};
    path.samples.push_back(s);
  }

  // Distance along the path, heading of travel, and curvature (+ = curving right)
  size_t n = path.samples.size();
  for (size_t i = 1; i < n; i++) {
    path.samples[i].distance = path.samples[i - 1].distance +
      hypot(path.samples[i].x - path.samples[i - 1].x, path.samples[i].y - path.samples[i - 1].y);
  }
  path.length = path.samples[n - 1].distance;
  for (size_t i = 0; i < n; i++) {
    size_t a = i > 0 ? i - 1 : i, b = i + 1 < n ? i + 1 : i;
    if (a != b) {
      path.samples[i].heading_deg = radToDeg(atan2(path.samples[b].x - path.samples[a].x, path.samples[b].y - path.samples[a].y));
    } else if (i > 0) {
      path.samples[i].heading_deg = path.samples[i - 1].heading_deg;
    }
    // Curvature from a circle through points two samples apart (smoother than neighbors)
    if (i >= 2 && i + 2 < n) {
      const PathSample& p = path.samples[i - 2];
      const PathSample& q = path.samples[i];
      const PathSample& r = path.samples[i + 2];
      double cross = (q.x - p.x) * (r.y - q.y) - (q.y - p.y) * (r.x - q.x);
      double lengths = hypot(q.x - p.x, q.y - p.y) * hypot(r.x - q.x, r.y - q.y) * hypot(r.x - p.x, r.y - p.y);
      path.samples[i].curvature = lengths > 1e-9 ? -2 * cross / lengths : 0;
    }
  }
  if (n >= 5) {
    path.samples[0].curvature = path.samples[1].curvature = path.samples[2].curvature;
    path.samples[n - 1].curvature = path.samples[n - 2].curvature = path.samples[n - 3].curvature;
  }
  return path;
}

/*
 * pathFromPoints
 * A smooth path through each waypoint in order (a centripetal Catmull-Rom spline).
 * - waypoints: Field points (inches), e.g. {{-36, -60}, {-36, -24}, {0, -12}}.
 *   An optional third number limits the speed there (0-1 of max speed), e.g. {0, -12, 0.5}.
 */
Path pathFromPoints(const std::vector<Waypoint>& waypoints) {
  if (waypoints.size() < 2) {
    return resamplePath(waypoints);
  }
  std::vector<Waypoint> padded;
  size_t n = waypoints.size();
  // Extra points past each end so the curve leaves and arrives in a straight line
  padded.push_back(Waypoint(2 * waypoints[0].x - waypoints[1].x, 2 * waypoints[0].y - waypoints[1].y, waypoints[0].speed));
  padded.insert(padded.end(), waypoints.begin(), waypoints.end());
  padded.push_back(Waypoint(2 * waypoints[n - 1].x - waypoints[n - 2].x, 2 * waypoints[n - 1].y - waypoints[n - 2].y, waypoints[n - 1].speed));

  std::vector<Waypoint> dense;
  for (size_t i = 1; i + 2 < padded.size(); i++) {
    const int steps = 50;
    for (int k = (i == 1 ? 0 : 1); k <= steps; k++) {
      double u = (double)k / steps, x = 0, y = 0;
      catmullRom(padded[i - 1], padded[i], padded[i + 1], padded[i + 2], u, x, y);
      dense.push_back(Waypoint(x, y, padded[i].speed + (padded[i + 1].speed - padded[i].speed) * u));
    }
  }
  return resamplePath(dense);
}

/*
 * pathFromJerryIO
 * A path drawn in path.jerryio. Save it in the "LemLib v0.5" format and paste the whole file in.
 * Each line is "x, y, speed" until "endData"; speed is 0-127 and limits the speed there.
 */
Path pathFromJerryIO(const char* text) {
  std::vector<Waypoint> points;  // Speeds stay 0-127 until the end
  double decel_rate = 0, top_speed = 127;
  int settings_read = 0;  // After "endData": jerryio's deceleration rate, then the path's top speed
  bool after_end = false;
  const char* line = text;
  while (line && *line) {
    const char* next = strchr(line, '\n');
    size_t length = next ? (size_t)(next - line) : strlen(line);
    char buffer[96];
    size_t copy = length < sizeof(buffer) - 1 ? length : sizeof(buffer) - 1;
    memcpy(buffer, line, copy);
    buffer[copy] = 0;
    const char* start = buffer + strspn(buffer, " \t");
    double x = 0, y = 0, speed = 127;
    if (strncmp(start, "endData", 7) == 0) {
      after_end = true;
    } else if (after_end) {
      if (settings_read >= 2 || sscanf(start, "%lf", settings_read == 0 ? &decel_rate : &top_speed) != 1) {
        break;
      }
      settings_read++;
    } else if (sscanf(start, "%lf , %lf , %lf", &x, &y, &speed) >= 2) {
      points.push_back(Waypoint(x, y, speed));
    }
    line = next ? next + 1 : 0;
  }

  // jerryio adds a "ghost point" 20" past the end for LemLib's follower. Drop it, so the robot
  // stops where the path was drawn.
  size_t n = points.size();
  if (n >= 3 && points[n - 1].speed == 0 &&
      fabs(hypot(points[n - 1].x - points[n - 2].x, points[n - 1].y - points[n - 2].y) - 20) < 0.5) {
    points.pop_back();
  }
  // jerryio also repeats the end point
  std::vector<Waypoint> clean;
  for (size_t i = 0; i < points.size(); i++) {
    if (clean.empty() || hypot(points[i].x - clean.back().x, points[i].y - clean.back().y) > 0.01) {
      clean.push_back(points[i]);
    }
  }
  n = clean.size();
  if (n < 2) {
    printf("pathFromJerryIO: found %d path points. Save the path in the LemLib v0.5 format and paste the whole file\n", (int)n);
    return resamplePath(clean);
  }

  // jerryio slows every path to a stop at decel_rate over its last few feet. RW plans its own
  // slow-down from your robot's acceleration, so raise that ramp to the speed just before it.
  if (decel_rate > 0 && clean[n - 1].speed == 0) {
    size_t i = n - 1;
    while (i > 0) {
      double ramp = sqrt(clean[i].speed * clean[i].speed + 2 * decel_rate * hypot(clean[i].x - clean[i - 1].x, clean[i].y - clean[i - 1].y));
      if (clean[i - 1].speed < ramp - 1.5) {
        break;  // Slower than the ramp needs: a speed set in jerryio, or a curve
      }
      i--;
    }
    double before_ramp = i > 0 ? clean[i - 1].speed : top_speed;
    for (size_t k = i; k < n; k++) {
      clean[k].speed = before_ramp;
    }
  }
  for (size_t i = 0; i < n; i++) {
    clean[i].speed = fmin(1.0, fmax(0.05, clean[i].speed / 127.0));
  }
  return resamplePath(clean);
}

/*
 * planPathSpeeds
 * The speed to drive at each sample of a path (in/s).
 * - max_speed: Top speed (in/s), scaled down by each waypoint's speed limit.
 * - max_accel: How fast to speed up and slow down (in/s^2).
 * - max_lateral_accel: Limits speed in curves so the robot doesn't slide (in/s^2).
 * - start_speed, end_speed: Speed at the start and end of the path.
 * - limit_start: true to also ramp up from start_speed (for timed trajectories).
 */
std::vector<double> planPathSpeeds(const Path& path, double max_speed, double max_accel, double max_lateral_accel,
                                   double start_speed, double end_speed, bool limit_start) {
  size_t n = path.samples.size();
  std::vector<double> speeds(n, 0.0);
  for (size_t i = 0; i < n; i++) {
    double limit = max_speed * path.samples[i].speed_limit;
    double curvature = fabs(path.samples[i].curvature);
    if (curvature > 1e-6) limit = fmin(limit, sqrt(max_lateral_accel / curvature));
    speeds[i] = limit;
  }
  if (n == 0) {
    return speeds;
  }
  if (limit_start) {
    speeds[0] = fmin(speeds[0], start_speed);
    for (size_t i = 1; i < n; i++) {
      double gap = path.samples[i].distance - path.samples[i - 1].distance;
      speeds[i] = fmin(speeds[i], sqrt(speeds[i - 1] * speeds[i - 1] + 2 * max_accel * gap));
    }
  }
  speeds[n - 1] = fmin(speeds[n - 1], end_speed);
  for (size_t i = n - 1; i > 0; i--) {
    double gap = path.samples[i].distance - path.samples[i - 1].distance;
    speeds[i - 1] = fmin(speeds[i - 1], sqrt(speeds[i] * speeds[i] + 2 * max_accel * gap));
  }
  return speeds;
}

/*
 * planPathTimes
 * When the robot should reach each sample, driving at the planned speeds (seconds).
 */
std::vector<double> planPathTimes(const Path& path, const std::vector<double>& speeds) {
  size_t n = path.samples.size();
  std::vector<double> times(n, 0.0);
  for (size_t i = 1; i < n; i++) {
    double gap = path.samples[i].distance - path.samples[i - 1].distance;
    double average = fmax((speeds[i - 1] + speeds[i]) / 2, 0.5);
    times[i] = times[i - 1] + gap / average;
  }
  return times;
}

// ============================================================================
// FOLLOWING
// ============================================================================

/*
 * closestSample
 * Index of the path sample nearest the robot, searching ahead of start_index (never backward,
 * so a path that crosses itself doesn't confuse it).
 */
int closestSample(const Path& path, double x, double y, int start_index) {
  int n = (int)path.samples.size();
  int best = start_index < 0 ? 0 : (start_index >= n ? n - 1 : start_index);
  double best_distance = 1e18;
  int stop = start_index + 40 < n ? start_index + 40 : n;
  for (int i = best; i < stop; i++) {
    double d = hypot(path.samples[i].x - x, path.samples[i].y - y);
    if (d < best_distance) {
      best_distance = d;
      best = i;
    }
  }
  return best;
}

/*
 * lookaheadPoint
 * The point on the path where a circle of radius lookahead around the robot crosses it,
 * furthest along the path (pure pursuit chases this point). Once the end is inside the circle,
 * the path continues straight past its end, so the robot arrives lined up with the path instead
 * of swinging toward the end point. Returns false if the robot is too far off the path to find one.
 */
bool lookaheadPoint(const Path& path, double x, double y, double lookahead, int from_index, double& target_x, double& target_y) {
  int n = (int)path.samples.size();
  if (n == 0) {
    return false;
  }
  bool found = false;
  int stop = from_index + (int)(lookahead * 3) + 5;
  if (stop > n - 1) stop = n - 1;
  for (int i = from_index < 0 ? 0 : from_index; i < stop; i++) {
    const PathSample& a = path.samples[i];
    const PathSample& b = path.samples[i + 1];
    double dx = b.x - a.x, dy = b.y - a.y, fx = a.x - x, fy = a.y - y;
    double qa = dx * dx + dy * dy, qb = 2 * (fx * dx + fy * dy), qc = fx * fx + fy * fy - lookahead * lookahead;
    double disc = qb * qb - 4 * qa * qc;
    if (qa < 1e-12 || disc < 0) continue;
    double root = sqrt(disc);
    double t2 = (-qb + root) / (2 * qa), t1 = (-qb - root) / (2 * qa);
    double t = (t2 >= 0 && t2 <= 1) ? t2 : ((t1 >= 0 && t1 <= 1) ? t1 : -1);
    if (t >= 0) {
      target_x = a.x + dx * t;
      target_y = a.y + dy * t;
      found = true;
    }
  }
  const PathSample& end = path.samples[n - 1];
  double ex = end.x - x, ey = end.y - y;
  if (!found && hypot(ex, ey) <= lookahead) {
    // Where the circle crosses the line continuing straight past the end
    double ux = sin(degToRad(end.heading_deg)), uy = cos(degToRad(end.heading_deg));
    double along = ex * ux + ey * uy;
    double past = -along + sqrt(fmax(0.0, along * along - (ex * ex + ey * ey) + lookahead * lookahead));
    target_x = end.x + ux * past;
    target_y = end.y + uy * past;
    found = true;
  }
  return found;
}

/*
 * curvatureToPoint
 * Curvature (1/inches, + = curving right) of the arc that starts at the robot pointing along
 * its heading and passes through the target.
 */
double curvatureToPoint(double x, double y, double heading_deg, double target_x, double target_y) {
  double h = degToRad(heading_deg);
  double dx = target_x - x, dy = target_y - y;
  double local_x = dx * cos(h) - dy * sin(h);   // To the robot's right
  double local_y = dx * sin(h) + dy * cos(h);   // In front of the robot
  double distance_squared = local_x * local_x + local_y * local_y;
  return distance_squared > 1e-9 ? 2 * local_x / distance_squared : 0;
}

/*
 * ramsete
 * RAMSETE trajectory tracking for a tank drive: given where the robot is and where the timed
 * path says it should be (and how fast it should be going), returns the speed and turn rate
 * to command. It steers back onto the path and catches up or waits as needed.
 * - x, y, heading_deg: Robot pose.
 * - ref_x, ref_y, ref_heading_deg: Where the path says to be right now.
 * - ref_speed: Planned speed (in/s, negative when driving backward).
 * - ref_turn_rate: Planned turn rate (radians/s, + = clockwise).
 * - speed, turn_rate: Set to the commands (in/s and radians/s clockwise).
 */
void ramsete(double x, double y, double heading_deg, double ref_x, double ref_y, double ref_heading_deg,
             double ref_speed, double ref_turn_rate, double& speed, double& turn_rate) {
  // How hard to steer back onto the path. The usual value is 2.0 per square meter for full-size
  // (FRC) robots; VEX paths are smaller, and 12 tracked best in simulation. Converted to per square inch.
  const double b = 12.0 * 0.0254 * 0.0254;
  const double zeta = 0.7;
  // Work in the usual math frame (counterclockwise from +X)
  double theta = M_PI / 2 - degToRad(heading_deg);
  double ref_theta = M_PI / 2 - degToRad(ref_heading_deg);
  double dx = ref_x - x, dy = ref_y - y;
  double error_x = cos(theta) * dx + sin(theta) * dy;    // Ahead of the robot
  double error_y = -sin(theta) * dx + cos(theta) * dy;   // To the robot's left
  double error_theta = remainder(ref_theta - theta, 2 * M_PI);
  double ref_omega = -ref_turn_rate;
  // A minimum gain keeps correcting at the end of the path, where the planned speed is zero
  double k = fmax(2 * zeta * sqrt(ref_omega * ref_omega + b * ref_speed * ref_speed), 1.5);
  double sinc = fabs(error_theta) < 1e-6 ? 1.0 : sin(error_theta) / error_theta;
  speed = ref_speed * cos(error_theta) + k * error_x;
  double omega = ref_omega + k * error_theta + b * ref_speed * sinc * error_y;
  turn_rate = -omega;
}

/*
 * feedforwardVolts
 * Voltage to drive a wheel at a speed and acceleration: kS to get moving, kV per in/s,
 * kA per in/s^2 (measure them with measureDriveFeedforward()).
 */
double feedforwardVolts(double speed, double accel, double kS, double kV, double kA) {
  double direction = speed > 0.01 ? 1 : (speed < -0.01 ? -1 : (accel > 0 ? 1 : (accel < 0 ? -1 : 0)));
  return kS * direction + kV * speed + kA * accel;
}
