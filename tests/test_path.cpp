// Native unit tests for RW-Template's motion profiles, paths, pure pursuit and RAMSETE math (src/path.cpp).
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "utils.h"
#include "path.h"

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

// Integrate a profile's speed and check it lands where its position says, within limits
static void checkProfile(const char* name, double distance, double max_speed, double max_accel, double v0, double v1) {
  MotionProfile p = makeProfile(distance, max_speed, max_accel, v0, v1);
  double position = 0, last_speed = 0, x = 0, v = 0, a = 0;
  profileAt(p, 0, x, last_speed, a);
  double dt = 0.0005;
  bool limits_ok = true;
  for (double t = dt; t <= p.total_time + 1e-9; t += dt) {
    profileAt(p, t, x, v, a);
    position += (last_speed + v) / 2 * dt;
    if (fabs(v) > p.max_speed + 1e-6) limits_ok = false;
    if (fabs(v - last_speed) / dt > p.max_accel * 1.001 + 1e-6) limits_ok = false;
    last_speed = v;
  }
  double end_x = 0, end_v = 0;
  profileAt(p, p.total_time + 1, end_x, end_v, a);
  CHECK(fabs(end_x - distance) < 1e-9, "%s: profile ends at %.4f, wanted %.4f", name, end_x, distance);
  CHECK(fabs(position - distance) < 0.02, "%s: integrated speed reaches %.4f, wanted %.4f", name, position, distance);
  CHECK(limits_ok, "%s: speed or acceleration went over the limit", name);
}

static void testProfiles() {
  checkProfile("trapezoid 48 in", 48, 60, 120, 0, 0);
  checkProfile("triangle 10 in", 10, 60, 120, 0, 0);
  checkProfile("backward 30 in", -30, 60, 120, 0, 0);
  checkProfile("chained 24 in", 24, 60, 120, 30, 30);
  checkProfile("can't stop in 2 in", 2, 60, 120, 60, 0);
  checkProfile("can't reach end speed", 3, 60, 120, 0, 60);
  checkProfile("turn 90 deg", 90, 300, 600, 0, 0);

  MotionProfile p = makeProfile(48, 60, 120);
  CHECK(fabs(p.peak_speed - 60) < 1e-9 && fabs(p.accel_time - 0.5) < 1e-9 && fabs(p.total_time - 1.3) < 1e-9,
        "48 in profile: peak %.3f accel %.3f total %.3f (want 60, 0.5, 1.3)", p.peak_speed, p.accel_time, p.total_time);
  MotionProfile tri = makeProfile(10, 60, 120);
  CHECK(fabs(tri.peak_speed - sqrt(1200.0)) < 1e-6 && tri.cruise_time == 0, "10 in profile should be a triangle");
  MotionProfile chained = makeProfile(2, 60, 120, 60, 0);
  CHECK(fabs(chained.end_speed - sqrt(3600.0 - 480.0)) < 1e-6, "too-short profile ends at %.3f", chained.end_speed);
}

static double nearestSample(const Path& path, double x, double y) {
  double best = 1e9;
  for (size_t i = 0; i < path.samples.size(); i++) best = fmin(best, hypot(path.samples[i].x - x, path.samples[i].y - y));
  return best;
}

// Text like path.jerryio's "LemLib v0.5" export of a 24" straight then a radius-24 right turn,
// with its default speeds (100, min 20, deceleration 127), a repeated end point and a ghost point.
// slow_last_in > 0 sets the last stretch to slow_speed, like a speed keyframe.
static std::string jerryioExport(double slow_last_in, double slow_speed, bool crlf, const char* indent) {
  std::vector<Waypoint> points;
  for (double s = 0; s < 24; s += 2) points.push_back(Waypoint(0, s, 100));
  double arc = 24 * M_PI / 2;
  for (double s = 0; s < arc; s += 2) points.push_back(Waypoint(24 - 24 * cos(s / 24), 24 + 24 * sin(s / 24), 100));
  points.push_back(Waypoint(24, 48, 100));
  points.push_back(Waypoint(24, 48, 100));
  size_t n = points.size();
  double to_end = 0;
  for (size_t i = n - 2; i > 0; i--) {
    to_end += hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    if (to_end <= slow_last_in) points[i - 1].speed = slow_speed;
  }
  points[n - 2].speed = 0;
  for (size_t i = n - 2; i > 0; i--) {
    double ramp = sqrt(points[i].speed * points[i].speed + 2 * 127 * hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y));
    points[i - 1].speed = fmax(fmin(points[i - 1].speed, ramp), 20);
  }
  const char* eol = crlf ? "\r\n" : "\n";
  std::string text;
  char line[128];
  for (size_t i = 0; i < n; i++) {
    snprintf(line, sizeof(line), "%s%.3f, %.3f, %.3f%s", indent, points[i].x, points[i].y, points[i].speed, eol);
    text += line;
  }
  snprintf(line, sizeof(line), "%s44.000, 48.000, 0%s%sendData%s%s127%s%s100%s%s200%s", indent, eol, indent, eol, indent, eol, indent, eol, indent, eol);
  text += line;
  snprintf(line, sizeof(line), "%s0, 0, 0, 8, 0, 16, 0, 24%s%s#PATH.JERRYIO-DATA {\"appVersion\":\"0.11.0\"}%s", indent, eol, indent, eol);
  text += line;
  return text;
}

static void testPaths() {
  // Straight line
  std::vector<Waypoint> line;
  line.push_back(Waypoint(0, 0));
  line.push_back(Waypoint(0, 48));
  Path straight = pathFromPoints(line);
  CHECK(fabs(straight.length - 48) < 0.05, "straight path length %.3f", straight.length);
  CHECK(straight.samples.size() >= 48 && straight.samples.size() <= 50, "straight path has %d samples", (int)straight.samples.size());
  bool flat = true;
  for (size_t i = 0; i < straight.samples.size(); i++) {
    if (fabs(straight.samples[i].curvature) > 1e-6 || fabs(remainder(straight.samples[i].heading_deg, 360.0)) > 1e-6) flat = false;
  }
  CHECK(flat, "straight path should have heading 0 and no curvature");
  CHECK(fabs(straight.samples.back().x) < 1e-9 && fabs(straight.samples.back().y - 48) < 1e-9, "straight path should end on the last point");

  // Clockwise quarter circle of radius 24, given as points every 15 degrees
  std::vector<Waypoint> arc;
  for (int deg = 0; deg <= 90; deg += 15) {
    double a = deg * M_PI / 180;
    arc.push_back(Waypoint(24 - 24 * cos(a), 24 * sin(a)));
  }
  Path circle = pathFromPoints(arc);
  for (size_t i = 0; i < arc.size(); i++) {
    CHECK(nearestSample(circle, arc[i].x, arc[i].y) < 0.6, "circle path should pass through waypoint %d", (int)i);
  }
  CHECK(fabs(circle.length - 24 * M_PI / 2) < 0.6, "circle path length %.3f (want %.3f)", circle.length, 24 * M_PI / 2);
  double mid_curvature = circle.samples[circle.samples.size() / 2].curvature;
  CHECK(fabs(mid_curvature - 1.0 / 24) < 0.006, "circle curvature %.4f (want %.4f, positive = right)", mid_curvature, 1.0 / 24);
  bool spacing_ok = true;
  for (size_t i = 1; i + 1 < circle.samples.size(); i++) {
    double gap = circle.samples[i].distance - circle.samples[i - 1].distance;
    if (gap < 0.95 || gap > 1.05) spacing_ok = false;
  }
  CHECK(spacing_ok, "samples should be about 1 in apart");

  // path.jerryio export (LemLib format)
  Path drawn = pathFromJerryIO("0, 0, 127\n0, 12, 127\n0, 24, 64\nendData\n200\n#PATH.JERRYIO-DATA {}\n");
  CHECK(fabs(drawn.length - 24) < 0.05, "jerryio path length %.3f", drawn.length);
  CHECK(fabs(drawn.samples.back().speed_limit - 64 / 127.0) < 1e-6, "jerryio speed column should become a speed limit");

  // A full jerryio export: 24" straight, then a quarter circle of radius 24 to the right
  double arc_length = 24 * M_PI / 2;
  Path exported = pathFromJerryIO(jerryioExport(0, 100, false, "").c_str());
  const PathSample& end = exported.samples.back();
  CHECK(fabs(end.x - 24) < 0.01 && fabs(end.y - 48) < 0.01, "jerryio path should end where it was drawn (24, 48), not at the ghost point (got %.2f, %.2f)", end.x, end.y);
  CHECK(fabs(exported.length - (24 + arc_length)) < 0.3, "jerryio path length %.2f (want %.2f)", exported.length, 24 + arc_length);
  CHECK(fabs(end.speed_limit - 100 / 127.0) < 1e-6, "jerryio's slow-down at the end should be lifted (end limit %.3f)", end.speed_limit * 127);
  double mid_arc = 0;
  for (size_t i = 0; i < exported.samples.size(); i++) {
    if (fabs(exported.samples[i].distance - (24 + arc_length / 2)) < 0.5) mid_arc = exported.samples[i].curvature;
  }
  CHECK(fabs(mid_arc - 1.0 / 24) < 0.003, "jerryio arc curvature %.4f (want %.4f)", mid_arc, 1.0 / 24);
  // Windows line endings and pasted indentation read the same
  Path pasted = pathFromJerryIO(jerryioExport(0, 100, true, "    ").c_str());
  CHECK(pasted.samples.size() == exported.samples.size() && fabs(pasted.length - exported.length) < 1e-9, "CRLF/indented export should read the same");
  // A slow stretch drawn at the end stays slow
  Path slow_end = pathFromJerryIO(jerryioExport(12, 40, false, "").c_str());
  CHECK(fabs(slow_end.samples.back().speed_limit - 40 / 127.0) < 1e-6, "a speed set at the end in jerryio should be kept (end limit %.3f)", slow_end.samples.back().speed_limit * 127);
  CHECK(fabs(slow_end.samples[5].speed_limit - 100 / 127.0) < 1e-6, "the start of the path should keep its speed (jerryio starts slowing about 33\" before a slow stretch)");

  // Speed plans
  std::vector<double> speeds = planPathSpeeds(straight, 60, 120, 120, 0, 0, true);
  bool accel_ok = true;
  double top = 0;
  for (size_t i = 1; i < speeds.size(); i++) {
    double gap = straight.samples[i].distance - straight.samples[i - 1].distance;
    if (fabs(speeds[i] * speeds[i] - speeds[i - 1] * speeds[i - 1]) > 2 * 120 * gap + 1e-6) accel_ok = false;
    top = fmax(top, speeds[i]);
  }
  CHECK(speeds.front() == 0 && speeds.back() == 0 && accel_ok && top <= 60 + 1e-9, "straight speed plan should ramp 0 to <=60 and back to 0");
  std::vector<double> times = planPathTimes(straight, speeds);
  MotionProfile same = makeProfile(48, 60, 120);
  CHECK(fabs(times.back() - same.total_time) < 0.12, "path takes %.3f s, a 48 in profile takes %.3f s", times.back(), same.total_time);
  std::vector<double> curve_speeds = planPathSpeeds(circle, 80, 120, 120, 0, 0, false);
  double curve_top = 0;
  for (size_t i = 3; i + 3 < curve_speeds.size(); i++) curve_top = fmax(curve_top, curve_speeds[i]);
  CHECK(curve_top < sqrt(120.0 * 24) * 1.1, "curve speed %.2f should stay under the sliding limit %.2f", curve_top, sqrt(120.0 * 24));
}

static void testFollowing() {
  std::vector<Waypoint> line;
  line.push_back(Waypoint(0, 0));
  line.push_back(Waypoint(0, 48));
  Path straight = pathFromPoints(line);
  int closest = closestSample(straight, 1, 10, 0);
  CHECK(closest == 10, "closest sample to (1, 10) is %d", closest);
  double tx = 0, ty = 0;
  bool found = lookaheadPoint(straight, 1, 10, 12, closest, tx, ty);
  CHECK(found && fabs(tx) < 1e-6 && fabs(ty - (10 + sqrt(143.0))) < 0.01, "lookahead point (%.3f, %.3f)", tx, ty);
  double k = curvatureToPoint(1, 10, 0, tx, ty);
  CHECK(k < 0 && fabs(k - (-2.0 / 144)) < 1e-4, "curvature toward the path should be left (negative): %.5f", k);
  found = lookaheadPoint(straight, 0, 40, 12, 40, tx, ty);
  CHECK(found && fabs(tx) < 1e-9 && fabs(ty - 52) < 1e-9, "near the end, the lookahead point should continue straight past the end (got %.3f, %.3f)", tx, ty);
  found = lookaheadPoint(straight, 1, 44, 12, 44, tx, ty);
  CHECK(found && fabs(tx) < 1e-9 && fabs((tx - 1) * (tx - 1) + (ty - 44) * (ty - 44) - 144) < 1e-6 && ty > 48, "off to the side near the end, the lookahead point should be on the line past the end (got %.3f, %.3f)", tx, ty);

  // RAMSETE: on track = no correction; right of the path = turn left; behind = speed up
  double speed = 0, turn = 0;
  ramsete(0, 10, 0, 0, 10, 0, 40, 0.2, speed, turn);
  CHECK(fabs(speed - 40) < 1e-9 && fabs(turn - 0.2) < 1e-9, "RAMSETE on track: speed %.3f turn %.3f", speed, turn);
  ramsete(2, 10, 0, 0, 10, 0, 40, 0, speed, turn);
  CHECK(turn < 0, "RAMSETE right of the path should turn left: %.4f", turn);
  ramsete(0, 6, 0, 0, 10, 0, 40, 0, speed, turn);
  CHECK(speed > 40, "RAMSETE behind the reference should speed up: %.3f", speed);
  ramsete(0, 10, 10, 0, 10, 0, 40, 0, speed, turn);
  CHECK(turn < 0, "RAMSETE turned right of the path heading should turn back left: %.4f", turn);
  // Facing -Y while reversing toward +Y, with the path 2 in to the robot's left (+X): the back
  // leads, so it swings toward the path by turning clockwise
  ramsete(-2, 10, 180, 0, 10, 180, -40, 0, speed, turn);
  CHECK(turn > 0, "RAMSETE driving backward with the path on its left should turn clockwise (turn %.4f)", turn);

  double v = feedforwardVolts(50, 0, 0.5, 0.18, 0.03);
  CHECK(fabs(v - (0.5 + 9)) < 1e-9, "feedforward volts %.3f", v);
  CHECK(feedforwardVolts(-50, -100, 0.5, 0.18, 0.03) < 0, "backward feedforward should be negative");
}

int main() {
  testProfiles();
  testPaths();
  testFollowing();
  if (failures == 0) printf("ALL PATH TESTS PASSED\n");
  else printf("%d FAILURES\n", failures);
  return failures != 0;
}
