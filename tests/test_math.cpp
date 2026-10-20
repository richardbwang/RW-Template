// Native unit tests for RW-Template's pure math (src/utils.cpp), built with the host compiler.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "utils.h"

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

// ---------------------------------------------------------------------------
// Odometry: drive a known path in small steps, feed odomStep the wheel travel
// a real robot would measure, and compare the integrated pose with the truth.
// ---------------------------------------------------------------------------
struct Geometry { double track, h_off, v_off; };

// Integrates a motion with constant forward speed v (in/step) and turn rate w (rad/step).
// Returns the final odometry pose and the true pose.
static void runPath(const Geometry& g, bool use_h, bool use_v, double v, double w, int steps,
                    double& ox, double& oy, double& tx, double& ty, double start_heading_deg) {
  double heading = start_heading_deg * M_PI / 180.0;
  ox = oy = tx = ty = 0;
  for (int i = 0; i < steps; i++) {
    // True motion: exact arc of length v and heading change w
    double new_heading = heading + w;
    double chord_fwd = (fabs(w) < 1e-12) ? v : 2.0 * sin(w / 2.0) * (v / w);
    double avg = heading + w / 2.0;
    tx += chord_fwd * sin(avg);
    ty += chord_fwd * cos(avg);
    // Wheel travel for this arc (clockwise positive heading): a wheel at lateral offset s
    // (positive right) rolls v - s*w; a sideways wheel at offset b behind center rolls -b*w.
    double dl = v - (-g.track / 2) * w;
    double dr = v - (g.track / 2) * w;
    double dv = v - g.v_off * w;
    double dh = -g.h_off * w;
    double dx, dy;
    odomStep(heading * 180 / M_PI, new_heading * 180 / M_PI, dl, dr, dh, dv, use_h, use_v,
             g.track, g.h_off, g.v_off, dx, dy);
    ox += dx;
    oy += dy;
    heading = new_heading;
  }
}

static void testOdometry() {
  Geometry g = {12.3, 2.7, -0.4};
  const char* names[] = {"no trackers", "horizontal only", "vertical only", "both trackers"};
  bool cfg[4][2] = {{false, false}, {true, false}, {false, true}, {true, true}};
  struct Path { const char* name; double v, w; int steps; double start; } paths[] = {
    {"straight", 0.24, 0.0, 100, 0},
    {"straight at 37 deg", 0.24, 0.0, 100, 37},
    {"turn in place 90", 0.0, (M_PI / 2) / 100, 100, 0},
    {"turn in place -270 fast", 0.0, (-1.5 * M_PI) / 40, 40, 15},
    {"arc R24 quarter", 24 * (M_PI / 2) / 200, (M_PI / 2) / 200, 200, 0},
    {"arc R24 quarter fast", 24 * (M_PI / 2) / 40, (M_PI / 2) / 40, 40, 0},
    {"reverse arc", -0.3, -0.01, 150, 200},
  };
  for (int c = 0; c < 4; c++) {
    for (const Path& p : paths) {
      double ox, oy, tx, ty;
      runPath(g, cfg[c][0], cfg[c][1], p.v, p.w, p.steps, ox, oy, tx, ty, p.start);
      double err = hypot(ox - tx, oy - ty);
      CHECK(err < 1e-6, "odometry %s / %s: odom (%.4f, %.4f) truth (%.4f, %.4f) err %.6f",
            names[c], p.name, ox, oy, tx, ty, err);
    }
  }
}

// ---------------------------------------------------------------------------
// Distance resets: put the robot somewhere, ray-trace the true reading by
// marching along the beam, and check the reset recovers the true coordinate.
// ---------------------------------------------------------------------------
// A small random number generator that gives the same numbers on every computer
// (rand() gives different numbers on macOS and Linux)
static unsigned long long random_state = 42;
static unsigned int nextRandom() {
  random_state = random_state * 6364136223846793005ULL + 1442695040888963407ULL;
  return (unsigned int)(random_state >> 33);
}
static double rand01() { return nextRandom() / 2147483647.0; }

static double marchToWall(double sx, double sy, double bx, double by, double half, bool& hit_x) {
  // Independent check: step along the beam until it leaves the field
  double t = 0;
  while (fabs(sx + t * bx) < half && fabs(sy + t * by) < half) t += 0.001;
  // Back up to the exact crossing with bisection
  double lo = t - 0.001, hi = t;
  for (int i = 0; i < 60; i++) {
    double mid = (lo + hi) / 2;
    if (fabs(sx + mid * bx) < half && fabs(sy + mid * by) < half) lo = mid; else hi = mid;
  }
  hit_x = fabs(sx + hi * bx) >= half - 1e-6 && fabs(sy + hi * by) < half - 1e-6;
  return hi;
}

static void testDistanceReset() {
  const double half = 70.25;
  random_state = 42;
  int applied = 0, skipped = 0;
  for (int trial = 0; trial < 4000; trial++) {
    double rx = (rand01() * 2 - 1) * 55, ry = (rand01() * 2 - 1) * 55;
    double heading = rand01() * 720 - 360;
    double s_x = (rand01() * 2 - 1) * 6, s_y = (rand01() * 2 - 1) * 6;
    double mount = (nextRandom() % 4) * 90.0;
    double beam = (heading + mount) * M_PI / 180;
    double bx = sin(beam), by = cos(beam);
    double ox, oy;
    localToGlobal(s_x, s_y, heading * M_PI / 180, ox, oy);
    bool truth_hits_x = false;
    double reading = marchToWall(rx + ox, ry + oy, bx, by, half, truth_hits_x);
    if (reading > 78) continue;

    // Give the reset a slightly wrong starting estimate, like real odometry drift
    double est_x = rx + (rand01() * 2 - 1) * 1.5, est_y = ry + (rand01() * 2 - 1) * 1.5;
    double new_pos = 0;
    int axis = distanceResetPosition(est_x, est_y, heading, s_x, s_y, mount, reading, half, 40, 0, new_pos);
    if (axis == 0) { skipped++; continue; }
    applied++;
    CHECK((axis == 1) == truth_hits_x, "reset picked the wrong wall at (%.1f, %.1f) heading %.1f mount %.0f",
          rx, ry, heading, mount);
    double truth = axis == 1 ? rx : ry;
    CHECK(fabs(new_pos - truth) < 1e-3, "reset value %.4f vs truth %.4f (axis %d)", new_pos, truth, axis);
    // Never accept steeper than the limit
    double facing = axis == 1 ? fabs(bx) : fabs(by);
    CHECK(facing >= cos(40 * M_PI / 180) - 1e-9, "accepted a steep reading (cos %.3f)", facing);
  }
  printf("distance reset: %d applied, %d skipped (steep/corner/out of range)\n", applied, skipped);
  CHECK(applied > 1000, "too few resets applied: %d", applied);

  // Straight-on cases from the old template's examples
  double p = 0;
  CHECK(distanceResetPosition(0, 0, 0, 0, 6.5, 0, 63.75, half, 40, 0, p) == 2 && fabs(p - 0) < 1e-9,
        "front sensor facing north wall should give y = 0, got %f", p);
  CHECK(distanceResetPosition(10, -20, 90, 0, -5, 180, 75.25, half, 40, 0, p) == 1 && fabs(p - 10) < 1e-9,
        "back sensor facing west wall should give x = 10, got %f", p);
  // Pointing into the corner is skipped
  CHECK(distanceResetPosition(50, 50, 45, 0, 0, 0, 28.6, half, 60, 0, p) == 0, "corner reading should be skipped");
  // Corner margin skips hits near the corner
  CHECK(distanceResetPosition(60, 0, 270, 0, 0, 0, 130.5, half, 40, 18, p) == 1, "mid-wall hit should pass the margin");
  CHECK(distanceResetPosition(60, 60, 270, 0, 0, 0, 130.5, half, 40, 18, p) == 0, "hit 10 in from the corner should be skipped");
}

int main() {
  testOdometry();
  testDistanceReset();
  if (failures == 0) printf("ALL TESTS PASSED\n");
  else printf("%d FAILURES\n", failures);
  return failures != 0;
}
