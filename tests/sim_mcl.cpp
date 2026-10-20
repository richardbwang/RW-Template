// Field simulation for RW-Template's MCL (src/mcl.cpp + src/utils.cpp, built with the host compiler).
// A robot drives a skills-style route with realistic odometry error; four distance sensors read the
// walls with noise, delay, and Override field elements (corner loaders, wall stacks) in the way.
// Compares odometry alone against odometry + MCL, using the same glue logic as runMcl().
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <ratio>
#include <vector>
#include "utils.h"
#include "mcl.h"

const double HALF = 70.25;
const double DT = 0.01;

// ---------------------------------------------------------------------------
// Simple seeded random numbers for the simulated world (separate from the filter's)
// ---------------------------------------------------------------------------
static unsigned long long world_rng = 88172645463325252ull;
static double worldUniform() {
  world_rng ^= world_rng << 13; world_rng ^= world_rng >> 7; world_rng ^= world_rng << 17;
  return (world_rng >> 11) * (1.0 / 9007199254740992.0);
}
static double worldGaussian(double s) {
  double u1 = fmax(worldUniform(), 1e-12), u2 = worldUniform();
  return s * sqrt(-2 * log(u1)) * cos(2 * M_PI * u2);
}

// ---------------------------------------------------------------------------
// Field: walls plus boxes for Override's corner loaders and wall stacks
// ---------------------------------------------------------------------------
struct Box { double x0, y0, x1, y1; };
static std::vector<Box> obstacles;

static void addBox(double cx, double cy, double w, double h) {
  obstacles.push_back({cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2});
}

// Distance along a ray to the first wall or obstacle
static double rayCast(double sx, double sy, double bx, double by) {
  bool hx; double gap;
  double best = beamToWall(sx, sy, bx, by, HALF, hx, gap);
  for (const Box& b : obstacles) {
    double tmin = -1e9, tmax = 1e9;
    if (fabs(bx) < 1e-12) { if (sx < b.x0 || sx > b.x1) continue; }
    else { double t1 = (b.x0 - sx) / bx, t2 = (b.x1 - sx) / bx; tmin = fmax(tmin, fmin(t1, t2)); tmax = fmin(tmax, fmax(t1, t2)); }
    if (fabs(by) < 1e-12) { if (sy < b.y0 || sy > b.y1) continue; }
    else { double t1 = (b.y0 - sy) / by, t2 = (b.y1 - sy) / by; tmin = fmax(tmin, fmin(t1, t2)); tmax = fmin(tmax, fmax(t1, t2)); }
    if (tmax >= tmin && tmin > 0 && tmin < best) best = tmin;
  }
  return best;
}

// ---------------------------------------------------------------------------
// Robot, sensors, and scenario settings
// ---------------------------------------------------------------------------
struct Sensor { double s_x, s_y, angle; };
static const Sensor SENSORS[4] = {{0, 6.0, 0}, {6.5, 0, 90}, {0, -6.0, 180}, {-6.5, 0, 270}};

struct Scenario {
  const char* name;
  bool stacks;           // Override wall stacks in place
  bool loaders;          // Override corner loaders
  double start_error;    // Initial pose error given to the robot (inches)
  double push_at;        // Time the robot gets shoved sideways without odometry noticing (s, <0 = never)
  double push_in;        // How far it gets shoved
  bool use_mcl;
  double corner_margin;  // distance_reset_corner_margin
  int particles;
  unsigned seed;
  int delay_ticks = 4;    // true sensor delay in 10 ms ticks
  int assumed_delay_ticks = 4;  // delay the robot compensates for (distance_sensor_delay_msec / 10)
};

struct Result { double rms, max, final_err; int corrections; };


static Result runScenario(const Scenario& sc, bool print_trace) {
  world_rng = 88172645463325252ull ^ (sc.seed * 2654435761ull);
  mclSeed(sc.seed * 7919u + 1u);
  obstacles.clear();
  if (sc.loaders) {
    // Loaders: about 2-4 in into the field, ~11.5 in from the corners, on the two alliance walls
    for (int sx = -1; sx <= 1; sx += 2)
      for (int sy = -1; sy <= 1; sy += 2) addBox(sx * 68.5, sy * 58.7, 4, 5);
  }
  if (sc.stacks) {
    // Cup+Pin stacks against every wall, about 24 in either side of each wall's center
    for (int s = -1; s <= 1; s += 2)
      for (int t = -1; t <= 1; t += 2) { addBox(s * 23.6, t * 68.7, 3.2, 3.2); addBox(t * 68.7, s * 23.6, 3.2, 3.2); }
  }

  // Route: a skills-style loop that visits walls, corners, and the middle, twice
  const double route[][2] = {{-48, -24}, {-24, -24}, {-24, 24}, {24, 24}, {48, 48}, {48, -48}, {0, -48},
                             {-48, -48}, {-58, 0}, {0, 0}, {56, 0}, {0, 56}, {-48, 40}, {-48, -58}};
  const int route_len = sizeof(route) / sizeof(route[0]);

  // True state
  double tx = -48, ty = -58, th = 0;
  // Odometry (what the robot believes), with and without MCL
  double fused_x = tx + sc.start_error, fused_y = ty, odom_x = fused_x, odom_y = fused_y;
  double imu_drift = 0, prev_imu = th;
  // Raw odometry movement accumulators for MCL (never touched by corrections)
  double motion_x = 0, motion_y = 0, last_motion_x = 0, last_motion_y = 0, last_mcl_heading = th;
  double moved_since_correct = 0;
  // Sensor delay: readings come from where the robot was ~40 ms ago, refreshed every ~33 ms
  std::vector<double> hist_x, hist_y, hist_h;
  double latest[4] = {-1, -1, -1, -1};
  double last_reading[4] = {-1, -1, -1, -1};  // Same repeat check as runMcl()
  double next_sensor = 0;

  double hist_motion_x[16] = {0}, hist_motion_y[16] = {0}, hist_heading[16] = {0};
  int tick = 0;
  if (sc.use_mcl) mclInit(sc.particles, fused_x, fused_y, 1.0);

  double sum_sq = 0, max_err = 0; int samples = 0, corrections = 0;
  int target = 0, laps = 0;
  double speed_ips = 0;
  for (double t = 0; t < 70 && laps < 2; t += DT) {
    // --- Drive toward the next waypoint (controller uses the TRUE pose; we're testing localization) ---
    double gx = route[target][0], gy = route[target][1];
    double dist = hypot(gx - tx, gy - ty);
    if (dist < 1.0) { target = (target + 1) % route_len; if (target == 0) laps++; gx = route[target][0]; gy = route[target][1]; dist = hypot(gx - tx, gy - ty); }
    double want = atan2(gx - tx, gy - ty);
    double err = remainder(want - th, 2 * M_PI);
    double v, w;
    if (fabs(err) > 0.25) { v = 0; w = (err > 0 ? 1 : -1) * fmin(5.0, 8 * fabs(err)); }  // turn in place
    else { v = fmin(45.0, 6 * dist + 5); w = 4 * err; }                                    // drive with steering
    speed_ips = fabs(v);
    double turn_rate_dps = fabs(w) * 180 / M_PI;

    // --- True motion, including sideways slip in turns that drive encoders can't see ---
    double ds = v * DT, dth = w * DT;
    double avg = th + dth / 2;
    double slip = 0.03 * v * w * DT;  // inches sideways
    tx += ds * sin(avg) + slip * cos(avg);
    ty += ds * cos(avg) - slip * sin(avg);
    th += dth;
    if (sc.push_at >= 0 && fabs(t - sc.push_at) < DT / 2) { tx += sc.push_in * cos(th); ty -= sc.push_in * sin(th); }

    // --- Odometry: drive encoders with scale error and noise, IMU with slow drift ---
    double track = 12.0;
    double dl = (ds + (track / 2) * dth) * 1.02 + worldGaussian(0.01 * fabs(ds) + 0.0005);
    double dr = (ds - (track / 2) * dth) * 0.99 + worldGaussian(0.01 * fabs(ds) + 0.0005);
    imu_drift += (0.4 / 60.0) * DT * M_PI / 180;  // 0.4 deg per minute
    double imu = th + imu_drift + worldGaussian(0.0003);
    double dx, dy;
    odomStep(prev_imu * 180 / M_PI, imu * 180 / M_PI, dl, dr, 0, 0, false, false, track, 0, 0, dx, dy);
    prev_imu = imu;
    odom_x += dx; odom_y += dy;
    fused_x += dx; fused_y += dy;
    motion_x += dx; motion_y += dy;

    // --- Sensors ---
    hist_x.push_back(tx); hist_y.push_back(ty); hist_h.push_back(th);
    if (t >= next_sensor) {
      next_sensor += 0.033;
      size_t lag = hist_x.size() > (size_t)sc.delay_ticks ? hist_x.size() - 1 - sc.delay_ticks : 0;
      for (int s = 0; s < 4; s++) {
        double ox, oy;
        localToGlobal(SENSORS[s].s_x, SENSORS[s].s_y, hist_h[lag], ox, oy);
        double b = hist_h[lag] + SENSORS[s].angle * M_PI / 180;
        double d = rayCast(hist_x[lag] + ox, hist_y[lag] + oy, sin(b), cos(b));
        d += worldGaussian(0.3 + 0.01 * d);
        latest[s] = (d > 78 || d < 0.8) ? -1 : d;
      }
    }

    // --- MCL, same steps as runMcl(): history every 10 ms, filter every 30 ms ---
    if (sc.use_mcl) {
      double heading_now = imu * 180 / M_PI;
      hist_motion_x[tick % 16] = motion_x; hist_motion_y[tick % 16] = motion_y; hist_heading[tick % 16] = heading_now;
      if (tick % 3 == 0) {
        double mdx = motion_x - last_motion_x, mdy = motion_y - last_motion_y;
        last_motion_x = motion_x; last_motion_y = motion_y;
        double dh = heading_now - last_mcl_heading;
        last_mcl_heading = heading_now;
        mclPredict(mdx, mdy, dh);
        moved_since_correct += hypot(mdx, mdy) + fabs(dh) * 0.05;

        // Where the robot was when the sensors took their readings
        int back = tick >= sc.assumed_delay_ticks ? (tick - sc.assumed_delay_ticks) % 16 : 0;
        double shift_x = motion_x - hist_motion_x[back], shift_y = motion_y - hist_motion_y[back];
        double heading_then = hist_heading[back];

        MclReading readings[4]; int n = 0;
        for (int s = 0; s < 4; s++) {
          bool repeat = latest[s] == last_reading[s];
          last_reading[s] = latest[s];
          if (repeat || latest[s] < 1.5 || latest[s] > 60) continue;
          double unused;
          if (distanceResetPosition(fused_x - shift_x, fused_y - shift_y, heading_then, SENSORS[s].s_x, SENSORS[s].s_y,
                                    SENSORS[s].angle, latest[s], HALF, 30, sc.corner_margin, unused) == 0) continue;
          readings[n++] = {SENSORS[s].s_x, SENSORS[s].s_y, SENSORS[s].angle, latest[s]};
        }
        if (n > 0 && moved_since_correct > 0.25 && turn_rate_dps < 200) {
          if (mclCorrect(heading_then, readings, n, HALF, speed_ips * 0.01, shift_x, shift_y)) corrections++;
          moved_since_correct = 0;
        }
        double ex, ey, spread_x, spread_y;
        mclEstimate(ex, ey, spread_x, spread_y);
        mclNudge(fused_x, fused_y, ex, ey, spread_x, spread_y);
      }
    }
    tick++;

    double e = hypot(fused_x - tx, fused_y - ty);
    sum_sq += e * e; samples++;
    if (t > 1.0) max_err = fmax(max_err, e);  // ignore the first second (initial-error scenarios)
    if (print_trace && samples % 500 == 0)
      printf("  t=%5.1f true (%6.1f,%6.1f) odom err %5.2f  fused err %5.2f\n", t, tx, ty, hypot(odom_x - tx, odom_y - ty), e);
  }
  return {sqrt(sum_sq / samples), max_err, hypot(fused_x - tx, fused_y - ty), corrections};
}

int main(int argc, char** argv) {
  bool trace = argc > 1 && strcmp(argv[1], "-v") == 0;
  Scenario base = {"walls only", false, false, 0, -1, 0, true, 0, 200, 1};
  std::vector<Scenario> list;
  Scenario s;
  s = base; list.push_back(s);
  s = base; s.name = "Override stacks + loaders"; s.stacks = s.loaders = true; list.push_back(s);
  s = base; s.name = "Override, corner margin 18"; s.stacks = s.loaders = true; s.corner_margin = 18; list.push_back(s);
  s = base; s.name = "starts 3 in off"; s.stacks = s.loaders = true; s.corner_margin = 18; s.start_error = 3; list.push_back(s);
  s = base; s.name = "shoved 4 in at 20 s"; s.stacks = s.loaders = true; s.corner_margin = 18; s.push_at = 20; s.push_in = 4; list.push_back(s);
  s = base; s.name = "Override, margin 18, no delay"; s.stacks = s.loaders = true; s.corner_margin = 18; s.delay_ticks = 0; list.push_back(s);
  s = base; s.name = "Override, delay 60 ms (assume 40)"; s.stacks = s.loaders = true; s.corner_margin = 18; s.delay_ticks = 6; list.push_back(s);
  s = base; s.name = "Override, delay 20 ms (assume 40)"; s.stacks = s.loaders = true; s.corner_margin = 18; s.delay_ticks = 2; list.push_back(s);
  s = base; s.name = "100 particles"; s.stacks = s.loaders = true; s.corner_margin = 18; s.particles = 100; list.push_back(s);

  printf("%-30s %26s %26s\n", "scenario (5 seeds avg)", "odometry only rms/max", "with MCL rms/max (final)");
  int failures = 0;
  for (const Scenario& sc : list) {
    double o_rms = 0, o_max = 0, m_rms = 0, m_max = 0, m_final = 0;
    for (unsigned seed = 1; seed <= 5; seed++) {
      Scenario a = sc; a.seed = seed; a.use_mcl = false;
      Scenario b = sc; b.seed = seed; b.use_mcl = true;
      Result ro = runScenario(a, false), rm = runScenario(b, trace && seed == 1);
      o_rms += ro.rms / 5; o_max += ro.max / 5; m_rms += rm.rms / 5; m_max += rm.max / 5; m_final += rm.final_err / 5;
    }
    printf("%-30s %13.2f / %-10.2f %13.2f / %-5.2f (%.2f)\n", sc.name, o_rms, o_max, m_rms, m_max, m_final);
    if (!(m_rms < o_rms * 0.6)) { printf("  FAIL: MCL should cut the rms error well below odometry\n"); failures++; }
    if (!(m_max < 4.0)) { printf("  FAIL: MCL max error too large\n"); failures++; }
  }

  // Timing: one correction with 200 particles and 4 readings
  mclInit(200, 0, 0, 1);
  MclReading r[4] = {{0, 6, 0, 64}, {6.5, 0, 90, 63}, {0, -6, 180, 64}, {-6.5, 0, 270, 63}};
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 2000; i++) { mclPredict(0.5, 0.2, 1); mclCorrect(10, r, 4, HALF, 0.5); }
  double us = std::chrono::duration<double, std::micro>(std::chrono::high_resolution_clock::now() - start).count() / 2000;
  printf("predict+correct, 200 particles x 4 sensors: %.1f us on this computer\n", us);

  printf(failures ? "%d FAILURES\n" : "ALL SIM CHECKS PASSED\n", failures);
  return failures != 0;
}
