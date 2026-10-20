// Fits a robot profile for the simulator from a drive log recorded on the robot by calibrateDrive().
//   sh sim/fit-profile.sh rw_drive_log.csv    writes sim/robot-profile.txt
//   sh sim/fit-profile.sh --self-test         checks the fitter on a made-up robot
// It replays the logged voltages through the simulator's drive physics (drive-model.h) and
// adjusts the physics until the simulated encoders, IMU and tracking wheels match the log.
#include "drive-model.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <random>
#include <string>
#include <vector>

using namespace rwsim;

namespace {

const double PI = 3.14159265358979323846;

struct LogRow { double t; int mode; double left_volts, right_volts, left_deg, right_deg, imu_deg, horizontal_deg, vertical_deg; };

struct DriveLog {
  std::vector<LogRow> rows;
  double wheel_distance_in = 7, track = 12;
  bool horizontal = false, vertical = false;
  double horizontal_offset = 0, vertical_offset = 0, horizontal_diameter = 2, vertical_diameter = 2;
};

// Speeds over time, from positions (the same smoothing for measured and simulated data)
struct Signals { std::vector<double> left, right, turn, forward, sideways; };

bool readLog(const char* path, DriveLog& log) {
  FILE* file = std::fopen(path, "r");
  if (!file) return false;
  char line[512];
  while (std::fgets(line, sizeof(line), file)) {
    if (line[0] == '#') {
      int horizontal = 0, vertical = 0;
      const char* at = std::strstr(line, "wheel_distance_in=");
      if (at) std::sscanf(at, "wheel_distance_in=%lf track=%lf horizontal=%d vertical=%d horizontal_offset=%lf vertical_offset=%lf horizontal_diameter=%lf vertical_diameter=%lf",
                          &log.wheel_distance_in, &log.track, &horizontal, &vertical, &log.horizontal_offset, &log.vertical_offset,
                          &log.horizontal_diameter, &log.vertical_diameter);
      log.horizontal = horizontal != 0;
      log.vertical = vertical != 0;
      continue;
    }
    LogRow r;
    if (std::sscanf(line, "%lf,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &r.t, &r.mode, &r.left_volts, &r.right_volts, &r.left_deg,
                    &r.right_deg, &r.imu_deg, &r.horizontal_deg, &r.vertical_deg) == 9) {
      log.rows.push_back(r);
    }
  }
  std::fclose(file);
  return log.rows.size() > 50;
}

Signals signalsFrom(const DriveLog& log, const std::vector<double>& left_deg, const std::vector<double>& right_deg,
                    const std::vector<double>& imu_deg, const std::vector<double>& horizontal_deg, const std::vector<double>& vertical_deg) {
  Signals s;
  size_t n = log.rows.size();
  for (size_t i = 2; i + 2 < n; i++) {
    double dt = log.rows[i + 2].t - log.rows[i - 2].t;
    if (dt <= 0) dt = 0.04;
    double turn = (imu_deg[i + 2] - imu_deg[i - 2]) / dt;
    s.left.push_back((left_deg[i + 2] - left_deg[i - 2]) / 360.0 * log.wheel_distance_in / dt);
    s.right.push_back((right_deg[i + 2] - right_deg[i - 2]) / 360.0 * log.wheel_distance_in / dt);
    s.turn.push_back(turn);
    // A tracking wheel off center also rolls when the robot turns; take that out
    double turn_rad = turn * PI / 180.0;
    s.forward.push_back((vertical_deg[i + 2] - vertical_deg[i - 2]) / 360.0 * PI * log.vertical_diameter / dt + log.vertical_offset * turn_rad);
    s.sideways.push_back((horizontal_deg[i + 2] - horizontal_deg[i - 2]) / 360.0 * PI * log.horizontal_diameter / dt + log.horizontal_offset * turn_rad);
  }
  return s;
}

Signals measured(const DriveLog& log) {
  std::vector<double> l, r, imu, h, v;
  for (const LogRow& row : log.rows) {
    l.push_back(row.left_deg); r.push_back(row.right_deg); imu.push_back(row.imu_deg);
    h.push_back(row.horizontal_deg); v.push_back(row.vertical_deg);
  }
  return signalsFrom(log, l, r, imu, h, v);
}

// Replays the logged commands through a drive model
Signals predicted(const DriveModel& m, const DriveLog& log) {
  size_t n = log.rows.size();
  std::vector<double> l(n), r(n), imu(n), h(n), v(n);
  DriveSide left, right;
  double left_deg = 0, right_deg = 0, heading = 0, horizontal = 0, vertical = 0;
  for (size_t i = 0; i < n; i++) {
    l[i] = left_deg; r[i] = right_deg; imu[i] = heading; h[i] = horizontal; v[i] = vertical;
    if (i + 1 == n) break;
    double span = log.rows[i + 1].t - log.rows[i].t;
    int steps = std::max(1, (int)std::lround(span / 0.001));
    double dt = span / steps;
    const LogRow& row = log.rows[i];
    for (int k = 0; k < steps; k++) {
      stepDriveSide(m, left, row.mode, row.left_volts, dt);
      stepDriveSide(m, right, row.mode, row.right_volts, dt);
      double speed = 0, turn = 0, skid = 0;
      driveMotion(m, left, right, log.track, speed, turn, skid);
      left_deg += left.wheel / log.wheel_distance_in * 360.0 * dt;
      right_deg += right.wheel / log.wheel_distance_in * 360.0 * dt;
      heading += turn * 180.0 / PI * dt;
      vertical += (speed - log.vertical_offset * turn) * dt / (PI * log.vertical_diameter) * 360.0;
      horizontal += (skid - log.horizontal_offset * turn) * dt / (PI * log.horizontal_diameter) * 360.0;
    }
  }
  return signalsFrom(log, l, r, imu, h, v);
}

double squaredError(const std::vector<double>& a, const std::vector<double>& b) {
  double total = 0;
  for (size_t i = 0; i < a.size() && i < b.size(); i++) total += (a[i] - b[i]) * (a[i] - b[i]);
  return total;
}

double rms(const std::vector<double>& a, const std::vector<double>& b) { return std::sqrt(squaredError(a, b) / std::max<size_t>(1, a.size())); }

// Nelder-Mead minimizer (no derivatives needed)
std::vector<double> minimize(const std::function<double(const std::vector<double>&)>& f, std::vector<double> start, std::vector<double> steps) {
  for (int restart = 0; restart < 3; restart++) {
    size_t n = start.size();
    std::vector<std::vector<double>> simplex(n + 1, start);
    for (size_t i = 0; i < n; i++) simplex[i + 1][i] += steps[i];
    std::vector<double> value(n + 1);
    for (size_t i = 0; i <= n; i++) value[i] = f(simplex[i]);
    for (int iteration = 0; iteration < 400 * (int)n; iteration++) {
      std::vector<size_t> order(n + 1);
      for (size_t i = 0; i <= n; i++) order[i] = i;
      std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return value[a] < value[b]; });
      std::vector<std::vector<double>> s2(n + 1);
      std::vector<double> v2(n + 1);
      for (size_t i = 0; i <= n; i++) { s2[i] = simplex[order[i]]; v2[i] = value[order[i]]; }
      simplex = s2; value = v2;
      if (std::fabs(value[n] - value[0]) <= 1e-10 * (std::fabs(value[0]) + 1e-12)) break;
      std::vector<double> centroid(n, 0.0);
      for (size_t i = 0; i < n; i++) for (size_t j = 0; j < n; j++) centroid[j] += simplex[i][j] / n;
      auto blend = [&](double t) { std::vector<double> x(n); for (size_t j = 0; j < n; j++) x[j] = centroid[j] + t * (simplex[n][j] - centroid[j]); return x; };
      std::vector<double> reflected = blend(-1);
      double fr = f(reflected);
      if (fr < value[0]) {
        std::vector<double> expanded = blend(-2);
        double fe = f(expanded);
        if (fe < fr) { simplex[n] = expanded; value[n] = fe; } else { simplex[n] = reflected; value[n] = fr; }
      } else if (fr < value[n - 1]) {
        simplex[n] = reflected; value[n] = fr;
      } else {
        std::vector<double> contracted = blend(fr < value[n] ? -0.5 : 0.5);
        double fc = f(contracted);
        if (fc < std::min(fr, value[n])) {
          simplex[n] = contracted; value[n] = fc;
        } else {
          for (size_t i = 1; i <= n; i++) {
            for (size_t j = 0; j < n; j++) simplex[i][j] = simplex[0][j] + 0.5 * (simplex[i][j] - simplex[0][j]);
            value[i] = f(simplex[i]);
          }
        }
      }
    }
    size_t best = std::min_element(value.begin(), value.end()) - value.begin();
    start = simplex[best];
    for (double& step : steps) step *= 0.3;
  }
  return start;
}

// Stage 1 parameters: friction, speed per volt, lag, current limit, braking, coasting
void wheelParams(const std::vector<double>& x, DriveModel& m) {
  m.kS = clampValue(x[0], 0, 3);
  m.kV = clampValue(std::exp(x[1]), 0.02, 1.0);
  m.kA = clampValue(std::exp(x[2]), 0.001, 0.3);
  m.max_accel = clampValue(std::exp(x[3]), 30, 3000);
  m.brake_decel = clampValue(std::exp(x[4]), 30, 5000);
  m.coast_decel = clampValue(std::exp(x[5]), 2, 1000);
}

DriveModel fitProfile(const DriveLog& log, bool verbose) {
  Signals meas = measured(log);
  DriveModel model;

  // Starting guess from the slow ramps: volts = kS + kV * speed
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  int count = 0;
  for (size_t i = 2; i + 2 < log.rows.size(); i++) {
    const LogRow& row = log.rows[i];
    double speed = (meas.left[i - 2] + meas.right[i - 2]) / 2;
    if (row.mode == 0 && std::fabs(row.left_volts - row.right_volts) < 1e-6 && std::fabs(row.left_volts) < 6.5 && std::fabs(speed) > 4) {
      double x = std::fabs(speed), y = std::fabs(row.left_volts);
      sx += x; sy += y; sxx += x * x; sxy += x * y; count++;
    }
  }
  if (count > 10) {
    double kV = (count * sxy - sx * sy) / std::max(count * sxx - sx * sx, 1e-9);
    if (kV > 0.02 && kV < 1) { model.kV = kV; model.kS = clampValue((sy - kV * sx) / count, 0, 3); }
  }
  model.kA = model.kV * 0.13;

  // Stage 1: wheel encoders
  auto stage1 = [&](const std::vector<double>& x) {
    DriveModel m = model;
    wheelParams(x, m);
    Signals p = predicted(m, log);
    return squaredError(p.left, meas.left) + squaredError(p.right, meas.right);
  };
  std::vector<double> x1 = {model.kS, std::log(model.kV), std::log(model.kA), std::log(model.max_accel), std::log(model.brake_decel), std::log(model.coast_decel)};
  x1 = minimize(stage1, x1, std::vector<double>(6, 0.3));
  wheelParams(x1, model);

  // Stage 2: turning scrub from the IMU
  auto turn_cost = [&](const std::vector<double>& x) {
    DriveModel m = model;
    m.turn_scrub = clampValue(x[0], 0.3, 1.1);
    return squaredError(predicted(m, log).turn, meas.turn);
  };
  model.turn_scrub = clampValue(minimize(turn_cost, {model.turn_scrub}, {0.05})[0], 0.3, 1.1);

  // Stage 3 (needs both tracking wheels): slip and grip from the forward wheel, skid from the sideways one
  bool trackers = log.horizontal && log.vertical;
  if (trackers) {
    auto forward_cost = [&](const std::vector<double>& x) {
      DriveModel m = model;
      m.tread_slip = clampValue(x[0], 0.7, 1.05);
      m.traction_accel = clampValue(std::exp(x[1]), 50, 5000);
      return squaredError(predicted(m, log).forward, meas.forward);
    };
    std::vector<double> x3 = minimize(forward_cost, {model.tread_slip, std::log(model.traction_accel)}, {0.02, 0.3});
    model.tread_slip = clampValue(x3[0], 0.7, 1.05);
    model.traction_accel = clampValue(std::exp(x3[1]), 50, 5000);
    auto skid_cost = [&](const std::vector<double>& x) {
      DriveModel m = model;
      m.lateral_skid = clampValue(x[0], 0, 0.1);
      return squaredError(predicted(m, log).sideways, meas.sideways);
    };
    model.lateral_skid = clampValue(minimize(skid_cost, {model.lateral_skid}, {0.005})[0], 0, 0.1);
    // Scrub again now that the grip limit is known (they affect each other a little)
    model.turn_scrub = clampValue(minimize(turn_cost, {model.turn_scrub}, {0.02})[0], 0.3, 1.1);
  }

  if (verbose) {
    Signals before = predicted(DriveModel(), log), after = predicted(model, log);
    std::printf("Fit quality (lower is better):\n");
    std::printf("  wheel speed error: %.2f in/s with the default robot, %.2f in/s fitted\n",
                (rms(before.left, meas.left) + rms(before.right, meas.right)) / 2, (rms(after.left, meas.left) + rms(after.right, meas.right)) / 2);
    std::printf("  turn rate error:   %.2f deg/s with the default robot, %.2f deg/s fitted\n", rms(before.turn, meas.turn), rms(after.turn, meas.turn));
    if (trackers) std::printf("  tracking wheels:   %.2f in/s fitted\n", (rms(after.forward, meas.forward) + rms(after.sideways, meas.sideways)) / 2);
    else std::printf("  (no tracking wheels in the log, so wheel slip and skid keep their default values)\n");
  }
  return model;
}

void printModel(const DriveModel& m) {
  std::printf("  kS = %.3f V              friction before the wheels turn\n", m.kS);
  std::printf("  kV = %.4f V per in/s     top speed about %.0f in/s at 12 V\n", m.kV, (12 - m.kS) / m.kV);
  std::printf("  kA = %.4f V per in/s^2   motor lag about %.2f s\n", m.kA, m.kA / m.kV);
  std::printf("  max_accel = %.0f in/s^2     brake_decel = %.0f in/s^2     coast_decel = %.0f in/s^2\n", m.max_accel, m.brake_decel, m.coast_decel);
  std::printf("  turn_scrub = %.3f    tread_slip = %.3f    traction_accel = %.0f in/s^2    lateral_skid = %.4f\n",
              m.turn_scrub, m.tread_slip, m.traction_accel, m.lateral_skid);
}

// ============================================================================
// SELF-TEST: make up a robot, record the calibration pattern on it, and fit it back
// (the pattern matches calibrateDrive() in src/drive-log.cpp)
// ============================================================================
int selfTest() {
  struct Step { double seconds; int mode; double l0, r0, l1, r1; };
  const Step steps[] = {
    {2.0, 0, 0, 0, 6, 6}, {1.0, 2, 0, 0, 0, 0}, {2.0, 0, 0, 0, -6, -6}, {1.0, 2, 0, 0, 0, 0},
    {0.45, 0, 12, 12, 12, 12}, {0.6, 1, 0, 0, 0, 0}, {0.45, 0, -12, -12, -12, -12}, {0.6, 1, 0, 0, 0, 0},
    {0.9, 0, 8, -8, 8, -8}, {0.5, 1, 0, 0, 0, 0}, {0.9, 0, -8, 8, -8, 8}, {0.5, 1, 0, 0, 0, 0},
    {1.0, 0, 9, 4, 9, 4}, {0.8, 2, 0, 0, 0, 0}, {1.0, 0, -9, -4, -9, -4}, {0.8, 2, 0, 0, 0, 0},
  };
  DriveModel truth;
  truth.kS = 0.8; truth.kV = 0.21; truth.kA = 0.035; truth.max_accel = 220; truth.brake_decel = 380; truth.coast_decel = 85;
  truth.turn_scrub = 0.9; truth.tread_slip = 0.97; truth.traction_accel = 260; truth.lateral_skid = 0.02;

  int failures = 0;
  for (int with_trackers = 0; with_trackers <= 1; with_trackers++) {
    DriveLog log;
    log.wheel_distance_in = 7.47; log.track = 12.3;
    log.horizontal = log.vertical = with_trackers != 0;
    log.horizontal_offset = 2.5; log.vertical_offset = -0.5; log.horizontal_diameter = 2; log.vertical_diameter = 2;
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0, 1);
    DriveSide left, right;
    double t = 0, ld = 0, rd = 0, heading = 0, hd = 0, vd = 0;
    for (const Step& s : steps) {
      for (double into = 0; into < s.seconds - 1e-9; into += 0.01) {
        double f = into / s.seconds;
        LogRow row = {t, s.mode, s.mode == 0 ? s.l0 + (s.l1 - s.l0) * f : 0, s.mode == 0 ? s.r0 + (s.r1 - s.r0) * f : 0,
                      std::round(ld * 10) / 10, std::round(rd * 10) / 10, heading + noise(rng) * 0.02,
                      with_trackers ? std::round(hd * 10) / 10 : 0, with_trackers ? std::round(vd * 10) / 10 : 0};
        log.rows.push_back(row);
        for (int k = 0; k < 10; k++) {
          stepDriveSide(truth, left, row.mode, row.left_volts, 0.001);
          stepDriveSide(truth, right, row.mode, row.right_volts, 0.001);
          double speed = 0, turn = 0, skid = 0;
          driveMotion(truth, left, right, log.track, speed, turn, skid);
          ld += left.wheel / log.wheel_distance_in * 360.0 * 0.001;
          rd += right.wheel / log.wheel_distance_in * 360.0 * 0.001;
          heading += turn * 180.0 / PI * 0.001;
          vd += (speed - log.vertical_offset * turn) * 0.001 / (PI * log.vertical_diameter) * 360.0;
          hd += (skid - log.horizontal_offset * turn) * 0.001 / (PI * log.horizontal_diameter) * 360.0;
        }
        t += 0.01;
      }
    }
    DriveModel fit = fitProfile(log, false);
    std::printf("%s tracking wheels:\n", with_trackers ? "With" : "Without");
    printModel(fit);
    auto close = [&](const char* name, double got, double want, double tolerance) {
      bool ok = std::fabs(got - want) <= tolerance * std::fabs(want);
      if (!ok) { failures++; std::printf("  FAIL: %s fitted %.4f, real %.4f\n", name, got, want); }
    };
    close("kS", fit.kS, truth.kS, 0.1);
    close("kV", fit.kV, truth.kV, 0.05);
    close("kA", fit.kA, truth.kA, 0.15);
    close("max_accel", fit.max_accel, truth.max_accel, 0.15);
    close("brake_decel", fit.brake_decel, truth.brake_decel, 0.15);
    close("coast_decel", fit.coast_decel, truth.coast_decel, 0.15);
    close("turn_scrub", fit.turn_scrub, truth.turn_scrub, 0.03);
    if (with_trackers) {
      close("tread_slip", fit.tread_slip, truth.tread_slip, 0.02);
      close("traction_accel", fit.traction_accel, truth.traction_accel, 0.2);
    }
  }
  std::printf(failures ? "%d FITTER SELF-TEST FAILURES\n" : "FITTER SELF-TEST PASSED\n", failures);
  return failures != 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 2 && std::string(argv[1]) == "--self-test") return selfTest();
  if (argc < 2 || std::string(argv[1]) == "--help") {
    std::printf("Usage: sh sim/fit-profile.sh rw_drive_log.csv [output]\n"
                "Record the log on the robot with calibrateDrive() (the \"Drive Calibration\" auton).\n");
    return argc < 2;
  }
  DriveLog log;
  if (!readLog(argv[1], log)) {
    std::fprintf(stderr, "Couldn't read a drive log from %s\n", argv[1]);
    return 1;
  }
  std::printf("Fitting %d rows from %s...\n", (int)log.rows.size(), argv[1]);
  DriveModel model = fitProfile(log, true);
  std::printf("Robot profile:\n");
  printModel(model);
  std::string output = argc >= 3 ? argv[2] : "sim/robot-profile.txt";
  if (!saveDriveModel(output.c_str(), model, std::string("fitted from ") + argv[1])) {
    std::fprintf(stderr, "Couldn't write %s\n", output.c_str());
    return 1;
  }
  if (output == "sim/robot-profile.txt") {
    std::printf("Saved %s. The simulator will use it from now on (run it with --no-profile to compare).\n", output.c_str());
  } else {
    std::printf("Saved %s. Run the simulator with --profile %s to use it.\n", output.c_str(), output.c_str());
  }
  return 0;
}
