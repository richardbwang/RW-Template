// RW-Template simulator engine: a cooperative scheduler on a simulated clock, tank-drive
// physics, and simulated sensors. The fake VEX classes in vex/v5_vcs.h call into this.
//
// Threads: every VEX thread is a real OS thread, but only one runs at a time, and it hands
// off whenever it calls wait() (like VEXos's cooperative scheduler). When every thread is
// waiting, the clock jumps to the earliest wake-up and the physics catches up in 1 msec steps.
#include "v5_vcs.h"
#include "sim-engine.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <thread>
#include <utility>

namespace rwsim {
namespace {

const double PI = 3.14159265358979323846;

// ============================================================================
// SCHEDULER STATE
// ============================================================================
struct SimThread {
  int32_t id = 0;
  double wake = 0;
  long ticket = 0;
  bool finished = false, interrupted = false;
  std::condition_variable cv;
  std::function<void()> body;
};
struct Interrupted {};

std::mutex sched_lock;
std::vector<SimThread*>& threadList() { static std::vector<SimThread*> list; return list; }
SimThread* current = nullptr;
double now_ms = 0;
long next_ticket = 0;
int32_t next_thread_id = 1;
std::atomic<long> progress{0};

// ============================================================================
// ROBOT AND WORLD STATE
// ============================================================================
struct MotorState {
  double free_rpm = 200;
  int mode = 0;               // 0 = stopped, 1 = voltage, 2 = percent velocity
  double command = 0;         // Volts or percent
  double velocity_setting = 50;
  int stopping = 0;           // Default brake mode for stop(): 0 coast, 1 brake, 2 hold
  int stop_mode = 0;          // Brake mode of the last stop
  double rpm = 0;
  double raw_deg = 0, zero_deg = 0;
  bool drive = false;
};
std::map<int32_t, MotorState>& motors() { static std::map<int32_t, MotorState> m; return m; }

struct ImuState { double zero_true_deg = 0, offset = 0, drift = 0, calibrate_until = -1; };
std::map<int32_t, ImuState>& imus() { static std::map<int32_t, ImuState> m; return m; }
ImuState& imuAt(int32_t port);

struct TrackerState { double deg = 0, rpm = 0; };
std::map<int32_t, TrackerState>& trackers() { static std::map<int32_t, TrackerState> m; return m; }

struct DistanceState { DistanceMount mount{}; bool mounted = false; double mm = 9999; uint32_t stamp = 0; double next_sample = 0; };
std::map<int32_t, DistanceState>& distances() { static std::map<int32_t, DistanceState> m; return m; }

RobotSetup robot;
WorldSetup world;
BeliefSource belief;
bool configured = false;

double true_x = 0, true_y = 0, true_heading = 0; // Inches, radians (clockwise from +Y)

// An IMU that hasn't been calibrated reads 0 where the robot started
ImuState& imuAt(int32_t port) {
  auto it = imus().find(port);
  if (it == imus().end()) {
    ImuState state;
    state.zero_true_deg = true_heading * 180.0 / 3.14159265358979323846;
    it = imus().emplace(port, state).first;
  }
  return it->second;
}
DriveModel drive_model;                          // Drivetrain physics (see drive-model.h)
DriveSide sides[2];                              // Left/right wheel and ground speeds (in/s)
double turn_rate = 0;                            // rad/s
double imu_scale_error = 1.0, imu_drift_dps = 0;

// Pose history (1 msec steps) for sensor delay
const int HISTORY = 256;
double hist_x[HISTORY], hist_y[HISTORY], hist_h[HISTORY];
long hist_count = 0;

bool comp_enabled = false, comp_auton = false;

// Field obstacles for distance sensor rays
struct Circle { double x, y, r; };
struct Box { double x0, y0, x1, y1; };
std::vector<Circle> circles;
std::vector<Box> boxes;

// Recording
struct Sample { double t, tx, ty, th, bx, by, bh, raw_x, raw_y, raw_h; bool moving, mcl; double mx, my, lv, rv; };
std::vector<Sample> samples;
bool recording = false;

// How RW's coordinates line up with the field: field = rotate(RW position, frame_turn) + frame_origin.
// It starts out relative to where the robot started; setPose() can switch it (see resolveFrame).
double frame_turn_deg = 0, frame_x = 0, frame_y = 0;
bool frame_pending = false;
double record_start = 0, next_record = 0;
std::string run_name, viewer_path, output_path;

std::mt19937 rng(1);
double gauss(double sigma) { std::normal_distribution<double> d(0, sigma); return sigma > 0 ? d(rng) : 0; }

// ============================================================================
// PHYSICS
// ============================================================================
double castRay(double sx, double sy, double bx, double by) {
  double half = world.field_half_in;
  double best = 1e9;
  if (std::fabs(bx) > 1e-9) best = std::min(best, ((bx > 0 ? half : -half) - sx) / bx);
  if (std::fabs(by) > 1e-9) best = std::min(best, ((by > 0 ? half : -half) - sy) / by);
  for (const Circle& c : circles) {
    double fx = sx - c.x, fy = sy - c.y;
    double b = fx * bx + fy * by, cc = fx * fx + fy * fy - c.r * c.r;
    double disc = b * b - cc;
    if (disc < 0) continue;
    double t = -b - std::sqrt(disc);
    if (t > 0 && t < best) best = t;
  }
  for (const Box& b : boxes) {
    double tmin = -1e9, tmax = 1e9;
    if (std::fabs(bx) < 1e-12) { if (sx < b.x0 || sx > b.x1) continue; }
    else { double t1 = (b.x0 - sx) / bx, t2 = (b.x1 - sx) / bx; tmin = std::max(tmin, std::min(t1, t2)); tmax = std::min(tmax, std::max(t1, t2)); }
    if (std::fabs(by) < 1e-12) { if (sy < b.y0 || sy > b.y1) continue; }
    else { double t1 = (b.y0 - sy) / by, t2 = (b.y1 - sy) / by; tmin = std::max(tmin, std::min(t1, t2)); tmax = std::min(tmax, std::max(t1, t2)); }
    if (tmax >= tmin && tmin > 0 && tmin < best) best = tmin;
  }
  return best;
}

// What a drive side is being told to do: volts, brake, or coast
void sideCommand(const std::vector<int32_t>& ports, int& mode, double& volts) {
  double total = 0;
  int driving = 0;
  bool any_coast = false;
  for (int32_t p : ports) {
    MotorState& m = motors()[p];
    if (m.mode == 1) {
      total += m.command;
      driving++;
    } else if (m.mode == 2) {
      // Velocity mode: the motor applies whatever voltage holds that speed
      double target = m.command / 100.0 * m.free_rpm / 60.0 * robot.inches_per_motor_rev;
      total += clampValue(drive_model.kS * (target > 0 ? 1 : (target < 0 ? -1 : 0)) + drive_model.kV * target, -12, 12);
      driving++;
    } else if (m.stop_mode == 0) {
      any_coast = true;
    }
  }
  if (driving > 0) { mode = DRIVE_VOLTS; volts = total / driving; }
  else { mode = any_coast ? DRIVE_COAST : DRIVE_BRAKE; volts = 0; }
}

void physicsStep(double dt) {
  // Mechanism motors: speed follows the command quickly, position integrates
  for (auto& entry : motors()) {
    MotorState& m = entry.second;
    if (m.drive) continue;
    double target = m.mode == 1 ? m.command / 12.0 * m.free_rpm : (m.mode == 2 ? m.command / 100.0 * m.free_rpm : 0);
    double tau = (m.mode == 0 && m.stop_mode == 0) ? 0.4 : 0.05;
    m.rpm += (target - m.rpm) * (1 - std::exp(-dt / tau));
    m.raw_deg += m.rpm * 6.0 * dt;
  }

  // Drive: friction, motor lag and current limit, wheel slip, scrub and skid (drive-model.h)
  for (int s = 0; s < 2; s++) {
    int mode = DRIVE_BRAKE;
    double volts = 0;
    sideCommand(s == 0 ? robot.left_ports : robot.right_ports, mode, volts);
    stepDriveSide(drive_model, sides[s], mode, volts, dt);
  }
  double v = 0, w = 0, lateral = 0;
  driveMotion(drive_model, sides[0], sides[1], robot.track_width_in, v, w, lateral);
  if (world.realistic) {
    v *= 1 + gauss(0.01);
    w *= 1 + gauss(0.01);
  }
  double avg = true_heading + w * dt / 2.0;
  double dx = (v * std::sin(avg) + lateral * std::cos(avg)) * dt;
  double dy = (v * std::cos(avg) - lateral * std::sin(avg)) * dt;

  // Walls: the robot stops at the perimeter but its wheels keep spinning (encoders keep counting)
  double limit = world.field_half_in - robot.robot_size_in / 2.0;
  double new_x = std::max(-limit, std::min(limit, true_x + dx));
  double new_y = std::max(-limit, std::min(limit, true_y + dy));
  dx = new_x - true_x;
  dy = new_y - true_y;
  true_x = new_x;
  true_y = new_y;
  double delta_heading = w * dt;
  true_heading += delta_heading;
  turn_rate = w;

  // Drive encoders measure the wheels, not the robot
  for (int s = 0; s < 2; s++) {
    for (int32_t p : (s == 0 ? robot.left_ports : robot.right_ports)) {
      MotorState& m = motors()[p];
      m.rpm = sides[s].wheel / robot.inches_per_motor_rev * 60.0;
      m.raw_deg += m.rpm * 6.0 * dt;
    }
  }

  // Tracking wheels measure the robot's real motion at their mounting spots
  double forward_moved = dx * std::sin(avg) + dy * std::cos(avg);
  double sideways_moved = dx * std::cos(avg) - dy * std::sin(avg);
  if (robot.vertical_tracker_port >= 0) {
    TrackerState& t = trackers()[robot.vertical_tracker_port];
    double travel = forward_moved - robot.vertical_offset_in * delta_heading;
    t.deg += travel / (PI * robot.vertical_diameter_in) * 360.0;
    t.rpm = travel / dt / (PI * robot.vertical_diameter_in) * 60.0;
  }
  if (robot.horizontal_tracker_port >= 0) {
    TrackerState& t = trackers()[robot.horizontal_tracker_port];
    double travel = sideways_moved - robot.horizontal_offset_in * delta_heading;
    t.deg += travel / (PI * robot.horizontal_diameter_in) * 360.0;
    t.rpm = travel / dt / (PI * robot.horizontal_diameter_in) * 60.0;
  }

  // IMU drift
  for (auto& entry : imus()) entry.second.drift += imu_drift_dps * dt;

  // Pose history for delayed sensors
  int slot = hist_count % HISTORY;
  hist_x[slot] = true_x;
  hist_y[slot] = true_y;
  hist_h[slot] = true_heading;
  hist_count++;

  // Distance sensors: a new reading about every 33 msec, from where the robot was ~40 msec ago
  for (auto& entry : distances()) {
    DistanceState& d = entry.second;
    if (!d.mounted || now_ms < d.next_sample) continue;
    d.next_sample = now_ms + 33;
    long back = std::min<long>(world.realistic ? 40 : 0, hist_count - 1);
    int old = (int)((hist_count - 1 - back) % HISTORY);
    double heading = hist_h[old];
    double ox = d.mount.x * std::cos(heading) + d.mount.y * std::sin(heading);
    double oy = -d.mount.x * std::sin(heading) + d.mount.y * std::cos(heading);
    double beam = heading + d.mount.angle_deg * PI / 180.0;
    double inches = castRay(hist_x[old] + ox, hist_y[old] + oy, std::sin(beam), std::cos(beam));
    if (world.realistic) inches = inches * 1.012 + gauss(0.15 + 0.01 * inches);
    double millimeters = inches * 25.4;
    d.mm = (millimeters < 20 || millimeters > 2000) ? 9999 : millimeters;
    d.stamp = (uint32_t)now_ms;
  }
}

// ============================================================================
// RECORDING AND OUTPUT
// ============================================================================
double sideVolts(const std::vector<int32_t>& ports) {
  double total = 0;
  for (int32_t p : ports) {
    MotorState& m = motors()[p];
    total += m.mode == 1 ? m.command : (m.mode == 2 ? m.command * 0.12 : 0);
  }
  return ports.empty() ? 0 : total / ports.size();
}

// After the code sets its pose (setPose), decide what its coordinates mean: if the new pose
// matches where the robot really is, it's using field coordinates; otherwise it just made
// the robot's current spot its new origin.
void resolveFrame(double raw_x, double raw_y, double raw_h) {
  double true_h = true_heading * 180.0 / PI;
  if (std::hypot(raw_x - true_x, raw_y - true_y) < 6 && std::fabs(std::remainder(raw_h - true_h, 360.0)) < 10) {
    frame_turn_deg = 0;
    frame_x = 0;
    frame_y = 0;
  } else {
    frame_turn_deg = true_h - raw_h;
    double turn = frame_turn_deg * PI / 180.0;
    frame_x = true_x - (raw_x * std::cos(turn) + raw_y * std::sin(turn));
    frame_y = true_y - (-raw_x * std::sin(turn) + raw_y * std::cos(turn));
  }
  frame_pending = false;
}

void recordSample() {
  Sample s{};
  s.t = (now_ms - record_start) / 1000.0;
  s.tx = true_x;
  s.ty = true_y;
  s.th = true_heading * 180.0 / PI;
  s.raw_x = belief.x ? *belief.x : 0;
  s.raw_y = belief.y ? *belief.y : 0;
  s.raw_h = belief.heading ? belief.heading() : 0;
  if (frame_pending) resolveFrame(s.raw_x, s.raw_y, s.raw_h);
  double turn = frame_turn_deg * PI / 180.0;
  s.bx = frame_x + s.raw_x * std::cos(turn) + s.raw_y * std::sin(turn);
  s.by = frame_y - s.raw_x * std::sin(turn) + s.raw_y * std::cos(turn);
  s.bh = s.raw_h + frame_turn_deg;
  s.moving = belief.motion_running ? *belief.motion_running : false;
  s.mcl = belief.mcl_running ? *belief.mcl_running : false;
  double mcl_raw_x = belief.mcl_x ? *belief.mcl_x : 0, mcl_raw_y = belief.mcl_y ? *belief.mcl_y : 0;
  s.mx = frame_x + mcl_raw_x * std::cos(turn) + mcl_raw_y * std::sin(turn);
  s.my = frame_y - mcl_raw_x * std::sin(turn) + mcl_raw_y * std::cos(turn);
  s.lv = sideVolts(robot.left_ports);
  s.rv = sideVolts(robot.right_ports);
  samples.push_back(s);
}

std::string fmt(double value) { char buf[32]; std::snprintf(buf, sizeof(buf), "%.2f", value); return buf; }

void advanceTo(double target_ms) {
  while (now_ms < target_ms - 1e-9) {
    double step = std::min(1.0, target_ms - now_ms);
    physicsStep(step / 1000.0);
    now_ms += step;
    progress++;
    if (recording && now_ms >= next_record) {
      recordSample();
      next_record += 10;
    }
    if (recording && now_ms - record_start >= world.time_limit_s * 1000.0) finishRun("time limit");
  }
}

// ============================================================================
// SCHEDULER
// ============================================================================
SimThread* pickNext() {
  SimThread* best = nullptr;
  for (SimThread* t : threadList()) {
    if (t->finished) continue;
    if (!best || t->wake < best->wake || (t->wake == best->wake && t->ticket < best->ticket)) best = t;
  }
  return best;
}

// Hand control to the next thread. Called with the lock held by the thread giving up control.
void switchFrom(std::unique_lock<std::mutex>& lk, SimThread* me) {
  SimThread* next = pickNext();
  if (!next) finishRun("every thread ended");
  if (next->wake > now_ms) advanceTo(next->wake);
  current = next;
  progress++;
  if (next == me) return;
  next->cv.notify_one();
  if (me->finished) return;
  me->cv.wait(lk, [me] { return current == me; });
}

void watchdog() {
  long last = progress.load();
  int stuck_seconds = 0;
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    long now_progress = progress.load();
    stuck_seconds = now_progress == last ? stuck_seconds + 1 : 0;
    last = now_progress;
    if (stuck_seconds >= 8) {
      std::fprintf(stderr, "\nThe simulation is stuck at %.2f s: some code is looping without calling wait().\n"
                           "On the real robot this would freeze other tasks too. Add a wait(10, msec) inside the loop.\n",
                   now_ms / 1000.0);
      std::_Exit(2);
    }
  }
}

}  // namespace

// ============================================================================
// ENGINE HOOKS USED BY THE FAKE VEX CLASSES
// ============================================================================
double nowMs() { return now_ms; }

int32_t currentThreadId() { return current ? current->id : 0; }

void sleepMs(double ms) {
  if (!current) return;
  std::unique_lock<std::mutex> lk(sched_lock);
  SimThread* me = current;
  me->wake = now_ms + std::max(ms, 0.0);
  me->ticket = next_ticket++;
  switchFrom(lk, me);
  if (me->interrupted) throw Interrupted();
}

int32_t spawnThread(std::function<void()> body) {
  std::unique_lock<std::mutex> lk(sched_lock);
  SimThread* t = new SimThread();
  t->id = next_thread_id++;
  t->wake = now_ms;
  t->ticket = next_ticket++;
  t->body = std::move(body);
  threadList().push_back(t);
  std::thread([t] {
    {
      std::unique_lock<std::mutex> start(sched_lock);
      t->cv.wait(start, [t] { return current == t; });
    }
    try {
      if (!t->interrupted) t->body();
    } catch (Interrupted&) {
    }
    std::unique_lock<std::mutex> end(sched_lock);
    t->finished = true;
    switchFrom(end, t);
  }).detach();
  return t->id;
}

void interruptThread(int32_t id) {
  for (SimThread* t : threadList()) {
    if (t->id == id && !t->finished) {
      t->interrupted = true;
      t->wake = now_ms;  // Run it soon so it can stop
      t->ticket = next_ticket++;
    }
  }
}

bool threadRunning(int32_t id) {
  for (SimThread* t : threadList()) if (t->id == id) return !t->finished;
  return false;
}

void motorRegister(int32_t port, double free_rpm) { if (free_rpm > 0) motors()[port].free_rpm = free_rpm; }
void motorVoltage(int32_t port, double volts) { MotorState& m = motors()[port]; m.mode = 1; m.command = std::max(-12.0, std::min(12.0, volts)); }
void motorPercent(int32_t port, double percent) { MotorState& m = motors()[port]; m.mode = 2; m.command = std::max(-100.0, std::min(100.0, percent)); }
void motorStop(int32_t port, int brake_mode) { MotorState& m = motors()[port]; m.mode = 0; m.command = 0; m.stop_mode = brake_mode < 0 ? m.stopping : brake_mode; }
void motorSetStopping(int32_t port, int brake_mode) { motors()[port].stopping = brake_mode; }
void motorSetVelocity(int32_t port, double percent) { motors()[port].velocity_setting = percent; }
double motorVelocitySetting(int32_t port) { return motors()[port].velocity_setting; }
double motorPositionDeg(int32_t port) { MotorState& m = motors()[port]; return m.raw_deg - m.zero_deg; }
void motorSetPositionDeg(int32_t port, double deg) { MotorState& m = motors()[port]; m.zero_deg = m.raw_deg - deg; }
double motorRpm(int32_t port) { return motors()[port].rpm; }
double motorFreeRpm(int32_t port) { return motors()[port].free_rpm; }

double imuRotationDeg(int32_t port) {
  ImuState& imu = imuAt(port);
  if (imu.calibrate_until > now_ms) return 0;
  return (true_heading * 180.0 / PI - imu.zero_true_deg) * imu_scale_error + imu.drift + imu.offset;
}
void imuSetRotationDeg(int32_t port, double deg) {
  ImuState& imu = imuAt(port);
  imu.offset += deg - imuRotationDeg(port);
  frame_pending = true;  // The code set its own heading (e.g. setPose); check what its coordinates mean
}
void imuCalibrate(int32_t port) {
  ImuState& imu = imuAt(port);
  imu.zero_true_deg = true_heading * 180.0 / PI;
  imu.offset = 0;
  imu.drift = 0;
  imu.calibrate_until = now_ms + 1000;
}
bool imuCalibrating(int32_t port) { return imuAt(port).calibrate_until > now_ms; }
double imuRateDps(int32_t port) { (void)port; return turn_rate * 180.0 / PI; }

double rotationSensorDeg(int32_t port) { return trackers()[port].deg; }
void rotationSensorSetDeg(int32_t port, double deg) { trackers()[port].deg = deg; }
double rotationSensorRpm(int32_t port) { return trackers()[port].rpm; }

double distanceMm(int32_t port) { auto it = distances().find(port); return it == distances().end() ? 9999 : it->second.mm; }
uint32_t distanceTimestamp(int32_t port) { auto it = distances().find(port); return it == distances().end() ? 0 : it->second.stamp; }

bool competitionEnabled() { return comp_enabled; }
bool competitionAutonomous() { return comp_auton; }

// ============================================================================
// RUNNER SIDE
// ============================================================================
void adoptMainThread() {
  SimThread* t = new SimThread();
  t->id = 0;
  threadList().push_back(t);
  current = t;
  std::thread(watchdog).detach();
}

void setup(const RobotSetup& robot_setup, const WorldSetup& world_setup, const BeliefSource& belief_source) {
  robot = robot_setup;
  world = world_setup;
  belief = belief_source;
  rng.seed(world.seed);
  true_x = world.start_x;
  true_y = world.start_y;
  true_heading = world.start_heading_deg * PI / 180.0;
  // Until the code sets its own pose, RW's numbers are relative to where the robot started
  frame_turn_deg = world.start_heading_deg;
  frame_x = world.start_x;
  frame_y = world.start_y;
  for (int i = 0; i < HISTORY; i++) { hist_x[i] = true_x; hist_y[i] = true_y; hist_h[i] = true_heading; }
  hist_count = 1;
  for (int32_t p : robot.left_ports) motors()[p].drive = true;
  for (int32_t p : robot.right_ports) motors()[p].drive = true;
  drive_model = robot.drive;
  if (!robot.drive_from_profile) {
    // Default drive from the motor cartridge and wheels: about 92% of free speed at 12 V
    double free_rpm = robot.left_ports.empty() ? 600 : motors()[robot.left_ports[0]].free_rpm;
    double free_ips = free_rpm / 60.0 * robot.inches_per_motor_rev;
    drive_model.kV = (12 - drive_model.kS) / (0.92 * free_ips);
    drive_model.kA = drive_model.kV * 0.13;
  }
  if (!world.realistic) {
    // Ideal robot: no slip, scrub or skid
    drive_model.tread_slip = 1;
    drive_model.turn_scrub = 1;
    drive_model.lateral_skid = 0;
    drive_model.traction_accel = 1e9;
  }
  if (world.realistic) {
    imu_scale_error = 0.996;      // Reads about 358.6 degrees per real turn
    imu_drift_dps = 0.4 / 60.0;   // About 0.4 degrees per minute
  }
  for (const DistanceMount& mount : robot.distance_sensors) {
    DistanceState& d = distances()[mount.port];
    d.mount = mount;
    d.mounted = true;
    d.next_sample = (mount.port % 4) * 8;  // Sensors don't all update on the same msec
  }
  if (world.override_field) {
    // Override (2026-27): 9 goals, corner loaders on the red (left) and blue (right) walls,
    // and Cup+Pin stacks along every wall
    circles.push_back({0, 0, 2.8});
    for (int sx = -1; sx <= 1; sx += 2) {
      for (int sy = -1; sy <= 1; sy += 2) {
        circles.push_back({sx * 47.09, sy * 23.55, 2.8});
        circles.push_back({sx * 23.55, sy * 47.09, 2.8});
        boxes.push_back({sx * 68.5 - 2, sy * 58.7 - 2.5, sx * 68.5 + 2, sy * 58.7 + 2.5});
        boxes.push_back({sx * 23.6 - 1.6, sy * 68.7 - 1.6, sx * 23.6 + 1.6, sy * 68.7 + 1.6});
        boxes.push_back({sx * 68.7 - 1.6, sy * 23.6 - 1.6, sx * 68.7 + 1.6, sy * 23.6 + 1.6});
      }
    }
  }
  configured = true;
}

void setCompetition(bool enabled, bool autonomous) {
  comp_enabled = enabled;
  comp_auton = autonomous;
}

void startRecording(const std::string& name, const std::string& viewer, const std::string& output) {
  run_name = name;
  viewer_path = viewer;
  output_path = output;
  record_start = now_ms;
  next_record = now_ms;
  recording = true;
  recordSample();
  next_record += 10;
}

[[noreturn]] void finishRun(const std::string& reason) {
  recordSample();
  double duration = samples.empty() ? 0 : samples.back().t;

  // Error between what RW believed and where the robot really was
  double max_error = 0, max_error_t = 0, end_error = 0, end_heading_error = 0;
  for (const Sample& s : samples) {
    double e = std::hypot(s.bx - s.tx, s.by - s.ty);
    if (e > max_error) { max_error = e; max_error_t = s.t; }
  }
  if (!samples.empty()) {
    const Sample& last = samples.back();
    end_error = std::hypot(last.bx - last.tx, last.by - last.ty);
    end_heading_error = std::remainder(last.bh - last.th, 360.0);
  }

  // Replay page: the viewer with this run's data filled in
  std::ostringstream data;
  data << "{\"name\":\"";
  for (char c : run_name) { if (c == '"' || c == '\\') data << '\\'; data << c; }
  data << "\",\"reason\":\"" << reason << "\",\"limit\":" << fmt(world.time_limit_s)
       << ",\"half\":" << fmt(world.field_half_in) << ",\"size\":" << fmt(robot.robot_size_in)
       << ",\"override\":" << (world.override_field ? "true" : "false")
       << ",\"realistic\":" << (world.realistic ? "true" : "false")
       << ",\"start\":[" << fmt(world.start_x) << "," << fmt(world.start_y) << "," << fmt(world.start_heading_deg) << "]"
       << ",\"maxError\":" << fmt(max_error) << ",\"maxErrorT\":" << fmt(max_error_t)
       << ",\"endError\":" << fmt(end_error) << ",\"endHeadingError\":" << fmt(end_heading_error)
       << ",\"samples\":[";
  for (size_t i = 0; i < samples.size(); i++) {
    const Sample& s = samples[i];
    data << (i ? "," : "") << "[" << fmt(s.t) << "," << fmt(s.tx) << "," << fmt(s.ty) << "," << fmt(s.th) << ","
         << fmt(s.bx) << "," << fmt(s.by) << "," << fmt(s.bh) << "," << (s.moving ? 1 : 0) << ","
         << (s.mcl ? 1 : 0) << "," << fmt(s.mx) << "," << fmt(s.my) << "," << fmt(s.lv) << "," << fmt(s.rv) << "]";
  }
  data << "]}";

  bool wrote = false;
  std::ifstream in(viewer_path);
  if (in) {
    std::stringstream page;
    page << in.rdbuf();
    std::string html = page.str();
    std::string marker = "/*SIM_DATA*/null";
    size_t at = html.find(marker);
    if (at != std::string::npos) html.replace(at, marker.size(), data.str());
    std::ofstream out(output_path);
    if (out) { out << html; wrote = true; }
  }

  std::printf("\n%s after %.2f s (%s)\n", run_name.c_str(), duration, reason.c_str());
  if (!samples.empty()) {
    const Sample& last = samples.back();
    std::printf("  Robot ended at     x %7.2f  y %7.2f  heading %7.2f  (field coordinates)\n", last.tx, last.ty, last.th);
    std::printf("  RW thought it was  x %7.2f  y %7.2f  heading %7.2f  (off by %.2f in, %.2f deg)\n", last.bx, last.by, last.bh, end_error, end_heading_error);
    std::printf("  RW's own numbers   x_pos %.2f  y_pos %.2f  heading %.2f\n", last.raw_x, last.raw_y, last.raw_h);
    std::printf("  Largest position error during the run: %.2f in at %.2f s\n", max_error, max_error_t);
  }
  if (wrote) std::printf("  Replay: %s\n", output_path.c_str());
  else std::printf("  Could not write the replay (viewer %s, output %s)\n", viewer_path.c_str(), output_path.c_str());
  std::fflush(stdout);
  std::_Exit(0);
}

}  // namespace rwsim
