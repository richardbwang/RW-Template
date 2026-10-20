// RW-Template simulator: runs your autonomous routines on a simulated robot and field and
// writes an HTML replay. Run it with: sh sim/run-sim.sh "Auton Name"   (see README)
#include "vex.h"
#include "motor-control.h"
#include "../custom/include/user.h"
#include "sim-engine.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

// Normally defined in src/main.cpp, which the simulator replaces
vex::competition Competition;

// Auton selector internals (src/auton-selector.cpp)
extern const char* auton_names[];
extern int auton_count;
extern int selected_auton;

namespace {

struct Options {
  std::string auton;
  std::string profile = "sim/robot-profile.txt";
  bool use_profile = true;
  std::string viewer = "sim/viewer.html";
  std::string output;
  rwsim::WorldSetup world;
  double robot_size = 15;
  bool list = false;
};

void usage() {
  std::printf(
    "Usage: sh sim/run-sim.sh [auton name] [options]\n"
    "  --list                 List the autons added with addAuton()\n"
    "  --start X,Y,HEADING    Where the robot starts (field-centered inches, compass degrees)\n"
    "  --time SECONDS         Stop after this long (default 15, use 60 for skills)\n"
    "  --ideal                No wheel slip, IMU error, or sensor noise\n"
    "  --walls                Empty field (no Override goals, loaders, or stacks)\n"
    "  --size INCHES          Robot size for wall collisions (default 15)\n"
    "  --seed N               Random seed for slip and noise\n"
    "  --profile FILE         Robot profile from fit-profile (default sim/robot-profile.txt if it exists)\n"
    "  --no-profile           Ignore the robot profile and use the default robot\n"
    "  --out FILE             Where to write the replay\n");
}

bool parseStart(const char* text, rwsim::WorldSetup& world) {
  return std::sscanf(text, "%lf,%lf,%lf", &world.start_x, &world.start_y, &world.start_heading_deg) == 3;
}

Options parse(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    auto next = [&](const char* name) -> const char* {
      if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", name); std::exit(1); }
      return argv[++i];
    };
    if (arg == "--list") opt.list = true;
    else if (arg == "--start") { if (!parseStart(next("--start"), opt.world)) { std::fprintf(stderr, "--start needs X,Y,HEADING\n"); std::exit(1); } }
    else if (arg == "--time") opt.world.time_limit_s = std::atof(next("--time"));
    else if (arg == "--ideal") opt.world.realistic = false;
    else if (arg == "--walls") opt.world.override_field = false;
    else if (arg == "--size") opt.robot_size = std::atof(next("--size"));
    else if (arg == "--seed") opt.world.seed = (unsigned)std::atoi(next("--seed"));
    else if (arg == "--out") opt.output = next("--out");
    else if (arg == "--profile") opt.profile = next("--profile");
    else if (arg == "--no-profile") opt.use_profile = false;
    else if (arg == "--viewer") opt.viewer = next("--viewer");
    else if (arg == "--help" || arg == "-h") { usage(); std::exit(0); }
    else if (!arg.empty() && arg[0] == '-') { std::fprintf(stderr, "Unknown option %s\n", arg.c_str()); usage(); std::exit(1); }
    else opt.auton = arg;
  }
  return opt;
}

std::string fileName(const std::string& name) {
  std::string out;
  for (char c : name) out += (std::isalnum((unsigned char)c) ? c : '-');
  return out.empty() ? "auton" : out;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt = parse(argc, argv);
  rwsim::adoptMainThread();

  // The simulated robot comes from custom/src/robot-config.cpp
  rwsim::RobotSetup robot;
  robot.left_ports = left_chassis.simPorts();
  robot.right_ports = right_chassis.simPorts();
  robot.track_width_in = distance_between_wheels;
  robot.inches_per_motor_rev = wheel_distance_in;
  robot.robot_size_in = opt.robot_size;
  if (using_horizontal_tracker) {
    robot.horizontal_tracker_port = horizontal_tracker.index();
    robot.horizontal_offset_in = horizontal_tracker_dist_from_center;
    robot.horizontal_diameter_in = horizontal_tracker_diameter;
  }
  if (using_vertical_tracker) {
    robot.vertical_tracker_port = vertical_tracker.index();
    robot.vertical_offset_in = vertical_tracker_dist_from_center;
    robot.vertical_diameter_in = vertical_tracker_diameter;
  }
  robot.distance_sensors = {
    {front_sensor.index(), front_sensor_offsetX, front_sensor_offsetY, 0},
    {right_sensor.index(), right_sensor_offsetX, right_sensor_offsetY, 90},
    {back_sensor.index(), back_sensor_offsetX, back_sensor_offsetY, 180},
    {left_sensor.index(), left_sensor_offsetX, left_sensor_offsetY, 270},
  };

  // A fitted robot profile, if there is one (otherwise a typical drive for these motors and wheels)
  bool from_profile = opt.use_profile && rwsim::loadDriveModel(opt.profile.c_str(), robot.drive);
  robot.drive_from_profile = from_profile;

  rwsim::BeliefSource belief;
  belief.x = &x_pos;
  belief.y = &y_pos;
  belief.heading = getInertialHeading;
  belief.motion_running = &motion_running;
  belief.mcl_running = &mcl_running;
  belief.mcl_x = &mcl_x;
  belief.mcl_y = &mcl_y;
  rwsim::setup(robot, opt.world, belief);

  // Pre-autonomous, like pre_auton() in src/main.cpp (calibrates the IMU, starts odometry and the selector)
  runPreAutonomous();

  if (opt.list || (auton_count == 0 && !opt.auton.empty())) {
    if (auton_count == 0) std::printf("No autons were added with addAuton(), so runAutonomous() decides what runs.\n");
    for (int i = 0; i < auton_count; i++) std::printf("  %s\n", auton_names[i]);
    std::fflush(stdout);
    std::_Exit(opt.list ? 0 : 1);
  }

  // Pick the routine like tapping it on the selector
  std::string name = opt.auton;
  if (auton_count > 0) {
    if (!name.empty()) {
      int found = -1;
      for (int i = 0; i < auton_count; i++) if (name == auton_names[i]) found = i;
      if (found < 0) {
        std::fprintf(stderr, "No auton named \"%s\". Added autons:\n", name.c_str());
        for (int i = 0; i < auton_count; i++) std::fprintf(stderr, "  %s\n", auton_names[i]);
        std::_Exit(1);
      }
      selected_auton = found;
    }
    name = auton_names[selected_auton];
  } else if (name.empty()) {
    name = "runAutonomous";
  }

  std::string output = opt.output.empty() ? "sim/out/" + fileName(name) + ".html" : opt.output;
  std::printf("Simulating \"%s\" from x %.1f, y %.1f, heading %.1f (%s, %s field, %.0f s limit)\n", name.c_str(),
              opt.world.start_x, opt.world.start_y, opt.world.start_heading_deg,
              opt.world.realistic ? "realistic" : "ideal", opt.world.override_field ? "Override" : "empty",
              opt.world.time_limit_s);
  std::printf("Robot: %s\n", from_profile ? ("fitted profile " + opt.profile).c_str() : "default drive (no robot profile)");
  std::fflush(stdout);

  rwsim::startRecording(name, opt.viewer, output);
  rwsim::setCompetition(true, true);
  runAutonomous();
  // Keep watching a moment so the replay shows the robot settle
  vex::wait(1000, vex::msec);
  rwsim::finishRun("auton finished");
}
