#!/bin/sh
# Drives a strafing auton in the simulator on each drive type: X-drive, mecanum (strafes less per
# wheel turn), H-drive, and a tank drive (which drives there and turns instead), in the VEXcode
# project and in the PROS one (pros/). Each one changes a copy of the starter files, so your own
# files aren't touched. Run: sh tests/test-drive-types.sh
set -e
cd "$(dirname "$0")/.."
work=$(mktemp -d "${TMPDIR:-/tmp}/rw-drive-types.XXXXXX")
trap 'rm -rf "$work"' EXIT
failures=0

fail() {
  echo "  FAILED: $*"
  failures=$((failures + 1))
}

strafe_test='
// Strafes right, goes up diagonally while turning, then comes back to where it started
void strafeTest() {
  setPose(0, 0, 0);
  strafe(24, 2000);
  strafeToPose(24, 24, 90, 3000);
  strafeToPose(0, 24, 180, 3000);
  strafeToPose(0, 0, 0, 3000);
}'

# Makes runPreAutonomous() in $1 call $2 (a drive setup line, or nothing) and add the strafing auton,
# and sets strafe_distance_in in the settings file $3 to $4 (or leaves it at 0)
set_up() {
  setup_line=$2 perl -0pi -e 's/(\n  startOdometry\(\);)/\n  $ENV{setup_line}$1/; s/(\n  startAutonSelector\(\);)/\n  addAuton("Strafe Test", strafeTest);$1/' "$1"
  if [ -n "$4" ]; then
    strafe=$4 perl -pi -e 's/^double strafe_distance_in = 0;/double strafe_distance_in = $ENV{strafe};/' "$3"
    grep -qF "double strafe_distance_in = $4;" "$3" || fail "couldn't set strafe_distance_in in $3"
  fi
}

# A copy of the VEXcode project in $1
make_vexcode() {
  mkdir -p "$1"
  cp -R src include custom sim "$1/"
  rm -rf "$1/sim/build" "$1/sim/out"
  set_up "$1/custom/src/user.cpp" "$2" "$1/custom/src/robot-config.cpp" "$3"
  printf '\nvoid strafeTest();\n' >> "$1/custom/include/autonomous.h"
  printf '%s\n' "$strafe_test" >> "$1/custom/src/autonomous.cpp"
}

# A copy of the PROS project in $1/pros (next to the shared code, which rw/sync.sh copies in like
# pros/Makefile does), with four corner motors for useHolonomicDrive()
make_pros() {
  mkdir -p "$1/pros"
  cp -R src include sim "$1/"
  rm -rf "$1/sim/build" "$1/sim/out"
  cp -R pros/include pros/src pros/rw pros/project.pros "$1/pros/"
  [ "$(sh "$1/pros/rw/sync.sh")" = ok ] || fail "couldn't copy RW's code into $1/pros"
  set_up "$1/pros/src/rw-user.cpp" "$2" "$1/pros/src/rw-config.cpp" "$3"
  printf '\nextern pros::Motor left_front, right_front, left_back, right_back;\nvoid strafeTest();\n' >> "$1/pros/include/rw-config.h"
  printf '\npros::Motor left_front(-1, pros::MotorGearset::blue), right_front(4, pros::MotorGearset::blue);\npros::Motor left_back(-2, pros::MotorGearset::blue), right_back(5, pros::MotorGearset::blue);\n' >> "$1/pros/src/rw-config.cpp"
  printf '%s\n' "$strafe_test" >> "$1/pros/src/rw-autons.cpp"
}

# Runs the auton in project $2 and checks where the robot ended up and how well RW tracked it
check_run() {
  name=$1
  project=$2
  shift 2
  echo "- $name"
  if ! sh "$project/sim/run-sim.sh" "Strafe Test" --walls --start 0,0,0 --time 20 > "$work/$name.txt" 2>&1 < /dev/null; then
    sed 's/^/    | /' "$work/$name.txt"
    fail "$name: the simulator didn't run"
    return
  fi
  sed -n 's/^/    | /p' "$work/$name.txt" | grep -E "Robot|ended|thought|Largest|strafe" || true
  ended=$(sed -n 's/.*Robot ended at *x *\([-0-9.]*\) *y *\([-0-9.]*\).*/\1 \2/p' "$work/$name.txt")
  off=$(sed -n 's/.*(off by \([0-9.]*\) in.*/\1/p' "$work/$name.txt")
  awk -v e="$ended" -v o="$off" 'BEGIN { split(e, p, " "); exit !(p[1]*p[1] + p[2]*p[2] < 2.0*2.0 && o != "" && o < 1.5) }' ||
    fail "$name: ended at ($ended), $off in from where RW thought (want within 2 in of the start, under 1.5 in off)"
  for expected in "$@"; do
    grep -q "$expected" "$work/$name.txt" || fail "$name: the output doesn't say \"$expected\""
  done
}

# Mecanum wheels slip when they strafe, so they move less sideways per wheel turn
mecanum_strafe="wheel_distance_in * 0.85"

corners="useHolonomicDrive(left_chassis1, right_chassis1, left_chassis2, right_chassis2);"
make_vexcode "$work/vexcode-x-drive" "$corners" ""
check_run vexcode-x-drive "$work/vexcode-x-drive" "Robot: holonomic drive"
make_vexcode "$work/vexcode-mecanum" "$corners" "$mecanum_strafe"
check_run vexcode-mecanum "$work/vexcode-mecanum" "Robot: holonomic drive"
# The H-drive's motor gets the name the docs use, so a clash with one of RW's own names shows up here
make_vexcode "$work/vexcode-h-drive" "useHDrive(strafe_motor);" ""
printf '\nmotor strafe_motor = motor(PORT18, ratio18_1, false);\n' >> "$work/vexcode-h-drive/custom/src/robot-config.cpp"
printf '\nextern motor strafe_motor;\n' >> "$work/vexcode-h-drive/custom/include/robot-config.h"
check_run vexcode-h-drive "$work/vexcode-h-drive" "Robot: H-drive"
make_vexcode "$work/vexcode-tank" "" ""
check_run vexcode-tank "$work/vexcode-tank" "Robot: tank drive" "can't strafe"

corners="useHolonomicDrive(left_front, right_front, left_back, right_back);"
make_pros "$work/pros-x-drive" "$corners" ""
check_run pros-x-drive "$work/pros-x-drive/pros" "Robot: holonomic drive"
make_pros "$work/pros-mecanum" "$corners" "$mecanum_strafe"
check_run pros-mecanum "$work/pros-mecanum/pros" "Robot: holonomic drive"
make_pros "$work/pros-h-drive" "useHDrive(strafe_motor);" ""
printf '\npros::Motor strafe_motor(18, pros::MotorGearset::green);\n' >> "$work/pros-h-drive/pros/src/rw-config.cpp"
printf '\nextern pros::Motor strafe_motor;\n' >> "$work/pros-h-drive/pros/include/rw-config.h"
check_run pros-h-drive "$work/pros-h-drive/pros" "Robot: H-drive"
make_pros "$work/pros-tank" "" ""
check_run pros-tank "$work/pros-tank/pros" "Robot: tank drive" "can't strafe"

if [ "$failures" -gt 0 ]; then
  echo "$failures drive type check(s) failed"
  exit 1
fi
echo "All drive type checks passed"
