#!/bin/sh
# Installs the PROS template (from tools/build-pros-template.sh) into new PROS projects the way a
# team would, builds them, checks that RW links with the team's code, and runs an auton in the
# simulator. Two projects: one without PROS's default libraries, with main.cpp replaced the way the
# docs say for a new project, and a default one (with liblvgl) that keeps PROS's main.cpp, LLEMU and
# all, and adds RW's calls, like a project a team already has. Needs the ARM toolchain, the PROS CLI,
# a C++ compiler, and internet access for the PROS kernel and libraries.
#   sh tests/test-pros-template.sh pros/RW-Template@4.1.0.zip
set -e
if [ ! -f "$1" ]; then
  echo "Usage: sh tests/test-pros-template.sh <RW-Template@version.zip>"
  exit 1
fi
zip="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
work=$(mktemp -d "${TMPDIR:-/tmp}/rw-pros-test.XXXXXX")
trap 'rm -rf "$work"' EXIT
failures=0
fail() {
  echo "  FAILED: $*"
  failures=$((failures + 1))
}

# Makes a PROS project in $work/$1 (with the new-project options in $2) and adds the template to it
add_template() {
  project="$work/$1"
  echo "- $1: making a new PROS project and adding the template to it"
  pros c new-project "$project" v5 4.2.2 $2 > "$work/log.txt" 2>&1 || { cat "$work/log.txt"; exit 1; }
  cp "$zip" "$project/"
  (cd "$project" && pros c fetch "$(basename "$zip")") > "$work/log.txt" 2>&1 || { cat "$work/log.txt"; exit 1; }
  # The exact version in the zip (RW-Template@4.1.0.zip), not a newer one the PROS CLI already has
  (cd "$project" && pros c apply "$(basename "$zip" .zip)") > "$work/log.txt" 2>&1 || { cat "$work/log.txt"; exit 1; }
  for file in src/rw-config.cpp src/rw-autons.cpp src/rw-user.cpp include/rw-config.h include/rw/api.h firmware/RW-Template.a \
      sim/run-sim.sh sim/pros/pros-sim.h sim/rw/hardware.cpp sim/rw/motor-control.cpp; do
    [ -f "$project/$file" ] || fail "$1: the template didn't add $file"
  done
}

# Builds the project in $work/$1, checks that RW links with the team's code, and runs an auton in the simulator
check_project() {
  project="$work/$1"
  echo "- $1: building it"
  if ! (cd "$project" && make) > "$work/log.txt" 2>&1; then
    cat "$work/log.txt"
    fail "$1: the project didn't build"
    return
  fi
  # RW uses the team's devices by name, so it has to link with the team's code (the hot package)
  if [ -f "$project/bin/hot.package.elf" ]; then
    arm-none-eabi-nm -C "$project/bin/hot.package.elf" | grep -q ' T driveTo(' || fail "$1: RW isn't in the hot package"
    ! arm-none-eabi-nm -C "$project/bin/cold.package.elf" | grep -q ' T driveTo(' || fail "$1: RW is in the cold package"
  fi
  echo "- $1: running an auton in the simulator"
  if sh "$project/sim/run-sim.sh" "Tuning Test" --start -35,-60,0 > "$work/sim.txt" 2>&1 < /dev/null; then
    grep -q "Robot ended at" "$work/sim.txt" || { cat "$work/sim.txt"; fail "$1: the simulator didn't finish the auton"; }
  else
    cat "$work/sim.txt"
    fail "$1: the simulator didn't run"
  fi
}

# A new project without PROS's default libraries, with main.cpp replaced the way the docs say
add_template new-project --no-default-libs
cat > "$work/new-project/src/main.cpp" <<'EOF'
#include "main.h"
#include "rw-config.h"

void initialize() { runPreAutonomous(); }
void disabled() {}
void competition_initialize() {}
void autonomous() { runAutonomous(); }
void opcontrol() { runDriver(); }
EOF
check_project new-project

# A default project (with liblvgl) that keeps PROS's main.cpp and adds RW's three calls to it
add_template existing-project ""
perl -0pi -e 's/(#include "main.h"\n)/$1#include "rw-config.h"\n/; s/(void initialize\(\) \{\n)/$1\trunPreAutonomous();\n/; s/void autonomous\(\) \{\}/void autonomous() { runAutonomous(); }/; s/(void opcontrol\(\) \{\n)/$1\trunDriver();\n/' \
  "$work/existing-project/src/main.cpp"
[ "$(grep -c 'rw-config.h\|runPreAutonomous();\|runAutonomous();\|runDriver();' "$work/existing-project/src/main.cpp")" = 4 ] ||
  fail "existing-project: couldn't add RW's calls to PROS's main.cpp"
check_project existing-project

if [ "$failures" -gt 0 ]; then
  echo "$failures PROS template check(s) failed"
  exit 1
fi
echo "All PROS template checks passed"
