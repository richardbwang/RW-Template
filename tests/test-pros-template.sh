#!/bin/sh
# Installs the PROS template (from tools/build-pros-template.sh) into a new PROS project the way a
# team would, builds that project, checks that RW links with the team's code, and runs an auton in
# the simulator. Needs the ARM toolchain, the PROS CLI, a C++ compiler, and internet access for the
# PROS kernel.
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

echo "- Making a new PROS project and adding the template to it"
pros c new-project "$work/team" v5 4.2.2 --no-default-libs > "$work/log.txt" 2>&1 || { cat "$work/log.txt"; exit 1; }
cd "$work/team"
cp "$zip" .
pros c fetch "$(basename "$zip")" > "$work/log.txt" 2>&1 || { cat "$work/log.txt"; exit 1; }
# The exact version in the zip (RW-Template@4.1.0.zip), not a newer one the PROS CLI already has
pros c apply "$(basename "$zip" .zip)" > "$work/log.txt" 2>&1 || { cat "$work/log.txt"; exit 1; }
for file in src/rw-config.cpp src/rw-autons.cpp src/rw-user.cpp include/rw-config.h include/rw/api.h firmware/RW-Template.a \
    sim/run-sim.sh sim/pros/pros-sim.h sim/rw/hardware.cpp sim/rw/motor-control.cpp; do
  [ -f "$file" ] || fail "the template didn't add $file"
done

# What the docs tell teams to put in main.cpp
cat > src/main.cpp <<'EOF'
#include "main.h"
#include "rw-config.h"

void initialize() { runPreAutonomous(); }
void disabled() {}
void competition_initialize() {}
void autonomous() { runAutonomous(); }
void opcontrol() { runDriver(); }
EOF

echo "- Building it"
if ! make > "$work/log.txt" 2>&1; then
  cat "$work/log.txt"
  fail "the project didn't build"
fi
# RW uses the team's devices by name, so it has to link with the team's code (the hot package)
if [ -f bin/hot.package.elf ]; then
  arm-none-eabi-nm -C bin/hot.package.elf | grep -q ' T driveTo(' || fail "RW isn't in the hot package"
  ! arm-none-eabi-nm -C bin/cold.package.elf | grep -q ' T driveTo(' || fail "RW is in the cold package"
fi

echo "- Running an auton in the simulator"
if sh sim/run-sim.sh "Tuning Test" --start -35,-60,0 > "$work/sim.txt" 2>&1 < /dev/null; then
  grep -q "Robot ended at" "$work/sim.txt" || { cat "$work/sim.txt"; fail "the simulator didn't finish the auton"; }
else
  cat "$work/sim.txt"
  fail "the simulator didn't run"
fi

if [ "$failures" -gt 0 ]; then
  echo "$failures PROS template check(s) failed"
  exit 1
fi
echo "All PROS template checks passed"
