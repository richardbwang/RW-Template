#!/bin/sh
# Builds the simulator from your RW-Template code and runs an autonomous routine on a
# simulated robot and field. Writes an HTML replay to sim/out/.
# Usage: sh sim/run-sim.sh "Auton Name" [options]     (sh sim/run-sim.sh --help for options)
set -e
cd "$(dirname "$0")/.."
. sim/find-compiler.sh
mkdir -p sim/build sim/out/sdcard

# Everything the robot runs, except src/main.cpp (the simulator has its own main)
SOURCES=""
for file in src/*.cpp custom/src/*.cpp; do
  [ "$file" = "src/main.cpp" ] || SOURCES="$SOURCES $file"
done
"$CXX" -std=c++17 -O2 -w $CXXFLAGS -I include -I sim/vex -I sim $SOURCES sim/sim-engine.cpp sim/sim-main.cpp -o sim/build/rw-sim -pthread
sim/build/rw-sim "$@"
