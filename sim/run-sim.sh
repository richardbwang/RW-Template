#!/bin/sh
# Builds the simulator from your RW-Template code and runs an autonomous routine on a
# simulated robot and field. Writes an HTML replay to sim/out/.
# Usage: sh sim/run-sim.sh "Auton Name" [options]     (sh sim/run-sim.sh --help for options)
set -e
cd "$(dirname "$0")/.."
. sim/find-compiler.sh
mkdir -p sim/build sim/out/sdcard

if [ -f project.pros ]; then
  # A PROS project: your src/ files and RW's code (sim/rw/, since firmware/RW-Template.a only runs on
  # the robot). Your include/ folder is copied to sim/build/include without PROS's own headers, and each
  # of those is replaced with one that includes the simulator's stand-in, sim/pros/pros-sim.h
  rm -rf sim/build/include sim/build/pros
  mkdir -p sim/build/include sim/build/pros/pros
  for entry in include/*; do
    case "$entry" in
      include/api.h|include/pros) ;;
      *) cp -R "$entry" sim/build/include/ ;;
    esac
  done
  for header in include/api.h include/pros/*; do
    case "$header" in
      # Just names and numbers, so the stand-in uses the real ones
      include/pros/error.h|include/pros/colors.h|include/pros/colors.hpp) cp "$header" sim/build/pros/pros/ ;;
      *) echo '#include "pros-sim.h"' > "sim/build/pros/${header#include/}" ;;
    esac
  done
  SOURCES=$(find src -name '*.cpp' ! -path 'src/rw/*' | sort)
  "$CXX" -std=gnu++20 -O2 -w $CXXFLAGS -DRW_SIM_PROS -I sim/build/include -I sim/build/include/rw -I sim/build/pros -I sim/pros -I sim \
    $SOURCES sim/rw/*.cpp sim/sim-engine.cpp sim/sim-main.cpp -o sim/build/rw-sim -pthread
else
  # Everything the robot runs, except src/vexcode/main.cpp (the simulator has its own main)
  SOURCES=""
  for file in src/*.cpp src/vexcode/*.cpp custom/src/*.cpp; do
    [ "$file" = "src/vexcode/main.cpp" ] || SOURCES="$SOURCES $file"
  done
  "$CXX" -std=c++17 -O2 -w $CXXFLAGS -I include -I sim/vex -I sim $SOURCES sim/sim-engine.cpp sim/sim-main.cpp -o sim/build/rw-sim -pthread
fi
sim/build/rw-sim "$@"
