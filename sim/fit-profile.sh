#!/bin/sh
# Turns a drive log recorded on the robot (calibrateDrive, the "Drive Calibration" auton) into a
# robot profile, so the simulator drives like your robot. Optional: the simulator works without it.
# Usage: sh sim/fit-profile.sh rw_drive_log.csv [output]     (or --self-test)
set -e
# Keep the log's path working after moving to the project folder
if [ -f "$1" ]; then
  LOG="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
  shift
  set -- "$LOG" "$@"
fi
cd "$(dirname "$0")/.."
. sim/find-compiler.sh
mkdir -p sim/build

"$CXX" -std=c++17 -O2 -w $CXXFLAGS -I sim sim/fit-profile.cpp -o sim/build/fit-profile
sim/build/fit-profile "$@"
