#!/bin/sh
# Builds and runs the computer-side tests: odometry, distance reset, path and motion profile math,
# the MCL field simulation, and the simulator's calibration fitter.
# These don't run on the robot; they need a regular C++ compiler. Run: sh tests/run-tests.sh
set -e
cd "$(dirname "$0")/.."
. sim/find-compiler.sh
OUT="${TMPDIR:-/tmp}/rw-template-tests"
mkdir -p "$OUT"

"$CXX" -std=c++14 -O2 -Wall $CXXFLAGS -I include src/utils.cpp tests/test_math.cpp -o "$OUT/test_math"
"$CXX" -std=c++14 -O2 -Wall $CXXFLAGS -I include src/utils.cpp src/mcl.cpp tests/sim_mcl.cpp -o "$OUT/sim_mcl"
"$CXX" -std=c++14 -O2 -Wall $CXXFLAGS -I include src/utils.cpp src/path.cpp tests/test_path.cpp -o "$OUT/test_path"
"$CXX" -std=c++17 -O2 -w $CXXFLAGS -I sim sim/fit-profile.cpp -o "$OUT/fit_profile"
"$OUT/test_math"
"$OUT/test_path"
"$OUT/sim_mcl"
"$OUT/fit_profile" --self-test | tail -1
