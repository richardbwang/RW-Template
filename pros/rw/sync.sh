#!/bin/sh
# Copies RW's shared code (../src, ../include) and its PROS-only files (rw/) into src/rw and
# include/rw, where the PROS build finds them, and the simulator (../sim) into sim/. pros/Makefile
# runs this before every build and expects "ok". Edit the originals, not the copies: the copies
# are replaced every build.
cd "$(dirname "$0")/.." || exit 1
rm -rf src/rw include/rw sim/rw sim/pros
mkdir -p src/rw include/rw sim/rw sim/pros || exit 1
# -p keeps the files' dates, so make only rebuilds what changed
for file in ../src/*.cpp rw/*.cpp; do
  cp -p "$file" src/rw/ || exit 1
done
for file in ../include/*.h rw/*.h; do
  case "$file" in
    ../include/vex.h) ;;  # VEXcode only
    *) cp -p "$file" include/rw/ || exit 1 ;;
  esac
done
# The simulator runs RW's code from sim/rw, since RW-Template.a only runs on the robot. sim/out,
# sim/build and sim/robot-profile.txt stay
for file in ../sim/*.sh ../sim/*.cpp ../sim/*.h ../sim/*.html ../sim/.gitignore; do
  cp -p "$file" sim/ || exit 1
done
cp -p ../sim/pros/*.h sim/pros/ || exit 1
for file in ../src/*.cpp rw/*.cpp; do
  cp -p "$file" sim/rw/ || exit 1
done
echo ok
