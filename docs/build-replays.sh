#!/bin/sh
# Runs the example autons in the simulator and copies their replays to docs/replays/ for the
# docs site. The docs workflow runs this before building; run it yourself before `mkdocs serve`.
# Usage: sh docs/build-replays.sh
set -e
cd "$(dirname "$0")/.."
mkdir -p docs/replays

for name in "Path Example" "Profiled Example" "Async Example" "Tuning Test" "Odom Test"; do
  file="$(printf '%s' "$name" | tr ' ' '-').html"
  sh sim/run-sim.sh "$name" --out "docs/replays/$file" < /dev/null
done
