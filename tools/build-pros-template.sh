#!/bin/sh
# Builds RW-Template@<version>.zip, the PROS template that "pros c apply RW-Template" installs,
# from the PROS project in pros/. Needs the ARM toolchain (arm-none-eabi-g++) and the PROS CLI.
# .github/workflows/release.yml runs this for every release. To run it yourself:
#   sh tools/build-pros-template.sh 4.1.0              writes pros/RW-Template@4.1.0.zip
#   sh tools/build-pros-template.sh 4.1.0 out-folder   writes out-folder/RW-Template@4.1.0.zip
set -e
version=$1
if [ -z "$version" ]; then
  echo "Usage: sh tools/build-pros-template.sh <version> [output folder]"
  exit 1
fi
command -v arm-none-eabi-g++ >/dev/null 2>&1 || { echo "This needs the ARM toolchain (arm-none-eabi-g++) on your PATH."; exit 1; }
command -v pros >/dev/null 2>&1 || { echo "This needs the PROS CLI (pip install pros-cli)."; exit 1; }
out=""
if [ -n "$2" ]; then
  out=$(mkdir -p "$2" && cd "$2" && pwd)
fi
cd "$(dirname "$0")/../pros"

# A clean build, so nothing from an earlier build ends up in the library
rm -f RW-Template@*.zip
make clean > /dev/null
make template VERSION="$version"
zip="RW-Template@$version.zip"
[ -f "$zip" ] || { echo "make template didn't write $zip"; exit 1; }
if [ -n "$out" ]; then
  mv "$zip" "$out/"
  echo "Wrote $out/$zip"
else
  echo "Wrote pros/$zip"
fi
