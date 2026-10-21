#!/bin/sh
# Tests the updater on throwaway projects. It builds an "old" and a "new" release zip from this
# folder with a few differences, sets up a project from the old one, changes it the way a team
# would, updates it to the new one, and checks that RW's files were updated and the team's weren't.
#   sh tests/test-update.sh                                     tests tools/update.sh
#   RW_TEST_POWERSHELL=pwsh sh tests/test-update.sh             tests tools/update.ps1
#   RW_TEST_POWERSHELL=powershell.exe sh tests/test-update.sh   the same with Windows PowerShell (Git Bash)
#   sh tests/test-update.sh --fixtures <folder>                 only builds the test zips
# Windows has no zip command, so there RW_TEST_FIXTURES=<folder> uses zips built with --fixtures.
set -e
LC_ALL=C
export LC_ALL
cd "$(dirname "$0")/.."
repo=$(pwd)
work=$(mktemp -d "${TMPDIR:-/tmp}/rw-update-test.XXXXXX")
trap 'rm -rf "$work"' EXIT
failures=0

fail() {
  echo "  FAILED: $*"
  failures=$((failures + 1))
}

# A path the updater under test understands (a Windows path for powershell.exe in Git Bash)
native() {
  if command -v cygpath >/dev/null 2>&1; then cygpath -w "$1"; else echo "$1"; fi
}

extract() {
  mkdir -p "$2"
  if command -v unzip >/dev/null 2>&1; then
    unzip -q "$1" -d "$2"
  else
    powershell.exe -NoProfile -Command "Expand-Archive -LiteralPath '$(native "$1")' -DestinationPath '$(native "$2")' -Force"
  fi
}

make_fixtures() {
  mkdir -p "$1"
  sh tools/build-release.sh 0.0.0 "$work/base.zip" > /dev/null
  extract "$work/base.zip" "$work/base"
  # The new version adds two settings
  cp -R "$work/base" "$work/new-src"
  printf '%s\n' 'RW_DEFAULT double test_new_setting = 1.5;   // New in this version' \
    'RW_DEFAULT double test_known_setting = 2;   // Also new, but the team already uses it' >> "$work/new-src/src/utils.cpp"
  # The old version has a different src/pid.cpp, a file the new version removes, and no sim/viewer.html
  cp -R "$work/base" "$work/old-src"
  echo '// old version' >> "$work/old-src/src/pid.cpp"
  echo '// removed in the new version' > "$work/old-src/src/old-feature.cpp"
  rm "$work/old-src/sim/viewer.html"
  sh tools/build-release.sh 9.8.0 "$1/old.zip" "$work/old-src" > /dev/null
  sh tools/build-release.sh 9.9.0 "$1/new.zip" "$work/new-src" > /dev/null
  # A zip from before the updater existed
  mkdir "$work/plain"
  echo 'no updater here' > "$work/plain/README.md"
  (cd "$work/plain" && zip -q -r "$1/plain.zip" .)
}

# Sets up a project from the old zip and changes it the way a team would
team_project() {
  extract "$fixtures/old.zip" "$1"
  echo '// team auton' >> "$1/custom/src/autonomous.cpp"
  echo 'double test_known_setting = 3;' >> "$1/custom/src/robot-config.cpp"
  echo '// team tweak' >> "$1/src/motor-control.cpp"
  echo '// team file' > "$1/src/intake.cpp"
  mkdir -p "$1/notes" "$1/.vscode"
  echo 'team notes' > "$1/notes/plan.txt"
  echo '{ "team": true }' > "$1/.vscode/settings.json"
  # Windows line endings, like a git checkout on Windows makes. Same file, so it's left as it is
  awk '{ printf "%s\r\n", $0 }' "$1/include/path.h" > "$work/crlf" && mv "$work/crlf" "$1/include/path.h"
  mkdir "$1/build"
  echo 'old build' > "$1/build/main.o"
}

run_update() {
  if [ -n "$RW_TEST_POWERSHELL" ]; then
    "$RW_TEST_POWERSHELL" -NoProfile -ExecutionPolicy Bypass -File "$(native "$1/tools/update.ps1")" "$(native "$2")"
  else
    sh "$1/tools/update.sh" "$2"
  fi > "$work/output.txt" 2>&1
}

show_output() {
  sed 's/^/    | /' "$work/output.txt"
}

snapshot() {
  (cd "$1" && find . -type f -exec cksum {} + | sort)
}

# Checks that every RW file in the project is the new version's
check_rw_files() {
  awk 'substr($0, 1, 1) != "#" { print substr($0, 67) }' "$work/new/tools/rw-files.txt" | while IFS= read -r path; do
    if [ "$path" = include/path.h ]; then
      tr -d '\015' < "$1/$path" | cmp -s - "$work/new/$path" || echo "$path"
    else
      cmp -s "$1/$path" "$work/new/$path" || echo "$path"
    fi
  done > "$work/mismatched.txt"
  [ -s "$work/mismatched.txt" ] && fail "not the new version: $(tr '\n' ' ' < "$work/mismatched.txt")"
  cmp -s "$1/tools/rw-files.txt" "$work/new/tools/rw-files.txt" || fail "tools/rw-files.txt isn't the new list"
  return 0
}

if [ "$1" = "--fixtures" ]; then
  make_fixtures "$2"
  echo "Wrote the updater test zips to $2"
  exit 0
fi
if [ -n "$RW_TEST_FIXTURES" ]; then
  fixtures=$(cd "$RW_TEST_FIXTURES" && pwd)
else
  fixtures="$work/fixtures"
  make_fixtures "$fixtures"
fi
extract "$fixtures/new.zip" "$work/new"
if [ -n "$RW_TEST_POWERSHELL" ]; then
  updater="tools/update.ps1 with $RW_TEST_POWERSHELL"
else
  updater="tools/update.sh"
fi
echo "Testing $updater"
grep -q '  \.vscode/tasks\.json$' "$work/new/tools/rw-files.txt" || fail "the zip's file list has no .vscode/tasks.json"

echo "- Updating a project a team has changed"
team="$work/team"
team_project "$team"
cp -R "$team/custom" "$work/custom-before"
run_update "$team" "$fixtures/new.zip" || fail "the update exited with an error"
show_output
check_rw_files "$team"
diff -r "$work/custom-before" "$team/custom" > /dev/null || fail "custom/ changed"
grep -q team "$team/.vscode/settings.json" || fail ".vscode/settings.json was replaced"
grep -q 'team file' "$team/src/intake.cpp" || fail "src/intake.cpp (a team file) changed"
grep -q 'team notes' "$team/notes/plan.txt" || fail "notes/plan.txt (a team file) changed"
[ ! -e "$team/src/old-feature.cpp" ] || fail "src/old-feature.cpp (removed in the new version) is still there"
[ ! -e "$team/build" ] || fail "build/ is still there"
grep -q "$(printf '\r')" "$team/include/path.h" || fail "include/path.h lost its Windows line endings"
backup=$(ls -d "$team"/rw-backup/9.8.0-* 2>/dev/null | head -n 1)
if [ -z "$backup" ]; then
  fail "no backup in rw-backup/9.8.0-*"
else
  grep -q 'team tweak' "$backup/src/motor-control.cpp" 2>/dev/null || fail "the team's src/motor-control.cpp wasn't backed up"
  [ ! -e "$backup/src/pid.cpp" ] || fail "backed up src/pid.cpp, which the team didn't change"
  [ ! -e "$backup/include/path.h" ] || fail "backed up include/path.h, which only has different line endings"
fi
grep -q 'Updating RW-Template 9.8.0 -> 9.9.0' "$work/output.txt" || fail "didn't say which versions it updated between"
grep -q 'double test_new_setting = 1.5;' "$work/output.txt" || fail "didn't list the new setting"
! grep -q test_known_setting "$work/output.txt" || fail "listed a setting custom/ already has"

echo "- Running it again"
snapshot "$team" > "$work/before.txt"
run_update "$team" "$fixtures/new.zip" || fail "the second update exited with an error"
show_output
grep -q 'Already up to date' "$work/output.txt" || fail "didn't say it was up to date"
snapshot "$team" > "$work/after.txt"
cmp -s "$work/before.txt" "$work/after.txt" || fail "changed files when already up to date"

echo "- Updating a project without tools/rw-files.txt (like a git clone)"
team2="$work/team2"
extract "$fixtures/old.zip" "$team2"
rm "$team2/tools/rw-files.txt"
run_update "$team2" "$fixtures/new.zip" || fail "the update exited with an error"
show_output
check_rw_files "$team2"
backup=$(ls -d "$team2"/rw-backup/old-* 2>/dev/null | head -n 1)
[ -n "$backup" ] && [ -f "$backup/src/pid.cpp" ] || fail "didn't back up src/pid.cpp when it couldn't tell if it was changed"
grep -q "can't tell" "$work/output.txt" || fail "didn't explain why it backed files up"

echo "- Using a zip from before the updater existed"
snapshot "$team" > "$work/before.txt"
if run_update "$team" "$fixtures/plain.zip"; then fail "it didn't fail"; fi
show_output
grep -q 'no tools/update' "$work/output.txt" || fail "didn't say the zip has no updater"
snapshot "$team" > "$work/after.txt"
cmp -s "$work/before.txt" "$work/after.txt" || fail "changed files"

if [ "$failures" -gt 0 ]; then
  echo "$failures updater check(s) failed ($updater)"
  exit 1
fi
echo "All updater checks passed ($updater)"
