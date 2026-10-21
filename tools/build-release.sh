#!/bin/sh
# Builds RW-Template.zip, the zip teams download from Releases. It has every file git would commit
# (tracked, or new and not ignored) except the repo-only ones below, plus tools/rw-files.txt: the
# list of RW's files that tools/update.sh and tools/update.ps1 use to update a project.
# .github/workflows/release.yml runs this for every release. To run it yourself:
#   sh tools/build-release.sh 4.1.0                    writes build/RW-Template.zip
#   sh tools/build-release.sh 4.1.0 out.zip            writes out.zip
#   sh tools/build-release.sh 4.1.0 out.zip folder     packages a folder as it is (for tests/test-update.sh)
set -e
LC_ALL=C
export LC_ALL

# Repo-only files that stay out of teams' projects: the PROS version (its own download, see
# tools/build-pros-template.sh) and its simulator stand-in, the docs site, the release tools and
# tests, the images for the GitHub page, and any zips lying around (like an RW-Template.zip built here)
repo_only() {
  case $1 in
    pros/* | sim/pros/* | tools/build-pros-template.sh | tests/test-pros-template.sh | tests/test-drive-types.sh | \
    docs/* | mkdocs.yml | .github/workflows/docs.yml | .github/workflows/release.yml | \
    tools/build-release.sh | tests/test-update.sh | assets/logo.png | assets/social-preview.* | *.zip) return 0 ;;
  esac
  return 1
}

# Files that are the team's once a project is set up, so the updater never replaces them
team_file() {
  case $1 in
    .vscode/tasks.json) return 1 ;;
    custom/* | .vscode/* | tools/rw-files.txt) return 0 ;;
  esac
  return 1
}

# SHA-256 ignoring CR characters, the same as in tools/update.sh
if command -v sha256sum >/dev/null 2>&1; then
  hash_file() { tr -d '\015' < "$1" | sha256sum | cut -c1-64; }
else
  hash_file() { tr -d '\015' < "$1" | shasum -a 256 | cut -c1-64; }
fi

version=$1
if [ -z "$version" ]; then
  echo "Usage: sh tools/build-release.sh <version> [output.zip] [folder]"
  exit 1
fi
command -v zip >/dev/null 2>&1 || { echo "This needs the zip command."; exit 1; }
caller=$(pwd)
absolute() {
  case $1 in
    /*) echo "$1" ;;
    *) echo "$caller/$1" ;;
  esac
}
cd "$(dirname "$0")/.."
if [ -n "$2" ]; then out=$(absolute "$2"); else out="$(pwd)/build/RW-Template.zip"; fi

stage=$(mktemp -d "${TMPDIR:-/tmp}/rw-release.XXXXXX")
trap 'rm -rf "$stage"' EXIT
files="$stage/files"
mkdir "$files"
if [ -n "$3" ]; then
  cp -R "$(absolute "$3")/." "$files/"
else
  git ls-files --cached --others --exclude-standard | while IFS= read -r path; do
    [ -f "$path" ] || continue
    repo_only "$path" && continue
    mkdir -p "$files/$(dirname "$path")"
    cp "$path" "$files/$path"
  done
fi

mkdir -p "$files/tools"
{
  echo "# RW-Template $version"
  echo "# RW's own files in this version. tools/update.sh and tools/update.ps1 use this list to update"
  echo "# a project: they replace these files and leave everything else alone. Each line is a file's"
  echo "# SHA-256 (ignoring CR characters, so Windows line endings don't count as a change) and its path."
  (cd "$files" && find . -type f | sed 's|^\./||' | sort) | while IFS= read -r path; do
    team_file "$path" && continue
    printf '%s  %s\n' "$(hash_file "$files/$path")" "$path"
  done
} > "$stage/rw-files.txt"
mv "$stage/rw-files.txt" "$files/tools/rw-files.txt"

mkdir -p "$(dirname "$out")"
rm -f "$out"
(cd "$files" && zip -q -X -r "$out" .)
count=$(grep -c -v '^#' "$files/tools/rw-files.txt" || true)
echo "Wrote $out (RW-Template $version, $count RW files)"
