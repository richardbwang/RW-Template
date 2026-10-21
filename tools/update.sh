#!/bin/sh
# Updates this project to the newest RW-Template release. Your custom folder stays as it is.
#   sh tools/update.sh                    downloads the newest release from GitHub
#   sh tools/update.sh RW-Template.zip    uses a release zip you already downloaded
# In VS Code you can also run it from Terminal > Run Task > Update RW-Template.
# tools/update.ps1 does the same on Windows.
#
# It replaces RW's own files (listed in tools/rw-files.txt) and leaves everything else alone:
# custom/, your .vscode settings, and any files you added. If you changed one of RW's files,
# your copy is saved in rw-backup/ before it's replaced.
#
# After downloading, it runs the new release's copy of this script to do the update, so changes
# to the update steps work right away. Older versions run it as
#   sh tools/update.sh --apply <new release folder> <project folder> <temp folder>
# so keep those arguments working.

REPO_URL="https://github.com/richardbwang/RW-Template"
DOCS_URL="https://richardbwang.github.io/RW-Template/getting-started/installation/"

fail() {
  echo "Update failed: $*" >&2
  exit 1
}

is_project() {
  [ -d "$1/custom" ] && [ -f "$1/makefile" ]
}

# SHA-256 of a file, ignoring CR characters so Windows line endings don't count as a change
if command -v sha256sum >/dev/null 2>&1; then
  hash_file() { tr -d '\015' < "$1" | sha256sum | cut -c1-64; }
else
  hash_file() { tr -d '\015' < "$1" | shasum -a 256 | cut -c1-64; }
fi

download() {
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL -o "$2" "$1"
  elif command -v wget >/dev/null 2>&1; then
    wget -q -O "$2" "$1"
  else
    echo "This needs curl or wget to download." >&2
    return 1
  fi
}

# The folder of the unzipped release that has tools/update.sh: the zip's top level or one folder down
find_release() {
  for dir in "$1" "$1"/*; do
    if [ -f "$dir/tools/update.sh" ]; then
      echo "$dir"
      return 0
    fi
  done
  return 1
}

version_of() {
  sed -n '1s/^# RW-Template \([A-Za-z0-9._-]*\).*/\1/p' "$1"
}

# The "hash  path" lines of a file list, without comments or any path the updater must never touch
file_list() {
  awk 'length($1) == 64 && $1 ~ /^[0-9a-f]+$/ && substr($0, 65, 2) == "  " {
    path = substr($0, 67)
    if (path == "" || path ~ /^\// || path ~ /\\/ || path ~ /^[A-Za-z]:/) next
    if (("/" path "/") ~ /\/\.\.?\//) next
    if (path ~ /^(custom|\.git|rw-backup)\// || path == "tools/rw-files.txt") next
    print
  }' "$1"
}

# The lines in a folder's src/ that give a setting its default (RW_DEFAULT)
default_lines() {
  if [ -d "$1/src" ]; then
    find "$1/src" -name '*.cpp' | sort | while IFS= read -r file; do
      grep '^RW_DEFAULT[[:space:]]' "$file"
    done
  fi
  return 0
}

setting_names() {
  sed -n 's/^RW_DEFAULT[[:space:]]*[A-Za-z_][A-Za-z0-9_]*[[:space:]]*\([A-Za-z_][A-Za-z0-9_]*\).*/\1/p'
}

# Step 1: get the new release, then hand over to its copy of this script
fetch() {
  project=$(cd "$(dirname "$0")/.." && pwd) || exit 1
  is_project "$project" || fail "$project doesn't look like an RW-Template project (it needs a custom folder and a makefile)."
  tmp=$(mktemp -d "${TMPDIR:-/tmp}/rw-update.XXXXXX") || fail "couldn't make a temporary folder."
  trap 'rm -rf "$tmp"' EXIT
  if [ -n "$1" ]; then
    [ -f "$1" ] || fail "can't find $1."
    zip=$1
  else
    zip="$tmp/RW-Template.zip"
    echo "Downloading the newest RW-Template from GitHub..."
    download "$REPO_URL/releases/latest/download/RW-Template.zip" "$zip" ||
      fail "couldn't download it. Check your internet connection and try again (right after a release comes out, give it a few minutes)."
  fi
  command -v unzip >/dev/null 2>&1 || fail "this needs unzip (on Linux: sudo apt install unzip)."
  unzip -q "$zip" -d "$tmp/new" || fail "couldn't unzip $zip."
  release=$(find_release "$tmp/new") ||
    fail "this release can't update projects (it has no tools/update.sh). Projects from 4.0.0 and older update by hand: $DOCS_URL"
  trap - EXIT
  exec sh "$release/tools/update.sh" --apply "$release" "$project" "$tmp"
}

# Step 2, run from the new release: update the project's files
apply() {
  release=$(cd "$1" && pwd) || fail "can't find the new release in $1."
  project=$2
  tmp=$3
  work=$(mktemp -d "${TMPDIR:-/tmp}/rw-update.XXXXXX") || fail "couldn't make a temporary folder."
  trap 'rm -rf "$work"; [ -z "$tmp" ] || rm -rf "$tmp"' EXIT
  is_project "$project" || fail "$project doesn't look like an RW-Template project (it needs a custom folder and a makefile)."
  cd "$project" || fail "couldn't open $project."
  if [ -f tools/build-release.sh ]; then
    fail "this is a copy of the RW-Template repository (it has tools/build-release.sh), and updating would overwrite your work in it. Use git pull instead, or delete tools/build-release.sh to update anyway."
  fi
  [ -f "$release/tools/rw-files.txt" ] || fail "the new release has no tools/rw-files.txt."

  new_version=$(version_of "$release/tools/rw-files.txt")
  old_version=""
  if [ -f tools/rw-files.txt ]; then
    old_version=$(version_of tools/rw-files.txt)
  fi
  if [ -n "$old_version" ] && [ "$old_version" = "$new_version" ]; then
    echo "Already up to date: this project has RW-Template $new_version."
    exit 0
  fi

  # RW's files in the new and old versions, then the project's copies of them right now
  file_list "$release/tools/rw-files.txt" > "$work/new.txt"
  if [ -f tools/rw-files.txt ]; then
    file_list tools/rw-files.txt > "$work/old.txt"
  else
    : > "$work/old.txt"
  fi
  cut -c67- "$work/new.txt" "$work/old.txt" | sort -u | while IFS= read -r path; do
    if [ -f "$path" ]; then
      printf '%s  %s\n' "$(hash_file "$path")" "$path"
    fi
  done > "$work/current.txt"

  # What to do with each file. A copy that matches the old version is RW's, so it's replaced
  # (or removed) as is; anything else is saved to rw-backup first. Removals go first, so a file
  # that only changed case on a Mac isn't deleted after its new version is copied in.
  awk -v newlist="$work/new.txt" -v oldlist="$work/old.txt" '
    { hash = substr($0, 1, 64); path = substr($0, 67) }
    FILENAME == newlist { new_hash[path] = hash; order[++count] = path; next }
    FILENAME == oldlist { old_hash[path] = hash; next }
    { current_hash[path] = hash }
    END {
      for (path in old_hash) {
        if ((path in new_hash) || !(path in current_hash)) continue
        action = current_hash[path] == old_hash[path] ? "remove" : "remove-backup"
        print action, path
      }
      for (i = 1; i <= count; i++) {
        path = order[i]
        if (!(path in current_hash)) print "add", path
        else if (current_hash[path] != new_hash[path]) {
          action = (path in old_hash) && current_hash[path] == old_hash[path] ? "update" : "update-backup"
          print action, path
        }
      }
    }' "$work/new.txt" "$work/old.txt" "$work/current.txt" > "$work/plan.txt"

  if [ ! -s "$work/plan.txt" ]; then
    mkdir -p tools && cp "$release/tools/rw-files.txt" tools/rw-files.txt
    echo "Already up to date: this project has RW-Template $new_version."
    exit 0
  fi
  if [ -n "$old_version" ]; then
    echo "Updating RW-Template $old_version -> $new_version"
  else
    echo "Updating to RW-Template $new_version"
  fi

  # Settings with defaults before the update, to list the new ones at the end
  default_lines . | setting_names > "$work/old-settings.txt"

  backup="rw-backup/${old_version:-old}-$(date +%Y%m%d-%H%M%S)"
  updated=0
  added=0
  removed=0
  : > "$work/saved.txt"
  : > "$work/failed.txt"
  while IFS=' ' read -r action path; do
    case $action in
      *-backup)
        if [ -f "$path" ]; then
          if mkdir -p "$backup/$(dirname "$path")" && cp -p "$path" "$backup/$path"; then
            echo "$path" >> "$work/saved.txt"
          else
            echo "$path" >> "$work/failed.txt"
            continue
          fi
        fi ;;
    esac
    case $action in
      remove*)
        if rm -f "$path"; then
          removed=$((removed + 1))
          rmdir -p "$(dirname "$path")" 2>/dev/null
        else
          echo "$path" >> "$work/failed.txt"
        fi ;;
      *)
        if mkdir -p "$(dirname "$path")" && cp "$release/$path" "$path"; then
          if [ "$action" = add ]; then added=$((added + 1)); else updated=$((updated + 1)); fi
        else
          echo "$path" >> "$work/failed.txt"
        fi ;;
    esac
  done < "$work/plan.txt"

  if [ -s "$work/failed.txt" ]; then
    echo "Couldn't update these files:"
    sed 's/^/  /' "$work/failed.txt"
    fail "close any program that has them open, then run the update again."
  fi
  # Saved last, so an update that stopped partway runs again from the old version
  mkdir -p tools && cp "$release/tools/rw-files.txt" tools/rw-files.txt || fail "couldn't save tools/rw-files.txt."
  # Start the next build from scratch, so nothing built from the old version is left over
  rm -rf build

  echo
  echo "Files updated: $updated, added: $added, removed: $removed. Your custom folder wasn't changed."
  if [ -s "$work/saved.txt" ]; then
    echo
    if [ -n "$old_version" ]; then
      echo "You had changed these RW files. They have the new versions now, and your copies are in $backup:"
    else
      echo "These files changed. Without tools/rw-files.txt the updater can't tell whether you edited them, so your copies are in $backup:"
    fi
    sed 's/^/  /' "$work/saved.txt"
  fi

  # New settings: defaults that weren't there before and that custom/ doesn't mention yet
  : > "$work/settings.txt"
  default_lines "$release" | while IFS= read -r line; do
    name=$(printf '%s\n' "$line" | setting_names)
    [ -n "$name" ] || continue
    grep -qxF "$name" "$work/old-settings.txt" && continue
    grep -rqw -- "$name" custom 2>/dev/null && continue
    printf '%s\n' "$line" | sed 's/^RW_DEFAULT[[:space:]]*/  /'
  done > "$work/settings.txt"
  if [ -s "$work/settings.txt" ]; then
    echo
    echo "New settings in $new_version. They work at their defaults; to change one, copy its line into custom/src/robot-config.cpp:"
    cat "$work/settings.txt"
  fi

  echo
  echo "What's new: $REPO_URL/releases/tag/$new_version"
  echo "Done. Build and download as usual. The first build takes longer because it rebuilds everything."
}

main() {
  LC_ALL=C
  export LC_ALL
  if [ "$1" = "--apply" ]; then
    apply "$2" "$3" "$4"
  else
    fetch "$1"
  fi
}

# On one line, so the shell has read all of this before the update replaces this file
main "$@"; exit $?
