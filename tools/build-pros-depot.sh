#!/bin/sh
# Builds the PROS depot that "pros c add-depot RW-Template .../depot/stable.json" reads: one entry
# for each published release's RW-Template-PROS.zip, with the name, version, target and kernels
# from the template.pros inside it. Releases marked as pre-releases go in beta.json, the rest in
# stable.json. .github/workflows/pros-depot.yml runs this and commits both files to the depot
# branch. Needs the GitHub CLI (signed in, or GH_TOKEN set), curl, unzip and python3.
#   sh tools/build-pros-depot.sh out-folder
set -e
out=$1
if [ -z "$out" ]; then
  echo "Usage: sh tools/build-pros-depot.sh <output folder>"
  exit 1
fi
repo=${GITHUB_REPOSITORY:-richardbwang/RW-Template}
work=$(mktemp -d "${TMPDIR:-/tmp}/rw-depot.XXXXXX")
trap 'rm -rf "$work"' EXIT
mkdir -p "$out"

# Each published release's PROS template: tag, pre-release or not, and download link (newest first)
gh api "repos/$repo/releases" --paginate --jq '.[] | select(.draft | not) | . as $release | .assets[]
  | select(.name == "RW-Template-PROS.zip") | "\($release.tag_name) \($release.prerelease) \(.browser_download_url)"' \
  > "$work/releases.txt"

# The details the PROS CLI needs are in the template.pros inside each zip
: > "$work/templates.txt"
while read -r tag prerelease url; do
  curl -fsSL -o "$work/template.zip" "$url"
  unzip -p "$work/template.zip" template.pros > "$work/$tag.pros"
  echo "$prerelease $url $work/$tag.pros" >> "$work/templates.txt"
done < "$work/releases.txt"

python3 -I - "$work/templates.txt" "$out" <<'EOF'
import json, sys
templates, out = sys.argv[1], sys.argv[2]
depots = {"stable": [], "beta": []}
for line in open(templates):
    prerelease, url, path = line.split()
    state = json.load(open(path))["py/state"]
    entry = {
        "metadata": {"location": url},
        "name": state["name"],
        "py/object": "pros.conductor.templates.base_template.BaseTemplate",
        "supported_kernels": state["supported_kernels"],
        "target": state["target"],
        "version": state["version"],
    }
    # A pre-release, or a version like 4.2.0-beta.1, goes in beta.json
    depots["beta" if prerelease == "true" or "-" in state["version"] else "stable"].append(entry)
for name, entries in depots.items():
    with open(f"{out}/{name}.json", "w") as file:
        json.dump(entries, file, indent=2)
        file.write("\n")
    print(f"{name}.json: {', '.join(e['version'] for e in entries) or 'no templates'}")
EOF
