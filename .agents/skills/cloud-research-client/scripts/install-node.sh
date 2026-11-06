#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
mkdir -p "$HP_SIM5_CLOUD_TOOLS/node"
cd "$HP_SIM5_CLOUD_TOOLS/node"

# The index is newest-first and includes non-LTS releases; select the newest LTS.
curl -fsSL https://nodejs.org/dist/index.json -o index.json
version=$(python3 -c '
import json, re, sys
for release in json.load(sys.stdin):
    if release["lts"] and re.fullmatch(r"v\d+\.\d+\.\d+", release["version"]):
        print(release["version"])
        break
else:
    sys.exit("No LTS release found in Node.js distribution index")
' < index.json)
directory="node-$version-linux-x64"
if [[ ! -x "$directory/bin/node" ]]; then
  archive="$directory.tar.xz"
  curl -fsSLO "https://nodejs.org/dist/$version/$archive"
  curl -fsSL "https://nodejs.org/dist/$version/SHASUMS256.txt" -o "SHASUMS256-$version.txt"
  awk -v archive="$archive" '$2 == archive' "SHASUMS256-$version.txt" | sha256sum -c -
  tar -xf "$archive"
fi
ln -sfn "$directory" current
echo "Using Node.js $version (latest LTS)."
