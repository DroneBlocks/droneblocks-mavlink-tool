#!/usr/bin/env bash
# Refresh the vendored DEXI param source of truth from dexi-fc-params.
#
# The params are NOT maintained in this repo. Source of truth:
#   https://github.com/DroneBlocks/dexi-fc-params -> src/dexi-3.json
#
# Same idea as firmware/fetch-latest.sh: we keep a committed copy so bench
# provisioning works offline, and re-pull it deliberately. If you need a param
# changed, change it THERE and run this — do not edit params/dexi-3.json.
set -euo pipefail

cd "$(dirname "$0")"
DEST="params/dexi-3.json"
SIBLING="../dexi-fc-params/src/dexi-3.json"
URL="https://raw.githubusercontent.com/DroneBlocks/dexi-fc-params/main/src/dexi-3.json"

mkdir -p params

if [[ -f "$SIBLING" ]]; then
  cp "$SIBLING" "$DEST"
  echo "synced $DEST from $SIBLING"
else
  curl -fsSL "$URL" -o "$DEST"
  echo "synced $DEST from $URL"
fi

python3 - "$DEST" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
n = sum(len(b["params"]) for b in d["blocks"].values())
print(f"  {d['name']} schema v{d['schemaVersion']}: "
      f"{len(d['blocks'])} blocks / {n} params / {len(d['profiles'])} profiles")
for p in d["profiles"]:
    seen = {}
    for b in p["blocks"]:
        for prm in d["blocks"][b]["params"]:
            seen[prm["name"]] = 1
    print(f"    {p['key']:<24} {len(seen)} params")
PY
