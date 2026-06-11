#!/bin/bash
# Re-pull the latest DroneBlocks H743-AIO bootloader + app firmware from the
# cloud GHA build, replacing the pinned files in droneblocks-h743-aio/.
set -euo pipefail
base="https://pub-a9128812de294697bc4f590727d409c8.r2.dev/droneblocks_h743-aio/latest"
cd "$(dirname "$0")/droneblocks-h743-aio"
echo "fetching from $base …"
curl -fsSL -o manifest.json                         "$base/manifest.json"
curl -fsSL -o droneblocks_h743-aio_bootloader.bin   "$base/droneblocks_h743-aio_bootloader.bin"
curl -fsSL -o droneblocks_h743-aio_default.px4       "$base/droneblocks_h743-aio_default.px4"
python3 -c "import json;d=json.load(open('manifest.json'));print('pinned now:', d['version'], '| git', d['git_sha'], '| board_id', d['board_id'])"
echo "done — commit the updated files to record the new pin."
