#!/bin/bash
set -euo pipefail

echo "IPC bridge is no longer needed in the Jazzy single-container flow."
echo "Use quick_start/2_mast3r_slam.sh; MASt3R now subscribes to robot topics through rosbridge."
exec /workspace/thor/quick_start/2_mast3r_slam.sh "$@"
