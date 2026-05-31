#!/bin/bash
set -euo pipefail

CACHE_ROOT=${NATIVE_CACHE_ROOT:-/workspace/shared_native_keyframe_cache}
DRY_RUN=${DRY_RUN:-0}

TARGETS=(
  "$CACHE_ROOT/backend/edges"
  "$CACHE_ROOT/robot1"
  "$CACHE_ROOT/robot2"
)

echo "Clearing Native MASt3R shared cache contents..."
echo "  CACHE_ROOT: $CACHE_ROOT"
echo "  DRY_RUN: $DRY_RUN"

for target in "${TARGETS[@]}"; do
  echo "  target: $target"
  mkdir -p "$target"
  if [ "$DRY_RUN" = "1" ]; then
    find "$target" -mindepth 1 -maxdepth 1 -print
  else
    find "$target" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
  fi
done

if [ "$DRY_RUN" = "1" ]; then
  echo "Dry run complete. Re-run without DRY_RUN=1 to delete these entries."
else
  echo "Native cache contents cleared."
fi
