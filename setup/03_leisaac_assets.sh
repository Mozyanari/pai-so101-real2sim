#!/usr/bin/env bash
# Download LeIsaac's SO-101 robot USD and sample scenes into third_party/leisaac/assets.
# https://lightwheelai.github.io/leisaac/docs/getting_started/installation (2. Asset Preparation)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSETS="$ROOT/third_party/leisaac/assets"
mkdir -p "$ASSETS/robots" "$ASSETS/scenes"

download() {  # tag file
  local tmp
  tmp="$(mktemp -d)"
  gh release download "$1" -R LightwheelAI/leisaac -p "$2" -D "$tmp"
  echo "$tmp/$2"
}

[ -f "$ASSETS/robots/so101_follower.usd" ] || mv "$(download v0.1.0 so101_follower.usd)" "$ASSETS/robots/"
for pair in "v0.1.0 kitchen_with_orange" "v0.1.2 table_with_cube"; do
  read -r tag scene <<<"$pair"
  if [ ! -d "$ASSETS/scenes/$scene" ]; then
    unzip -q "$(download "$tag" "$scene.zip")" -d "$ASSETS/scenes/"
  fi
done
find "$ASSETS" -maxdepth 3 | head -20
