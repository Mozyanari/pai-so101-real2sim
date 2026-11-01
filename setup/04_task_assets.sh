#!/usr/bin/env bash
# Fetch the NVIDIA-official task assets into assets/pai (git-ignored; licenses not cleared for redistribution).
#   - Blocks: Isaac Sim 5.1 asset library, Isaac/Props/Blocks/{red,blue,green,yellow}_block.usd (4.7 cm cubes)
#   - Plates: HF nvidia/simready-assets, dishware_plastic_{big,small}Plate_a01 and dishware_bowl_a01 (flattened into
#     a round plate) (CC-BY-4.0 per package metadata)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$ROOT/assets/pai"
ISAAC="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Props/Blocks"
SIMREADY="https://huggingface.co/datasets/nvidia/simready-assets"
PLATES="dishware_plastic_bigPlate_a01 dishware_plastic_smallPlate_a01 dishware_bowl_a01"

mkdir -p "$DEST/blocks/Materials/Textures"
TEXTURES=$(for c in red1 green1 blue1 yellow; do for k in BaseColor Normal Roughness; do echo "Materials/Textures/basic_block_${c}_${k}.png"; done; done)
for f in basic_block.usd red_block.usd blue_block.usd green_block.usd yellow_block.usd Materials/Materials.usd $TEXTURES; do
  [ -f "$DEST/blocks/$f" ] || curl -sfL -o "$DEST/blocks/$f" "$ISAAC/$f"
done

# Keep source_assets: it holds the OmniPBR.mdl the plate materials point to.
for PLATE in $PLATES; do
  curl -sf "https://huggingface.co/api/datasets/nvidia/simready-assets/tree/main/$PLATE?recursive=true" \
    | python3 -c "import json,sys; [print(x['path']) for x in json.load(sys.stdin) if x['type']=='file']" \
    | grep -vE '/\.thumbs/' \
    | while read -r f; do
        [ -f "$DEST/$f" ] && continue
        mkdir -p "$DEST/$(dirname "$f")"
        curl -sfL -o "$DEST/$f" "$SIMREADY/resolve/main/$f"
      done
done

find "$DEST" -name "*.usd*" -maxdepth 3 | sort
