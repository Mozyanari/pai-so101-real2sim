#!/usr/bin/env bash
# Convert every recorded session of one kind into a single LeRobot dataset.
#
#   scripts/dataset/build.sh sim  pai_sim       # data/bags/sim/*  -> data/datasets/pai_sim  (repo id local/pai_sim)
#   scripts/dataset/build.sh real pai_real      # data/bags/real/* -> data/datasets/pai_real
#
# Only successful episodes are kept (INCLUDE_FAILED=1 keeps all). The dataset is rebuilt from scratch.
# TRIM_IDLE=<motor units> drops each episode's still start (real demos: TRIM_IDLE=2).
# Upload afterwards (private): .venv/bin/python -m so101_pai.dataset.push --repo-id <hf_user>/<name> --root data/datasets/<name> --yes
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"

KIND=${1:?sim | real | eval}
NAME=${2:?dataset name, e.g. pai_sim}
OUT="$ROOT/data/datasets/$NAME"
mapfile -t BAGS < <(find "$ROOT/data/bags/$KIND" -mindepth 1 -maxdepth 1 -type d -exec test -f {}/metadata.yaml \; -print | sort)
[ ${#BAGS[@]} -gt 0 ] || { echo "no finished bags in data/bags/$KIND" >&2; exit 1; }
echo "converting ${#BAGS[@]} session(s): ${BAGS[*]##*/}"

FLAGS=()
[ "${INCLUDE_FAILED:-0}" = "1" ] && FLAGS+=(--include-failed)
[ -n "${TRIM_IDLE:-}" ] && FLAGS+=(--trim-idle-start "$TRIM_IDLE")
rm -rf "$OUT"
ros_env
"$PY" -m so101_pai.dataset.bag2lerobot "${BAGS[@]}" --repo-id "local/$NAME" --root "$OUT" "${FLAGS[@]}"
