#!/usr/bin/env bash
# Write the run conditions on the project video clips: outputs/video/*.mp4 -> outputs/video/captioned/*.mp4.
#
#   scripts/video/caption_all.sh
#
# The clips come from so101_pai.dataset.bag2video (real and sim evaluations) and sim/tools/orbit_video.py.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
IN=outputs/video
OUT=$IN/captioned
mkdir -p "$OUT"

cap() {  # cap <in name> <out name> <speed> <line>...
  local src=$1 dst=$2 speed=$3
  shift 3
  local args=()
  for line in "$@"; do args+=(--text "$line"); done
  [ -f "$IN/$src" ] || { echo "skip (missing): $src" >&2; return; }
  .venv/bin/python -m so101_pai.dataset.caption_video "$IN/$src" "$OUT/$dst" --speed "$speed" "${args[@]}"
}

result() { [[ $1 == *success* ]] && echo 成功 || echo 失敗; }

A_COND="A：実機のお手本 10 本だけで学習"
B_COND="B：シミュレーター 225 本で事前学習 → 実機のお手本 10 本で追加学習"
for model in A B; do
  cond=A_COND
  [ $model = B ] && cond=B_COND
  for k in 1 5 10 20; do
    for f in "$IN"/${model}${k}k_video_ep[0-2]_*.mp4; do
      [ -f "$f" ] || continue
      name=$(basename "$f")
      ep=${name#*_ep}
      ep=${ep%%_*}
      cap "$name" "$name" 1 "実機｜ACT $model（${k}000 ステップ）" "${!cond}" "試行 $((ep + 1))：$(result "$name")｜等倍"
    done
  done
done

cap B20k_positions_ep0_success.mp4 B20k_positions_2x.mp4 2 \
  "実機｜ACT B（2 万ステップ）" "$B_COND" "積み木の場所を手で変えながら連続で実行｜2 倍速"

cap C_ver_01_ep0_fail.mp4 C_ver_01_ep0_fail.mp4 1 \
  "実機｜ACT C" "C：シミュレーター 225 本だけで学習（実機のデータなし）" "試行 1：失敗｜等倍"

for f in "$IN"/A_sim_act_v5_video_ep*.mp4; do
  name=$(basename "$f")
  ep=${name#*_ep}
  ep=${ep%%_*}
  cap "$name" "sim_act_ep${ep}_$(result "$name" | sed 's/成功/success/;s/失敗/fail/').mp4" 1 \
    "シミュレーター｜ACT C" "シミュレーター 225 本だけで学習（4 万ステップ）" "試行 $((ep + 1))：$(result "$name")｜等倍"
done

cap sim_scene_orbit.mp4 sim_scene_orbit.mp4 1 \
  "シミュレーター環境（Isaac Sim）" "机・壁・棚：実機の机を撮影して 3DGS で再現｜ロボット・皿・積み木：3D モデル" "等倍"
cap sim_scene_orbit_pick.mp4 sim_scene_orbit_pick.mp4 1 \
  "シミュレーター環境（Isaac Sim）" "腕は自動のお手本（積み木の位置から IK で動かす）。学習データはこの動きで集めた" "等倍"
