# SO-101 Real2Sim ピック&プレイス

実機の SO-101 の作業机を 3D Gaussian Splatting でシミュレーター（Isaac Sim）に再現し（Real-to-Sim）、シミュレーターで自動のお手本から集めたデータで学習した方策が実機で動くかを調べました（Sim-to-Real）。Physical AI 応用編1（知能ロボティクス講座 2026 Summer）の最終課題です。

タスクは、積み木をつかんで白い紙皿（直径約 18 cm）に置くことです。

## 結果（概要）

- シミュレーター 225 本で学習した ACT：シミュレーターで 13/20 成功。実機ではそのままでは成功せず、動作の流れ（近づく → 閉じる → 皿へ運ぶ）は出るが、積み木の位置に追従しない。
- 実機のお手本 10 本だけで一から学習した ACT（A）と、シミュレーターの ACT から同じ 10 本で追加学習した ACT（B）を、学習途中のチェックポイントで比べた（定性的な比較）。1000 ステップの時点で、B は積み木を追いかけて閉じたが、A は追従も閉じる動作もまだできなかった。2 万ステップでは、どちらも実機でつかんで皿に置けた。
- 実機で動かすときに効いたこと：開始姿勢（`REST_POSE=start`）、予測し直す間隔（`ACTION_STEPS=30`）、正面カメラ・皿・背景の配置をお手本の収集時にそろえること（`so101_pai/robot/camera_overlay.py`）。

詳しい経過は `docs/WORKLOG.md` にあります。

## 全体の流れ

```
スマホで撮影 → COLMAP → 3DGRUT（3DGUT）→ USDZ 背景 → 机の実測に合わせて配置 ┐
NVIDIA 公式アセット（積み木・皿）＋ SO-101 ───────────────────────────┴→ Isaac Sim 5.1 / Isaac Lab（LeIsaac）
自動のお手本（IK + DART ノイズ）またはリーダーアーム → ROS 2 → rosbag2 → LeRobot v3.0 データセット
  → 学習：ACT ／ SmolVLA（ローカル）／ GR00T N1.7（B300、HF Hub からストリーミング）
  → 推論ノード（シミュレーターと実機で共通）→ 評価：シミュレーター／実機（そのまま）／実機 10 本で追加学習
```

シミュレーターと実機は、同じ ROS 2 トピック（`so101_pai/common/contract.py`）でつながります。そのため、テレオペ・記録・推論・評価のコードは、シミュレーターと実機で共通です。

## ディレクトリ構成

| 場所 | 役割 | 実行環境 |
|---|---|---|
| `so101_pai/common/` | トピックの取り決め（関節角とモーター単位の変換）、エピソードのイベント、ROS メッセージの変換、ノード共通の起動処理、シミュレーター側のエピソード制御 | 両方 |
| `so101_pai/robot/` | 実機ドライバのノード（リーダー、フォロワー、カメラ、テスト用の偽リーダー）、キャリブレーション、評価時の開始姿勢（`rest.py`）、カメラの位置合わせ（`camera_overlay.py`） | `.venv` |
| `so101_pai/teleop/` | テレオペの操作ツール（キー 1 つでエピソードの開始・成功・失敗を送る）と、人の代わりにお手本を作る自動の pick-and-place（`expert.py`） | `.venv`、conda `leisaac` |
| `so101_pai/dataset/` | rosbag2 → LeRobot v3.0 の変換、HF Hub へのアップロード、rosbag2 → MP4（`bag2video.py`） | `.venv` |
| `so101_pai/policy/` | 学習済み方策の推論（`runner.py`）、推論ノード、評価（`evaluation.py`、`eval_node.py`） | `.venv` |
| `so101_pai/recon/` | 3DGS 背景の位置合わせ（`align.py`）、本物の物体の除去（`crop.py`）、USDZ の座標系の修正（`usdz.py`） | conda `3dgrut` |
| `sim/pai_sim/` | Isaac Lab のタスク `PAI-SO101-PlaceBlock-v0` | conda `leisaac` |
| `sim/ros_bridge.py` | シミュレーターを ROS 2 のロボットとして動かすブリッジ | conda `leisaac` |
| `sim/tools/` | 動作確認と計測（`smoke_env.py`、`profile_reset.py`） | conda `leisaac` |
| `scripts/` | 起動スクリプト。`sim/`、`teleop/`、`dataset/`、`train/`、`eval/`、`recon/`、共通処理の `lib/` に分かれる | — |
| `setup/` | 環境構築スクリプト（01〜07） | — |
| `tests/` | pytest（ROS ノードを含め、ハードウェアなしで動く） | `.venv` |
| `docs/WORKLOG.md` | 作業ログ | — |

## 環境構築

システムの Python には何も入れません。ツールごとに環境を分けています（詳しくは `PLAN.md` §2.5）。

```bash
setup/01_leisaac_env.sh      # conda leisaac: Isaac Sim 5.1 + Isaac Lab + LeIsaac（Isaac Sim の EULA への同意が必要）
setup/02_ros_venv.sh         # .venv: ROS 2 Jazzy のノードと LeRobot（~/lerobot）
setup/03_leisaac_assets.sh   # LeIsaac の SO-101 の USD
setup/04_task_assets.sh      # NVIDIA 公式の積み木と皿（公開リポジトリには含めない）
setup/05_colmap_env.sh       # conda colmap
setup/06_3dgrut_env.sh       # conda 3dgrut
setup/07_cloud_train_env.sh  # B300 上の学習専用環境
```

## 使い方

```bash
# 0. ハードウェアなしの通し試験（シミュレーター → rosbag → LeRobot）
scripts/sim/e2e_pipeline.sh

# 1. シミュレーターでテレオペして記録する（キー：s=開始 y=成功 n=失敗 r=リセット q=終了）
scripts/teleop/sim.sh session01                  # リーダーアーム（キャリブレーション名 pai_leader）
FAKE_LEADER=1 scripts/teleop/sim.sh test         # リーダーアームなし
GUI=1 scripts/teleop/sim.sh session02           # Isaac Sim の画面を出す（記録は約 20 Hz に落ちる。標準は画面なし＋カメラ映像 2 窓で 30 Hz）
SEED=10 scripts/teleop/expert.sh expert_main 150  # 人の代わりに自動のお手本で成功 150 エピソードを記録（画面なし、約 40 分）

# 2. データセットを作り、HF Hub に private でアップロードする
scripts/dataset/build.sh sim pai_sim
.venv/bin/python -m so101_pai.dataset.push --repo-id <hf_user>/pai_sim --root data/datasets/pai_sim --yes

# 3. 学習
scripts/train/train.sh smolvla local/pai_sim data/datasets/pai_sim     # SmolVLA（ローカル）
STREAMING=1 scripts/train/train.sh smolvla <hf_user>/pai_sim           # ストリーミング
PYTHON=.venv-train/bin/python STREAMING=1 scripts/train/train.sh groot <hf_user>/pai_sim   # GR00T N1.7（B300）
scripts/train/train.sh act local/pai_sim data/datasets/pai_sim         # ACT（比較用）

# 4. 評価
scripts/eval/sim.sh outputs/train/<run>/checkpoints/last/pretrained_model 20 A_sim
ENABLE_FOLLOWER=1 ACTION_STEPS=30 REST_POSE=start RECORD=1 scripts/eval/real.sh <checkpoint> 10 B_real
#   実機（人が成否を判定、試行の間に r で開始姿勢へ）。起動時に実際に使われた設定を表示する
scripts/dataset/build.sh real pai_real                                  # 実機のお手本（scripts/teleop/real.sh で記録）
INIT=outputs/train/<sim run>/checkpoints/last/pretrained_model scripts/train/train.sh act local/pai_real data/datasets/pai_real   # 実機データで追加学習
.venv/bin/python -m so101_pai.dataset.bag2video data/bags/eval/B_real --out outputs/video   # 評価の記録を試行ごとの MP4 に
PYTHONPATH=$PWD /usr/bin/python3 -m so101_pai.robot.camera_overlay --reference <お手本の正面画像>  # 正面カメラを合わせる
scripts/real/rest.sh                                                   # 実機のアームを初期姿勢へ戻すだけ

# 背景の 3DGS（スマホの動画 → USDZ → 机に位置合わせ → 本物のフォロワーを消す）
scripts/recon/run.sh capture/room.mp4 workspace
CKPT=$(ls outputs/3dgrut/workspace/workspace/*/ckpt_last.pt)
conda run -n 3dgrut python -m so101_pai.recon.align $CKPT assets/scene/workspace.align.json \
    --table-length-cm 90 --table-width-cm 40 --table-height-cm 35 --facing across-far
conda run -n 3dgrut python -m so101_pai.recon.crop $CKPT assets/scene/workspace.align.json ${CKPT%.pt}_crop.pt \
    --box=-0.13,-0.20,0.006,0.08,0.15,0.30 --patch=0.10,-0.09,-0.02,0.31,0.15,0.006:-0.23,0,0
#   → ${CKPT%.pt}_crop.pt を 3DGRUT で書き出し、so101_pai.recon.usdz で assets/scene/workspace_crop.usdz に（WORKLOG 参照）
# シミュレーターは assets/scene/workspace_crop.usdz があれば自動で背景に使う（BACKGROUND=none で無効）
```

アームのキャリブレーション（初回のみ）：

```bash
.venv/bin/lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.id=pai_leader
.venv/bin/lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=pai_follower
```

## テスト

```bash
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m pytest -q           # ハードウェアもシミュレーターも不要
.venv/bin/ruff check so101_pai sim tests
```

## 使用している外部のもの

- [LeIsaac](https://github.com/LightwheelAI/leisaac)（Apache-2.0）：SO-101 の USD と片腕タスクのテンプレート。`sim/pai_sim` のタスクはこれを土台にしています。
- [Isaac Sim 5.1 / Isaac Lab](https://github.com/isaac-sim/IsaacLab)：シミュレーター。
- [LeRobot](https://github.com/huggingface/lerobot)（Apache-2.0）：データセットの形式、SmolVLA・GR00T N1.7・ACT の学習と推論、SO-101 のドライバ。
- [3DGRUT](https://github.com/nv-tlabs/3dgrut)：3D ガウシアンによる背景の再構成。
- アセット：Isaac Sim 公式の `Props/Blocks`、NVIDIA SimReady `dishware_plastic_smallPlate_a01`。ファイルはこのリポジトリに含めず、`setup/04_task_assets.sh` で取得します。

このリポジトリで独自に作ったもの：3DGS 背景の位置合わせと加工（`so101_pai/recon`）、積み木を皿に置くタスクと自動のお手本（IK + DART）、シミュレーターと ROS 2 のブリッジ、ROS 2 のトピックの取り決めと、シミュレーターと実機で共通のノード群、rosbag2 → LeRobot / MP4 の変換、推論・評価の仕組み、起動スクリプト、実機とシミュレーターのずれの調査と修正（手首の回転、カメラ位置）。
