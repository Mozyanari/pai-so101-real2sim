# SO-101 Real2Sim ピック&プレイス

実機の作業机をシミュレーションで再現し、シミュレーター内のテレオペで集めたデータだけでピック&プレイスを学習します。その方策が実機の SO-101 でゼロショットで動くかを調べます。Physical AI 応用編1（知能ロボティクス講座 2026 Summer）の最終課題です。

タスクは、赤い積み木（4.7 cm 角）をつかんで白い紙皿（直径約 18 cm）に置き、手を離すことです。

## 全体の流れ

```
スマホで撮影 → COLMAP → 3DGRUT（3DGUT）→ USDZ 背景 ┐
NVIDIA 公式アセット（積み木・皿）＋ SO-101 ──────────┴→ Isaac Sim 5.1 / Isaac Lab（LeIsaac）
リーダーアーム → ROS 2 → シミュレーター ⇄ ROS 2 → rosbag2 → LeRobot v3.0 データセット → HF Hub（private）
  → 学習：SmolVLA（ローカル）／ GR00T N1.7（B300、HF Hub からストリーミング）／ ACT（比較用）
  → 推論ノード（シミュレーターと実機で共通）→ 評価：A シミュレーター／B 実機ゼロショット／C 実機データを少し追加
```

シミュレーターと実機は、同じ ROS 2 トピック（`so101_pai/common/contract.py`）でつながります。そのため、テレオペ・記録・推論・評価のコードは、シミュレーターと実機で共通です。

## ディレクトリ構成

| 場所 | 役割 | 実行環境 |
|---|---|---|
| `so101_pai/common/` | トピックの取り決め（関節角とモーター単位の変換）、エピソードのイベント、ROS メッセージの変換、ノード共通の起動処理、シミュレーター側のエピソード制御 | 両方 |
| `so101_pai/robot/` | 実機ドライバのノード（リーダー、フォロワー、カメラ、テスト用の偽リーダー）、キャリブレーション | `.venv` |
| `so101_pai/teleop/` | テレオペの操作ツール（キー 1 つでエピソードの開始・成功・失敗を送る） | `.venv` |
| `so101_pai/dataset/` | rosbag2 → LeRobot v3.0 の変換、HF Hub へのアップロード | `.venv` |
| `so101_pai/policy/` | 学習済み方策の推論（`runner.py`）、推論ノード、評価（`evaluation.py`、`eval_node.py`） | `.venv` |
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
ENABLE_FOLLOWER=1 scripts/eval/real.sh <checkpoint> 20 B_real          # 実機（人が成否を判定）

# 背景の 3DGS（スマホの動画 → USDZ）
scripts/recon/run.sh ~/capture/room.mp4 workspace
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

このリポジトリで独自に作ったもの：ROS 2 のトピックの取り決めと、シミュレーターと実機で共通のノード群、rosbag2 → LeRobot の変換、シミュレーターのブリッジとタスク、評価の仕組み、起動スクリプト。
