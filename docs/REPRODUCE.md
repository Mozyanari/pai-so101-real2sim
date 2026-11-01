# 再現手順

レポートと動画の結果（シミュレーターで学習した ACT、実機のお手本 10 本での追加学習 A／B、実機評価、動画）を、もう一度作るための手順です。コマンドは、実際に使った設定（ログに残っているもの）のとおりに書いています。経過や試行錯誤は `docs/WORKLOG.md` にあります。

## 0. 前提

- 機材：SO-101（リーダー `/dev/ttyACM0`・フォロワー `/dev/ttyACM1`）、正面カメラ C920（`/dev/video0`）、手先カメラ（`/dev/video2`）、RTX 5060 Ti 16 GB、Ubuntu 24.04 + ROS 2 Jazzy。
- 物：木の積み木（約 3 cm 角）、白い紙皿（直径約 18 cm）、机（90 × 40 cm、高さ 35 cm）。
- リポジトリに入っていないもの（自分で作るか取得する）：アセット（`setup/04_task_assets.sh`）、3DGS の背景（`assets/scene/*.usdz`）、rosbag、データセット、学習済みモデル、動画。
- システムの Python には何も入れない。ツールごとに環境を分ける（`docs/DESIGN.md` §3）。
- Isaac Sim は ROS を source していないシェルから起動する（`scripts/sim/run.sh` やシミュレーター用のスクリプトが処理する）。

## 1. 環境構築

```bash
setup/01_leisaac_env.sh      # conda leisaac：Isaac Sim 5.1 + Isaac Lab + LeIsaac（Isaac Sim の EULA への同意が必要）
setup/02_ros_venv.sh         # .venv：ROS 2 のノードと LeRobot v0.6.1（~/lerobot）
setup/03_leisaac_assets.sh   # SO-101 の USD
setup/04_task_assets.sh      # NVIDIA 公式の積み木と皿
setup/05_colmap_env.sh       # conda colmap
setup/06_3dgrut_env.sh       # conda 3dgrut
```

アームのキャリブレーション（初回のみ）：

```bash
.venv/bin/lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.id=pai_leader
.venv/bin/lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=pai_follower
```

動作確認：

```bash
source /opt/ros/jazzy/setup.bash && .venv/bin/python -m pytest -q   # ハードウェア・シミュレーターなしで通る
scripts/sim/e2e_pipeline.sh                                          # シミュレーター → rosbag → LeRobot の通し試験（約 2 分）
```

## 2. 実機の机をシミュレーターに再現する（3DGS の背景）

1. 机のまわりをスマホやカメラで動画に撮る（ぶれの少ない約 350 枚が取れるように、ゆっくり一周）。
2. COLMAP → 3DGRUT（3DGUT）で 3DGS を作る：

   ```bash
   scripts/recon/run.sh capture/room.mp4 workspace
   CKPT=$(ls outputs/3dgrut/workspace/workspace/*/ckpt_last.pt)
   ```

3. 天板を見つけて、実測した机の寸法でロボットの根元を原点とする座標に置く：

   ```bash
   conda run -n 3dgrut python -m so101_pai.recon.align $CKPT assets/scene/workspace.align.json \
       --table-length-cm 90 --table-width-cm 40 --table-height-cm 35 --facing across-far
   ```

4. 3DGS に写っている本物のフォロワーを箱で消し、穴を手前の天板で埋める（最終版はテープが写らない 2 本の帯でコピー）：

   ```bash
   conda run -n 3dgrut python -m so101_pai.recon.crop $CKPT assets/scene/workspace.align.json ${CKPT%.pt}_crop.pt \
       --box=-0.13,-0.20,0.006,0.08,0.15,0.30 \
       --patch=0.10,-0.09,-0.02,0.31,0.095,0.006:-0.23,0,0 --patch=0.10,0.04,-0.02,0.31,0.095,0.006:-0.23,0.055,0
   ```

5. 3DGRUT で USDZ に書き出し、`so101_pai.recon.usdz` で座標系を直して `assets/scene/workspace2b_v3.usdz` に置く（書き出しの細かい手順と色合わせは `docs/WORKLOG.md` の 3DGS の節）。
   - シミュレーターは、このファイルを自動で背景に使う（`scripts/lib/common.sh`、`BACKGROUND=none` で無効）。

確認：`sim/tools/smoke_env.py --view=...` で、正面カメラの画像が実機の C920 の画像と重なることを見る（`so101_pai/robot/camera_overlay.py` で実機側に重ねて合わせる）。

## 3. シミュレーターの配置（必ず付ける環境変数）

レポートの結果はすべて次の配置です。付け忘れると、昔の配置（角皿・赤い積み木など）で動きます。

```bash
export PAI_LAYOUT=real1031 PAI_PLATE=round PAI_BLOCK_COLOR=wood
# 積み木の大きさは PAI_BLOCK_SIZE（m）。データ集めでは 0.028 / 0.030 / 0.032、評価と動画では 0.03
```

- 皿は (−0.05, 0.17) に固定。積み木は中心 (0.12, 0.17) から左右 ±3 cm・前後 ±4 cm、向き ±45°（毎回ランダム）。
- `sim/tools/orbit_video.py` のようにシミュレーターを直接起動するツールでは、`PAI_BACKGROUND_USD=$PWD/assets/scene/workspace2b_v3.usdz` も付ける。

## 4. シミュレーターでお手本を集め、学習する（条件 C ＝ ACT v5）

自動のお手本（IK + DART ノイズ 2°）で、積み木の大きさごとに成功 75 本ずつ、計 225 本を集める（画面なし、1 セッション約 40 分）：

```bash
PAI_BLOCK_SIZE=0.028 SEED=101 scripts/teleop/expert.sh ws3_b28 75
PAI_BLOCK_SIZE=0.030 SEED=102 scripts/teleop/expert.sh ws3_b30 75
PAI_BLOCK_SIZE=0.032 SEED=103 scripts/teleop/expert.sh ws3_b32 75
```

（3 の環境変数を export した状態で実行する。SEED はセッションごとに 101〜103 を使った。）

LeRobot のデータセットにする（成功したエピソードだけ。225 本、失敗 112 本は除外）：

```bash
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m so101_pai.dataset.bag2lerobot data/bags/sim/ws3_b28 data/bags/sim/ws3_b30 data/bags/sim/ws3_b32 \
    --repo-id local/pai_sim_v5 --root data/datasets/pai_sim_v5
```

ACT を一から 4 万ステップ学習する（約 4 時間）：

```bash
OUT=outputs/train/act_pai_sim_v5 STEPS=40000 BATCH=16 SAVE_FREQ=10000 \
EXTRA="--dataset.image_transforms.enable=true --policy.use_amp=true" \
scripts/train/train.sh act local/pai_sim_v5 data/datasets/pai_sim_v5
```

シミュレーターで 20 回評価する（結果：13/20）：

```bash
PAI_BLOCK_SIZE=0.03 scripts/eval/sim.sh outputs/train/act_pai_sim_v5/checkpoints/last/pretrained_model 20 A_sim_act_v5
# 記録して動画にするなら RECORD=1 を付ける
```

## 5. 実機のお手本 10 本で追加学習する（条件 A・B）

### 5.1 実機の配置をそろえる

正面カメラ・皿・背景の扉の位置が、お手本を録ったときと違うと、学習したモデルはつかめません。毎回、お手本の画像を重ねて合わせます（`.venv` の OpenCV は画面表示なしなので、システムの Python で動かす）：

```bash
PYTHONPATH=$PWD /usr/bin/python3 -m so101_pai.robot.camera_overlay --device /dev/video0 \
    --reference assets/reference/front_real_demo_ep9.png   # お手本の最初の正面画像（Git には入れていない）
# キー：[ / ] 重ねる濃さ、e 輪郭だけ、q 終了
```

### 5.2 お手本を録る

リーダーアームで操作して、成功 10 本を録る（キー：s=開始 y=成功 n=失敗 q=終了。積み木は手で戻す）：

```bash
ENABLE_FOLLOWER=1 scripts/teleop/real.sh real_1101a
scripts/dataset/build.sh real pai_real_v1          # data/bags/real/* の成功エピソードだけ → data/datasets/pai_real_v1
```

### 5.3 学習（A と B）

設定はどちらも同じ（batch 16、学習率 1e-5、画像拡張・AMP あり）で、違いは初期値だけです。GPU メモリの都合で 1 本ずつ学習します（2 万ステップで約 2 時間）。

```bash
COMMON='--dataset.image_transforms.enable=true --policy.use_amp=true'
# A：実機 10 本だけで一から
OUT=outputs/train/act_real_v1_scratch STEPS=20000 BATCH=16 SAVE_FREQ=5000 EXTRA="$COMMON" \
  scripts/train/train.sh act local/pai_real_v1 data/datasets/pai_real_v1
# B：シミュレーターの ACT v5 から追加学習
INIT=outputs/train/act_pai_sim_v5/checkpoints/last/pretrained_model \
OUT=outputs/train/act_real_v1_from_sim STEPS=20000 BATCH=16 SAVE_FREQ=5000 EXTRA="$COMMON" \
  scripts/train/train.sh act local/pai_real_v1 data/datasets/pai_real_v1
```

1000 ステップのモデルは、同じ設定で 5000 ステップまで 1000 ステップごとに保存した別の学習から取りました：

```bash
OUT=outputs/train/act_real_v1_scratch_early STEPS=5000 BATCH=16 SAVE_FREQ=1000 EXTRA="$COMMON" \
  scripts/train/train.sh act local/pai_real_v1 data/datasets/pai_real_v1
INIT=outputs/train/act_pai_sim_v5/checkpoints/last/pretrained_model \
OUT=outputs/train/act_real_v1_from_sim_early STEPS=5000 BATCH=16 SAVE_FREQ=1000 EXTRA="$COMMON" \
  scripts/train/train.sh act local/pai_real_v1 data/datasets/pai_real_v1
```

## 6. 実機で評価する

実機はかならず人がそばにいるときに動かします。キー：r=開始姿勢へ戻す → 積み木を置く → s=開始 → y=成功 / n=失敗、q=終了。

共通の設定は `ACTION_STEPS=30`（予測した 100 ステップのうち 30 だけ実行して予測し直す）と `REST_POSE=start`（手首を下げてグリッパーを少し開いた開始姿勢）です。

| 条件 | チェックポイント | 名前の例 |
|---|---|---|
| A 1000 | `outputs/train/act_real_v1_scratch_early/checkpoints/001000/pretrained_model` | `A1k_video` |
| A 5000 / 1 万 / 2 万 | `outputs/train/act_real_v1_scratch/checkpoints/{005000,010000,020000}/pretrained_model` | `A5k_video` など |
| B 1000 | `outputs/train/act_real_v1_from_sim_early/checkpoints/001000/pretrained_model` | `B1k_video` |
| B 5000 / 1 万 / 2 万 | `outputs/train/act_real_v1_from_sim/checkpoints/{005000,010000,020000}/pretrained_model` | `B5k_video` など |
| C（ゼロショット） | `outputs/train/act_pai_sim_v5/checkpoints/last/pretrained_model` | `C_ver_01` |

例（B 5000 を 3 回）：

```bash
ENABLE_FOLLOWER=1 ACTION_STEPS=30 REST_POSE=start RECORD=1 scripts/eval/real.sh \
    outputs/train/act_real_v1_from_sim/checkpoints/005000/pretrained_model 3 B5k_video
```

- 結果は `outputs/eval/<名前>.json`、記録は `data/bags/eval/<名前>`。名前は毎回変える。
- レポートの表（各 3 回）は、積み木を皿の手前のほぼ同じ場所に置いて続けて動かした結果。
- 判定を押し間違えたら、録画で確かめて JSON を直す（B 2 万ステップの試行 2 はそうした）。

## 7. 動画とレポート

```bash
source /opt/ros/jazzy/setup.bash
# 評価の記録 → 試行ごとの MP4（正面カメラの右下に手先カメラ）
.venv/bin/python -m so101_pai.dataset.bag2video data/bags/eval/A1k_video data/bags/eval/B5k_video ... --out outputs/video
# シミュレーターの場面を回り込んで撮る（腕を動かすなら --pick）
PAI_LAYOUT=real1031 PAI_PLATE=round PAI_BLOCK_SIZE=0.03 PAI_BLOCK_COLOR=wood \
PAI_BACKGROUND_USD=$PWD/assets/scene/workspace2b_v3.usdz \
  scripts/sim/run.sh python sim/tools/orbit_video.py --headless --enable_cameras --out logs/orbit_pick --start-deg -40 --end-deg 35 --pick
.venv/bin/python -m so101_pai.dataset.frames2video logs/orbit_pick outputs/video/sim_scene_orbit_pick.mp4
# 条件の字幕を入れる → outputs/video/captioned/
scripts/video/caption_all.sh
# 提出用の動画を組み立てる（1〜3 分の範囲外なら止まる）→ outputs/video/final/pai_final.mp4
.venv/bin/python scripts/video/make_movie.py
```

レポートは `report/report.html`（Git には入れていない）を Chrome で PDF にしました：

```bash
google-chrome --headless=new --no-pdf-header-footer --print-to-pdf=report/report.pdf "file://$PWD/report/report.html"
```

## 8. つまずきやすい点

- シミュレーターの環境変数（3 節）を付け忘れると、別の配置で動いて結果が全く変わる。
- 実機のお手本は開始から 1〜4 秒止まっているため、モデルが初期姿勢で「待つ」ことを覚える。`REST_POSE=start` で避けられる（`TRIM_IDLE=2` で変換時に取り除くこともできるが、今回の結果には使っていない）。
- `ACTION_STEPS` が 50 や 100 だと、途中で位置を直せず横にずれてつかめない。30 にする。
- 実機のカメラ・皿・扉の位置がずれると、2 万ステップのモデルでも失敗する。毎回 5.1 で合わせる。
- 「Input voltage error」でフォロワーが起動しないときは、アームの電源を挿し直す。
- シミュレーターと実機を同時に動かさない。
