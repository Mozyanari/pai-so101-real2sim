# 設計

このリポジトリの仕組みの決まりごとと、最初の計画（2026-10-27）から何を変えたかをまとめます。使い方は `README.md`、結果を作り直す手順は `docs/REPRODUCE.md`、日ごとの経過は `docs/WORKLOG.md` にあります。

## 1. 目的とタスク

- **目的**：実機の SO-101 の作業机をシミュレーターに再現し（Real-to-Sim）、シミュレーターのデータで学習した方策が実機で動くかを調べる。動かない場合は、少ない実機データでの追加学習に、シミュレーターでの事前学習が役立つかを調べる。
- **タスク**：木の積み木（約 3 cm 角）をつかみ、白い紙皿（直径約 18 cm）の上に置く。
- **成功**：積み木が皿の上にあり、グリッパーが開いた状態（シミュレーターは自動で判定、45 秒で打ち切り。実機は人が目で判定）。

## 2. 全体の構成

```
[リーダーアーム] leader_node ─┐
[自動のお手本] ros_bridge --expert ─┼→ /leader/joint_states（行動）
[学習した方策] policy_node ─┘                │
                                            ↓
        シミュレーター：Isaac Sim + LeIsaac + 3DGS 背景（sim/ros_bridge.py）
        実機：follower_node + camera_node × 2
                                            │
              /joint_states, /cam_front/image_raw, /cam_wrist/image_raw, /episode/status
                                            ↓
                 rosbag2（mcap）→ bag2lerobot → LeRobot v3.0 データセット → lerobot-train（ACT / SmolVLA）
```

- 行動のトピックは、リーダー・自動のお手本・方策のどれが出しても同じ。シミュレーターと実機の違いを、観測（画像と関節）の違いだけにする。
- データは rosbag だけで集める（LeIsaac の LeRobot レコーダーは使わない）。変換は LeRobot の API（`LeRobotDataset.create` / `add_frame` / `save_episode`）で書く。

### トピックの決まり（`so101_pai/common/contract.py`）

| トピック | 型 | LeRobot の特徴量 | 中身 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `observation.state` | フォロワーの 6 軸（5 関節＋グリッパー）、ラジアン |
| `/leader/joint_states` | `sensor_msgs/JointState` | `action` | リーダー・自動のお手本・方策の指令 |
| `/cam_front/image_raw` | `sensor_msgs/Image` | `observation.images.front` | 640×480、30 fps |
| `/cam_wrist/image_raw` | `sensor_msgs/Image` | `observation.images.wrist` | 640×480、30 fps |
| `/episode/status` | `std_msgs/String`（JSON） | エピソードの区切り | 開始・終了・成功、タスクの文 |
| `/episode/command` | `std_msgs/String`（JSON） | — | 操作ツールから開始・成功・失敗・リセットを送る |

- 関節はトピックではラジアン、データセットでは LeRobot のモーター単位（`radians_to_arm_motor` で変換）。手首の回転は実機の向きの基準に合わせて補正している。
- 変換は 30 fps の格子に並べ直し、各時刻で直前のメッセージを使う（`so101_pai/dataset/timeline.py`）。

## 3. 環境の分け方

システムの Python と apt には何も入れない。ツールごとに環境を分ける。

| 用途 | 環境 | Python | 作り方 |
|---|---|---|---|
| シミュレーター（Isaac Sim 5.1、Isaac Lab、LeIsaac） | conda `leisaac` | 3.11 | `setup/01_leisaac_env.sh` |
| ROS 2 のノード、変換、学習、推論（LeRobot v0.6.1） | リポジトリ直下の `.venv`（uv、`--system-site-packages` で `/opt/ros/jazzy` の rclpy を読む） | 3.12 | `setup/02_ros_venv.sh` |
| 3DGS（3DGRUT） | conda `3dgrut`（CUDA は conda から） | 3.11 | `setup/06_3dgrut_env.sh` |
| COLMAP | conda `colmap` | — | `setup/05_colmap_env.sh` |

- `~/.bashrc` で ROS 2 を読み込んでいると、Python 3.12 のパッケージが Isaac Sim（3.11）に混ざる。シミュレーター用のスクリプト（`scripts/sim/run.sh` など）は ROS を外して起動する。
- 変換・学習・推論は、すべて `~/lerobot`（v0.6.1）で行う。

## 4. 評価の設計（実際に行ったもの）

| 条件 | 学習データ | 評価 |
|---|---|---|
| C | シミュレーター 225 本だけ（ACT、4 万ステップ） | シミュレーター 20 回、実機（ゼロショット） |
| A | 実機のお手本 10 本だけ（一から） | 実機、1000・5000・1 万・2 万ステップで各 3 回 |
| B | C の ACT から実機の 10 本で追加学習 | A と同じ |

- 実機の評価は、同じ配置（正面カメラ・皿・扉をお手本のときにそろえる）、`ACTION_STEPS=30`、`REST_POSE=start` で行った。
- 実機の評価は回数が少ない（各 3 回、積み木の置き場所もほぼ 1 か所）ので、成功の数は目安として扱う。

## 5. 最初の計画から変えたこと

| 項目 | 最初の計画 | 実際 | 理由 |
|---|---|---|---|
| シミュレーターのデータ | リーダーアームでのテレオペ | IK の自動のお手本＋DART ノイズ（`so101_pai/teleop/expert.py`） | 人手で 200 本以上集める時間がない。テレオペも同じ経路で使える |
| 方策 | SmolVLA（ローカル）＋ GR00T N1.7（クラウド GPU） | ACT が主。SmolVLA は比較（シミュレーターで 4/20） | 同じデータで ACT の方がシミュレーターで高かった（13/20）。GR00T は使わなかった（関連のコードは削除） |
| 評価 | 3 条件 × 20 回、成功率で比べる | シミュレーター 20 回。実機は A・B のチェックポイントごとに各 3 回と、動作の観察 | 実機は時間がかかり、ゼロショットは成功しなかったため、追加学習の比較に重点を移した |
| 積み木と皿 | Isaac の 4.7 cm の積み木、角形の皿を白く | 積み木を 2.8〜3.2 cm に縮めて木目に、丸皿 | 実物（約 3 cm の木の積み木、丸い紙皿）に合わせた |
| 背景 | 3DGS をそのまま | 机の実測に合わせて配置、本物のアームを消す、C920 に色を合わせる | 3DGS の向きと大きさが任意で、写っている物が学習の邪魔になるため |

## 6. 使っている外部のもの

- LeIsaac（Apache-2.0）：SO-101 の USD と片腕タスクのテンプレート。
- Isaac Sim 5.1 / Isaac Lab：シミュレーター。
- LeRobot（Apache-2.0）：データセット形式、ACT・SmolVLA の学習と推論、SO-101 のドライバ。
- 3DGRUT、COLMAP：背景の再構成。
- アセット：Isaac Sim 公式の `Props/Blocks`、NVIDIA SimReady `dishware_plastic_smallPlate_a01`（リポジトリには含めず、`setup/04_task_assets.sh` で取得）。
