# PAI最終課題 計画書

- 締切：**2026-11-02（月）AM 10:00**
- 作成：2026-10-27
- 課題の要項：`最終課題（10_19 改訂版）.pdf`

## 1. テーマ

**自室の作業環境をシミュレーションで再現（Real2Sim）し、シミュレーションのデータだけで学習したピック&プレイスが、実機のSO-101でゼロショットで動くかを調べる。**

| 評価項目 | この課題で示すこと |
|---|---|
| 創造性 | 3DGRUTで自室を再現した背景、SimReadyの物体、ROS 2でシミュレーションと実機のインターフェースを統一 |
| 技術力 | rosbagからLeRobot形式への変換、ストリーミング学習、B300でのVLAのファインチューニング |
| 実用性 | シミュレーションだけのデータが実機でどこまで通用するかを定量的に評価する |
| 表現力 | 3条件の比較と、失敗の分類を動画とレポートで示す |

## 2. 決定事項

| 項目 | 決定 |
|---|---|
| 実機 | SO-101のリーダー＋フォロワー、カメラ2台（正面と手首） |
| タスク | 積み木を1個つかんで皿に置く。余裕があれば「赤を置いて」のような色の指定に発展させる |
| 掴む物 | Isaac Sim 5.1公式 `Isaac/Props/Blocks/{red,blue,green,yellow}_block.usd`（4.7cmの立方体、20g） |
| 置く場所 | NVIDIA SimReady `dishware_plastic_smallPlate_a01`（丸皿）を scale (1.8, 1.8, 0.4) で直径約17.8cm・高さ約1.8cmにし、白に塗り替える（10/27変更）。実物は直径約18cmの白い紙皿 |
| シミュレータ | Isaac Sim 5.1、Isaac Lab v2.3.0、LeIsaac（ベースとして使い、クレジットを明記する） |
| 背景 | 3DGRUT（3DGUT）で作ったUSDZ |
| ROS | ROS 2 Jazzy。シミュレーションも実機も同じトピックを使う。Isaac ROSは使わない |
| 実機のドライバ | **LeRobotの `SO101Follower` / `SO101Leader` を包む自作のrclpyノード** |
| データ収集 | **rosbag2のみ** → 自作の変換スクリプト → LeRobot v3.0 → HF Hub（private） |
| 学習 | ローカル（RTX 5060 Ti）：**SmolVLA**。B300：**GR00T N1.7**（3B、HF Hubからストリーミングで学習）。ACTは比較用のベースライン（10/27変更） |
| LeRobotのバージョン | 変換、学習、推論は `~/lerobot`（v0.6.1）に統一する。LeIsaacの環境にはLeRobotを入れない（必要になったら見直す） |
| 進め方 | Claudeが環境構築と実装を行う（10/27にユーザー自身が行う方針から変更）。手順は `setup/` のスクリプトとして残す。ライセンスへの同意、sudo、`~/.bashrc` の変更、実機を動かす操作はユーザーに確認する |

## 2.5 環境の分離方針（ローカルを汚さない）

**ルール**：システムのPythonやaptに直接パッケージを入れない。`sudo pip`、`pip install --user`、システムにインストールされたPythonへの直接の`pip install`は禁止。どの作業も専用の仮想環境の中で行う。

| 用途 | 環境 | Python | 作り方 | 入れるもの |
|---|---|---|---|---|
| シミュレーション（Isaac Sim、Isaac Lab、LeIsaac） | conda `leisaac` | 3.11 | `conda create -n leisaac python=3.11` | CUDA toolkit 12.8（conda経由）、torch 2.7.0+cu128、`isaacsim[all,extscache]==5.1.0`、Isaac Lab、LeIsaac |
| 3D再構成（3DGRUT） | conda `3dgrut`（中身は3DGRUTの `install_env_uv.sh` がuvで入れる） | 3.11 | `setup/06_3dgrut_env.sh`（公式の「conda-managed CUDA」手順） | CUDA toolkit 12.8.1（conda経由）、torch 2.8.0+cu128、3DGRUT、tiny-cuda-nn、Kaolin、slangc。システムにnvccが無く、`.venv` 方式はCUDA runfileのEULAを `--silent` で受諾してしまうためcondaにした |
| COLMAP | conda `colmap` | — | `setup/05_colmap_env.sh` | COLMAP 4.2.1（conda-forgeのCUDA 12.9版）、ffmpeg（aptでは入れない） |
| ROS 2ノード（leader、follower、推論、変換） | このリポジトリ直下の `.venv`（uv） | **3.12（システムと同じ）** | `uv venv --python /usr/bin/python3.12 --system-site-packages` | `~/lerobot`（v0.6.1、feetech付き）、mcap関連。rclpyは `/opt/ros/jazzy` のものを読むだけで使う |
| B300での学習 | B300上のリポジトリ直下 `.venv-train`（uv） | 3.12 | `setup/07_cloud_train_env.sh` | LeRobot（ローカルと同じコミット e40b58a8）、CUDA 12.9以上のtorch（既定 cu130） |

補足：
- ROSのノード用の環境は、Jazzyのrclpyを使うために `--system-site-packages` を付ける。システムのパッケージは**読むだけ**で、追加のインストールはすべて `.venv` に入る
- 既存の conda 環境（`lerobot`、`tv`、`unitree_sim_env`）は変更しない
- `cmake` と `build-essential` はすでにシステムに入っているので、Isaac Labのビルドのために `sudo apt install` する必要はない
- **`~/.bashrc` で `/opt/ros/jazzy/setup.bash` が常に読み込まれている**。この状態だと、Python 3.12向けのROSのパッケージが `PYTHONPATH` 経由で `leisaac` 環境（Python 3.11）に混ざり、Isaac Simが誤動作するおそれがある。Isaac Simを使うターミナルでは、ROSを読み込まない状態で起動する（`~/.bashrc` を変えるかどうかは別途ユーザーが判断する）
- Isaac SimやHFのキャッシュ（`~/.cache` 配下）は、仮想環境とは別にホームに作られる。容量が気になる場合は、まとめて削除できるように場所を記録しておく

## 3. システム構成

```
                    ┌──────────── 共通のROS 2トピック ────────────┐
[リーダーアーム] → leader_node ─ /leader/joint_states ──────┬──────────────┐
                                                           ↓              ↓
              シミュレーション: Isaac Sim + LeIsaac         実機: follower_node（LeRobot）
                3DGS背景 + 積み木 + 皿                       + カメラのノード×2
                ROS 2ブリッジで出力                                │
                      │                                          │
                      └──── /joint_states, /cam_*/image_raw, /episode/status ────┘
                                         ↓
                                 rosbag2（mcap）
                                         ↓
                    bag2lerobot（自作。LeRobotDatasetのAPIを使う）
                                         ↓
                    LeRobot v3.0 → HF Hub（private）
                          ↓                         ↓
                SmolVLA（ローカル）        GR00T N1.7（B300、ストリーミング）
                          └──────────┬──────────────┘
                                     ↓
                        policy_node（シミュレーションと実機で共通）
                         → /leader/joint_states の代わりに指令を出す
```

ポイント：
- **リーダーアームのノードはシミュレーションと実機で共通**にする。行動（action）のトピックがまったく同じになり、sim2realの差を「観測の差」だけに絞り込める
- 推論ノードはリーダーアームのノードと同じトピックに指令を出す。テレオペと推論を差し替えるだけで済む

### 共通トピックの仕様（初日に確定させる）

| トピック | 型 | LeRobotでの特徴量 | 備考 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `observation.state` | フォロワーの6軸（5関節＋グリッパー） |
| `/leader/joint_states` | `sensor_msgs/JointState` | `action` | リーダーアーム、または推論ノードの指令 |
| `/cam_front/image_raw` | `sensor_msgs/Image` | `observation.images.front` | 解像度とfpsを実機に合わせる |
| `/cam_wrist/image_raw` | `sensor_msgs/Image` | `observation.images.wrist` | 同上 |
| `/episode/status` | 自作（または `std_msgs/String` のJSON） | エピソードの区切り、成功フラグ、`task` の文字列 | LeIsaacのレコーダーを使わないため必要 |

決めること：関節の単位（ラジアンに統一）、グリッパーの値の範囲、関節名、記録のfps（30Hzを想定）、時刻同期の方法（最も近い時刻のサンプルを使う）。

## 4. 自作するもの（公開するGitHubリポジトリの中身）

| モジュール | 内容 | 優先度 |
|---|---|---|
| `so101_ros/leader_node` | LeRobotの`SO101Leader`を読み、`/leader/joint_states`を出す | 高 |
| `so101_ros/follower_node` | LeRobotの`SO101Follower`を動かし、`/joint_states`を出す | 高 |
| `sim/` | LeIsaacのカスタムタスク（積み木→皿）、ROS 2への出力、`/episode/status`、ドメインランダム化 | 高 |
| `bag2lerobot/` | rosbag2をLeRobot v3.0に変換する（`LeRobotDataset.create` → `add_frame` → `save_episode`） | **最優先** |
| `policy_node/` | SmolVLA・GR00T N1.7・ACTで推論し、指令を出す | 中 |
| `eval/` | 成功判定と記録用のスクリプト | 中 |
| `scripts/fetch_assets.sh` | アセットをダウンロードするスクリプト。**アセット本体はリポジトリに含めない** | 低 |

## 5. スケジュール

> **10/30 の変更（ユーザーの判断）**：シミュレーションのデータは、リーダーアームでの人のテレオペではなく、**自動のお手本**（`so101_pai/teleop/expert.py`、`scripts/teleop/expert.sh`）で集める。シミュレーター内の積み木の位置を使って IK でつかんで運ぶ動きを作り、関節の指令を `/leader/joint_states` に流すので、rosbag → LeRobot の経路と話題の仕様はテレオペと同じ。エピソードごとに速さ・つかむ位置と向き・高さ・寄り道・置く位置を乱数で変える。リーダーでのテレオペ（`scripts/teleop/sim.sh`）も使えるので、集めたデータに混ぜてよい。

| 日 | やること | 完了の基準 |
|---|---|---|
| **10/27（火）** | LeIsaacとIsaac Sim 5.1の環境を作る。PickOrangeを動かす。共通トピックの仕様を確定させる。leader_nodeを作る。Isaac SimのROS 2ブリッジからrosbagに記録する。**買い出し** | リーダーアームの動きがシミュレーションに反映され、rosbagに記録できる |
| **10/28（水）** | **bag2lerobotを完成させる**。作業環境を撮影 → COLMAP → 3DGRUT → USDZ → Isaac Simで位置合わせ | 短いbagを変換し、`lerobot`で読み込める。**夜：3DGSを続けるか切り替えるかを判断** |
| **10/29（木）** | カスタムタスクを作る（ドメインランダム化を含む）。テレオペで**50エピソード以上**集める。follower_nodeを作り、カメラのノードを立ち上げる | データセットをHF Hubにpushできている |
| **10/30（金）** | ローカルでSmolVLAを学習（夜間）。**B300の環境を作り、GR00T N1.7をストリーミングで学習**。実機のデータを10エピソード集める。**記録したエピソードを実機で再生して、関節の対応を確認する** | 学習済みのモデルが2つある。実機で再生が正しく動く |
| **10/31（土）** | 3条件×20回で評価する。動画の素材を撮影する | 成功率の表と、失敗の分類 |
| **11/1（日）** | 動画、レポート、リポジトリの整理。YouTubeとomnicampusに提出する | 提出が完了している |
| 11/2（月）朝 | 予備 | — |

### 判断のタイミング

| 時点 | 条件 | 対応 |
|---|---|---|
| 10/27の夜 | Isaac Sim 5.1がRTX 5060 Tiでまともに動かない | カメラの解像度を下げ、`num_envs=1`にする。それでも無理なら Isaac Sim 5.0 と Isaac Lab v2.2.1 に変更 |
| 10/28の夜 | 3DGSの背景がカメラ画像に映らない | LeIsaacの DigitalTwin Env（実背景の合成）に切り替え、それも無理なら既存のシーンで進める |
| 10/28の夜 | bag2lerobotが完成していない | 10/29の午前までに終わらなければ、LeIsaacのLeRobotレコーダーで一時的に記録し、変換スクリプトは後から仕上げる |
| 10/30の昼 | B300の環境構築でつまずく | ローカルのSmolVLAを本番にし、B300（GR00T）の結果は「追加の実験」として扱う |

## 6. 評価の設計

- **成功の定義**：積み木が皿の上にあり、グリッパーが開いた状態でエピソードが終わること（30秒以内）
- **初期配置**：積み木の初期位置を事前に5か所決め、各4回で計20回試す。シミュレーションと実機で同じ位置を使う
- **条件**：

| 条件 | 学習データ | 評価する環境 |
|---|---|---|
| A | シミュレーションのみ | シミュレーション（上限の目安） |
| B | シミュレーションのみ | **実機（ゼロショット、本題）** |
| C | シミュレーション＋実機10エピソード | 実機 |

- それぞれSmolVLAとGR00T N1.7で評価する（時間が足りなければ、GR00TはBだけ）
- **失敗の分類**：届かない／掴み損ねる／掴めたが落とす／置く位置がずれる
- 追加の比較（技術面）：通常の学習とストリーミング学習で、学習速度（steps/秒）と成功率を比べる

## 7. 提出物

| 提出物 | 内容 |
|---|---|
| 動画（1〜3分） | 構成：課題の説明 → Real2Simで再現した環境とシミュレーションでのテレオペ → 学習 → シミュレーションでの成功 → 実機ゼロショット → 結果の表。倍速にした場合は明記する |
| レポート（PDF） | 目的、システム構成、**LeIsaacや公式サンプルとの差分（自分で作った部分）**、評価結果、失敗の分析、考察 |
| GitHub（public） | 自作のコード一式とREADME。アセット本体、3DGSのデータ、bag、データセットは含めない |

**個人情報のチェック**：動画、3DGSの背景、カメラ画像、README、gitのコミット情報に、氏名・顔・部屋の個人的な物が映っていないかを提出前に確認する。

## 8. リスク

| リスク | 対策 |
|---|---|
| RTX 5060 Tiの性能がIsaac Simの推奨を下回る | 上の「判断のタイミング」を参照 |
| Isaac Sim（Python 3.11）とJazzy（Python 3.12）でPythonのバージョンが違う | シミュレーション側はIsaac Sim同梱のROS 2ライブラリを使う。自作ノードはシステムのPython 3.12にLeRobotを入れた仮想環境で動かす |
| B300（Blackwell Ultra）にPyTorchとCUDAが対応しているか | 10/30の朝に、最初にGPUが認識されるかを確認する |
| シミュレーションと実機で関節の単位やキャリブレーションがずれる | 実機の再生テスト（10/30）で確認してから評価に進む |
| 紙皿が軽くてずれる | 実機では両面テープで固定し、シミュレーションでは動かない物体として設定する |

## 9. 参照する公式ページ

- LeIsaac：https://lightwheelai.github.io/leisaac/
- Isaac Lab のインストール：https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html
- Isaac Sim 5.1：https://docs.isaacsim.omniverse.nvidia.com/5.1.0/
- 3DGRUT：https://github.com/nv-tlabs/3DGRUT
- LeRobot：https://huggingface.co/docs/lerobot （SO-101、データセット、学習、SmolVLA）
- rosbag2：https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html
- 参考（Humble向け）：https://github.com/nimicurtis/so101_ros2
