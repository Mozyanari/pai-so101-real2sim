# 作業ログ

作業した内容・分かったこと・決めたことを時系列で残す。計画そのものは `PLAN.md`、作業の進め方は `CLAUDE.md` を参照。

## 2026-10-27

### 環境構築
- conda `leisaac`（Python 3.11）を作成した。中身は torch 2.7.0+cu128、`isaacsim[all,extscache]==5.1.0`、Isaac Lab、LeIsaac v0.4.0。手順は `setup/01_leisaac_env.sh`。
  - Isaac Sim が起動直後に落ちる問題があった。原因は numpy が 2.x に上がっていたことで、numpy 1.26.0 などの版を固定して解消した。
  - flatdict がビルドできない問題があった。setuptools 81 で `pkg_resources` が消えたためで、`setuptools<81` に固定し、`--no-build-isolation` で入れて解消した。
  - Isaac Sim の EULA には、ユーザーの了承を得たうえで `OMNI_KIT_ACCEPT_EULA=YES` で同意した。
- ROS 2・LeRobot 用に、リポジトリ内の uv `.venv` を作成した（`setup/02_ros_venv.sh`）。
  - `/usr/bin/python3.12` に `--system-site-packages` を付けて作り、LeRobot v0.6.1 を editable で入れた。torch は 2.11.0+cu128。
- アセットの取得スクリプトを作った。`setup/03_leisaac_assets.sh` は LeIsaac 用、`setup/04_task_assets.sh` は積み木と皿用。取得したファイルは git 管理外。
- `scripts/leisaac_run.sh` は ROS の環境変数を消してから conda `leisaac` で実行する。`LEISAAC_ROS=1` を付けると、Isaac Sim 同梱の Jazzy rclpy を使う。

### ROS 2 の取り決めと変換
- トピックの取り決めは `so101_pai/contract.py` にまとめた。
  - 関節角はシミュレーターの USD の関節空間（ラジアン）で流す。データセットには LeRobot のモーター単位で保存する。
- `so101_pai/bag2lerobot.py`：rosbag2（mcap）を受信時刻で 30 fps に並べ直し、LeRobot v3.0 の API で書き出す。
- テストは `tests/` に 9 件あり、すべて合格。

### シミュレーション
- タスク `PAI-SO101-PlaceBlock-v0` を作った（`sim/pai_sim/tasks/place_block/`）。LeIsaac の片腕テンプレートを土台にしている。
  - 机は 80×60 cm の当たり判定付きの板で、天面が z=0。
  - 積み木は Isaac Sim 公式の `red_block.usd`（4.7 cm、20 g）。位置は ±5/±4 cm、向きは ±45° でランダムにする。
  - 皿は最初 SimReady `dishware_plastic_bigPlate_a01`（青・角）にしていた。実物の紙皿に合わせて `dishware_plastic_smallPlate_a01`（丸）に変え、scale (1.8, 1.8, 0.4) で直径約 17.8 cm・高さ約 1.8 cm にし、白に塗り替えた。動かない固定物として置いている。
  - アームは実機と同じ折りたたみ姿勢から始まる。`use_default_offset=False` にして、関節角の指令をそのままの値として扱う。
  - 成功条件（`mdp.block_on_plate_released`）：積み木が皿の上にあり、グリッパーが 8 cm 以上離れていること。
- シミュレーターと ROS 2 をつなぐ `sim/ros_bridge.py` を作った。
  - 受け取るもの：`/leader/joint_states`、`/episode/command`。
  - 送るもの：`/joint_states`、カメラ 2 台の画像、`/episode/status`。
- 速度の調整：物理計算は 60 Hz、制御と描画は 30 Hz（`decimation=2`、`render_interval=2`）にし、`--rendering_mode performance` を使う。録画中の速度が約 22 Hz から 29.9 Hz に上がった。
  - CPU の省電力設定は関係なかった。`intel_pstate` の `powersave` は動的にクロックを変える設定で、実際に 5.2 GHz まで出ていた。
  - 描画を軽くすると、画像が暗く、ざらつきが出る。実機へ持ち込むときの課題として残っている。
- Isaac Sim は SIGTERM を無視する。止めるときは、停止信号を送って 5 秒待ち、止まらなければ強制終了（SIGKILL）する。
  - 計測用のプロセスが 6 時間以上残っていて、これを強制終了したことで判明した。
- 失敗コマンドでリセットすると、約 6.5 秒止まる（エピソードの間のみ）。

### ハードウェアなしの通し試験（`scripts/e2e_sim_pipeline.sh`）
- 偽リーダー → シミュレーター → rosbag（mcap と zstd_fast で 25 秒、約 333 MB）→ LeRobot 形式の順につないだ。
- エピソード中の関節データは 29.9 Hz で記録され、データ間の最大の空きは 36 ms だった。白い皿に変えた後も通ることを確認した（2 エピソード、209 フレーム）。
- rosbag をバックグラウンドで動かすと Ctrl-C（SIGINT）が届かず、17 GB まで膨らんだ。そのため、`timeout -s INT` で記録時間を区切るようにした。

### リーダーアーム（保留）
- `/dev/ttyACM0` として認識された。
- 既存の `my_awesome_leader_arm` のキャリブレーションが、モーター内に保存された値と 6 関節すべてで食い違っていた。今の姿勢の値も範囲外だった。
- 実機キャリブレーションは後で行う。そのときのコマンド：`.venv/bin/lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.id=pai_leader`

### 報告ページ
- 進捗ページ（Artifact）：https://claude.ai/artifact/RC9TjGstahyjU9SpsdkBjE

### 皿の変更（白い丸皿）
- SimReady には白くて丸い皿がない。そこで `dishware_plastic_smallPlate_a01`（丸、直径 9.9 cm、高さ 4.5 cm、パッケージの記載は CC-BY-4.0）を scale (1.8, 1.8, 0.4) で直径約 17.8 cm・高さ約 1.8 cm にし、`visual_material` で白に上書きした。
  - Isaac Lab の上書きは `strongerThanDescendants` で付くので、子メッシュの材質より優先される。
- `setup/04_task_assets.sh` は、大皿と小皿の両方を取得するようにした。

### 3DGS 背景の環境（サブエージェントに任せた）
- conda `colmap` を作った（COLMAP 4.2.1 CUDA 版、ffmpeg）。手順は `setup/05_colmap_env.sh`。
- conda `3dgrut` を作った（CUDA 12.8.1、torch 2.8.0+cu128、sm_120 向けにビルド）。手順は `setup/06_3dgrut_env.sh`。
  - システムに nvcc がない。また `.venv` 方式は CUDA の EULA を `--silent` で受諾してしまう。そのため、公式手順のうち conda で CUDA を管理する方式にした。
- `scripts/recon/run.sh`：動画またはフォルダの画像 → COLMAP → 3DGUT（MCMC）→ NuRec USDZ（`assets/scene/<name>.usdz`）。
  - South Building の画像 40 枚で通しで動いた。COLMAP が 17 秒、学習 500 回と書き出しが 53 秒、GPU メモリの最大は 2.8 GB。
  - 出力は COLMAP の正規化座標で、実寸ではない。Isaac Sim で机に合わせる位置合わせが必要。
  - Isaac Sim 5.1 で読み込めるかは未確認。

### リファクタと構成の整理（役割ごとに分割）
- `so101_pai/` を次の 5 つに分けた。
  - `common/`：トピックの取り決め、エピソード、ROS メッセージの変換、ノード共通の `run()`、シミュレーター側のエピソード制御 `episode_control.py`
  - `robot/`：実機のノードとキャリブレーション
  - `teleop/`：キー操作のツール
  - `dataset/`：変換と HF Hub へのアップロード
  - `policy/`：推論の `runner.py`、`policy_node`、評価の `evaluation.py` と `eval_node`
- 判断のロジックは ROS や Isaac に依存しないクラスに分け、単体テストを付けた。ROS のノードは薄い殻にした。
  - ノードは `**node_kwargs` を受け取るので、テストから `parameter_overrides` を渡して作れる。
- `scripts/` を `sim/ teleop/ dataset/ train/ eval/ recon/ lib/` に分けた。プロセスの起動と停止は `scripts/lib/common.sh` にまとめた。
- `sim/ros_bridge.py`：エピソードの判断を `EpisodeControl` に任せた。計測の処理は `RateProfile` に分けた。
  - 操作者の判断で成功にできる `success` コマンドを追加した。
  - 開始時にエピソードのタイマーを 0 に戻すようにした。
- リーダーとフォロワーのキャリブレーション名を `pai_leader` / `pai_follower` に変えた。
  - `connect_calibrated()`：ファイルがなければキャリブレーションのコマンドを示して止まる。ファイルがあれば、モーターに書き込んでから接続する（対話入力は使わない）。
- ruff（lint と format）を `.venv` に追加し、`pyproject.toml` に設定した。
- テストは 32 件すべて合格し、カバレッジは 67%。実機のノードはハードウェアが必要なので、カバレッジに入っていない。

### 学習・推論・評価
- `scripts/train/train.sh`：ACT、ストリーミング、SmolVLA を切り替えられる。
  - ACT を 30 ステップ学習させて、チェックポイントを `PolicyRunner` で読み込み、推論できることを確認した。
  - SmolVLA は、`lerobot/smolvla_base` のカメラ名が `camera1..3` のため `--rename_map` で front→camera1、wrist→camera2 に対応付けた。5 ステップの学習と推論を確認した（2 回目以降は約 2 ms）。
- `setup/07_cloud_train_env.sh`：B300 用。LeRobot をローカルと同じコミット e40b58a8 に固定し、torch は cu130。B300 では未実行。
- `scripts/eval/sim.sh`：policy_node と eval_node でシミュレーターを自動で評価する。試しの ACT で 3 回評価し、毎回 30 秒のタイムアウトで終わり、結果を JSON に書き出した。
- `scripts/eval/real.sh`：実機で人が成否を判定する。`ENABLE_FOLLOWER=1` のときだけアームが動く。

### 原因が分かった問題
- **リセットで 5〜6 秒止まる問題**の原因は、LeIsaac のテンプレートの `recorders`（Isaac Lab の HDF5 録画機能）だった。リセットのたびに、カメラ画像を含むエピソードを `/tmp/isaaclab/logs/dataset.hdf5` に書き出していた。
  - cProfile では、`h5py create_dataset` に 5.6 秒かかっていた。
  - タスクの設定で `recorders = RecorderManagerBaseCfg()` にして無効化した。リセット中も 28 Hz 以上を保てるようになった。
  - 溜まっていた 243 MB の hdf5 は削除した。
- **rosbag の停止**：バックグラウンドの `ros2 bag record` は SIGINT を無視する。SIGTERM なら正しく止まり、metadata.yaml も書き出される（実験で確認）。
- **SIGTERM 時の RCLError**：spin の途中でコンテキストが無効になる。`rclpy.ok()` が False のときは、正常な終了として扱うようにした。

### 3DGS 背景を Isaac Sim 5.1 に読み込めるか確認
- タスクに `PAI_BACKGROUND_USD=<usdz>` を渡すと、背景を読み込むようにした（`add_background()`）。
  - 置く位置は、同じ名前の `<name>.align.json`（pos・rot・scale）で指定する。
- サンプルの `south40.usdz`（500 回だけ学習、位置合わせなし）で描画を確認した。
  - NuRec のガウシアンは、TiledCamera の画像に描かれた。通常の描画でも、軽量描画（`--rendering_mode performance`）でも表示された。
  - 120 ステップの速度は 45.9 / 53.3 steps/s で、背景による速度低下は見られなかった。
- 10/28 の判断ポイント「3DGS の背景がカメラ画像に映るか」は、技術的には通った。残りは実際の撮影の品質と、机への位置合わせ。

### 学習モデルの変更（ユーザーの判断）
- 手元（RTX 5060 Ti）で学習するのは **SmolVLA**、B300 では **GR00T N1.7**（3B）に変えた。ACT は比較用のベースラインとして残す。
- `scripts/train/train.sh` は `smolvla | groot | act` を切り替えられる。
  - SmolVLA はバッチ 16 の場合、40 ステップで GPU メモリの最大使用量が 5.5 GB、速度は 2.3 ステップ/秒。20,000 ステップで約 2.4 時間の見込み。
  - GR00T は LeRobot の SO-101 向けの手順（`docs/source/groot.mdx`）に合わせた：`--policy.type=groot --policy.base_model_path=nvidia/GR00T-N1.7-3B --policy.embodiment_tag=new_embodiment`、チャンク 16、bf16。
    - `use_relative_actions=false` にした。LeRobot の同期推論は、相対行動に未対応のため。
- `.venv` と `setup/07` に LeRobot の `groot` の追加パッケージを入れた（transformers 5.5.4 のまま、SmolVLA と共存できる）。
- **ブロッカー**：GR00T N1.7 の土台の VLM `nvidia/Cosmos-Reason2-2B` は、利用規約への同意が必要な gated リポジトリだった。HF での同意と `hf auth login` をしないとダウンロードできない。
  - 手元で 2 ステップ学習させる確認は、401 エラーで止まった。
  - 同意はユーザーの判断なので、未実施。
