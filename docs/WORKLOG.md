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

## 2026-10-30

### 実機アームのキャリブレーション
- ユーザーが `pai_leader`（/dev/ttyACM0）と `pai_follower`（/dev/ttyACM1）をキャリブレーションした。
- leader_node は 60 Hz で `/leader/joint_states` を出せた。
- トルクを入れずに両アームの現在値を読み、キャリブレーションがモーターと一致することを確認した（`bus.is_calibrated` が True）。たたんだ姿勢での値はリーダーとフォロワーでほぼ同じ（差が大きいのは shoulder_lift の約 20 と wrist_roll の約 18、モーター単位）。
- 注意：LeRobot の `SO101Follower.connect()` は `configure()` の最後でトルクを入れる。follower_node を `enable:=false` で起動しても、アームは動かないが姿勢を保持して固くなる。

### 作業机の 3DGS 背景（本番の撮影）
- ユーザーが Pixel で撮影した動画（1080p・60 fps・SDR・97 秒）を `capture/room.mp4` に置いた。部屋の映像なので `.gitignore` に `capture/` を追加した。
- COLMAP：2 fps で取り出した 194 枚のうち 157 枚を登録、平均の再投影誤差 0.73 px。
- 3DGRUT（3DGUT + MCMC、30,000 回、ガウシアン 100 万個）：約 48 分。テスト画像で PSNR 26.1 dB、SSIM 0.93。`assets/scene/workspace.usdz`（113 MB）を書き出した。
- NuRec の USDZ の中身：Volume の `xformOp:transform` には、3DGRUT の正規化の逆変換が「列ベクトル形式のまま」書かれている（3DGRUT 自身の読み込み側も同じ解釈）。そのためステージ上の座標は COLMAP の座標と同じになる。位置合わせは COLMAP の座標を基準に計算すればよい。
- 位置合わせのツール `so101_pai/recon/align.py` を作った（テスト 8 件）。
  - COLMAP の疎な点群は木目の少ない天板をほとんど拾えなかったので、学習済みのガウシアン（不透明で小さいもの約 14 万個）を使う。
  - 木の色の点で RANSAC して天板の面を出し、カメラがある側を上とした。同じ面にある別の木（棚など）は、格子上でいちばん大きくつながった領域だけ残して除いた。
  - 天板は角の丸い長方形（長さ:幅 ≈ 2.2:1、高さ ≈ 長さの 0.39 倍）。両端にアームが 1 本ずつあり、明るいほうを白いフォロワーとした。
  - 縮尺は、ユーザーが測る天板の長さから決める。測定待ち。
- Isaac Sim で背景を読み込み、仮の縮尺で手首カメラに天板と黒いリーダーアームが映ることを確認した。背景があるときはシミュレーターの机の箱を非表示にした（当たり判定は残す）。

### git にしたことで LeIsaac がアセットを見つけられなくなっていた
- LeIsaac はカレントディレクトリの git のルート直下の `assets/` を探す。このリポジトリを git にしたため `advance_lecture/assets/robots/so101_follower.usd` を探して失敗していた（シミュレーター、テレオペ、評価のすべてに影響）。
- `scripts/sim/run.sh` で `LEISAAC_ASSETS_ROOT=third_party/leisaac/assets` を指定して直した。

### 背景の位置合わせを確定（実測：天板 90 × 40 cm、高さ 35 cm、フォロワーは机を横切る向き）
- 長さ 90 cm から縮尺を決めると、点群から測った幅は 40.2 cm、高さは 35.2 cm で、実測との差は 1% 以内。フォロワーの根元は短い辺から 19.7 cm、長い辺から 8.9 cm（自動検出）。
- `workspace.align.json` に机の天板のシミュレーター座標（中心と大きさ）も書き出し、背景があるときはシミュレーターの机の当たり判定をそれに合わせるようにした。
- **背景が机から約 30 cm ずれていた原因**：3DGRUT の NuRec 書き出しは、ガウシアンの座標は COLMAP のまま、Volume に正規化の逆変換（列ベクトル形式）を付ける。Isaac Sim はそれを適用するので、ステージ上の座標は COLMAP の座標にならない。先の「ステージ座標＝COLMAP 座標」という読みは誤りだった。
  - `so101_pai/recon/usdz.py` で Volume の変換を単位行列に戻して USDZ を詰め直し、`scripts/recon/run.sh` の export の最後に組み込んだ。真上からの画像で、本物の机・フォロワー・リーダーの位置がシミュレーターと一致した。
- `sim/tools/smoke_env.py` に `--view=x,y,z:tx,ty,tz`（複数指定可）を追加した。正面カメラをワールド座標の任意の位置に動かして撮る。値が負の数で始まるときは `--view=` の形で書く（argparse がオプションと誤認するため）。
- 多視点の確認：机の上とロボットの正面側はきれい。撮影で回り込めなかったロボットの背中側や左の壁はぼやける、または真っ白になる。
- 残りの課題：背景に焼き込まれた本物のフォロワーが、シミュレーターのロボットと重なる（手首カメラが白く覆われる）。皿の初期位置が机の縁から少しはみ出している。

### 背景から本物のフォロワーを消す
- `so101_pai/recon/crop.py`：シミュレーター座標の箱の中のガウシアンを不透明度 0 にする（`--box`）。消したところの下の天板は撮影されていないので、隣の天板のガウシアンをずらして複製して埋める（`--patch`）。テスト 4 件。
  - 本物のフォロワーは x −10〜+5 cm、y −15〜+11 cm、z 1〜17 cm にあった（白い点の分布から）。箱 `-0.13,-0.20,0.006,0.08,0.15,0.30` で 114,554 個を隠し、`0.10,-0.09,-0.02,0.31,0.15,0.006` の天板を x 方向に −0.23 m ずらして 7,131 個を複製した。
  - 複製したテンソルは `nn.Parameter` に戻さないと、3DGRUT の書き出しで失敗する。
  - 結果は `assets/scene/workspace_crop.usdz`。真上とロボットの近くからの画像で、白いフォロワーは消え、ロボットの下は木の天板になった。
- `sim/tools/smoke_env.py` に `--pose-deg=a,b,c,d,e,f`（複数指定可）を追加した。アームを指定の関節角まで動かして、両方のカメラを保存する。
- **分かった弱点**：腕を机に伸ばしたときの手首カメラでは、天板が 5〜15 cm の近さに見える。撮影は 0.5〜1 m 離れて行ったので、この距離では天板がオレンジや白の筋状にぼやける。実物の手首カメラには木目がはっきり映るはずなので、ここが手首カメラでの見た目の差（sim2real のギャップ）の主な原因になる。正面カメラから見た画像は問題ない。

### 今の背景で進められるところまで（ユーザーの指示）
- `scripts/lib/common.sh` の `start_sim_bridge` が、`assets/scene/workspace_crop.usdz` があれば自動で背景に使うようにした（`BACKGROUND=none` で背景なし）。テレオペ・評価・通し試験のすべてに効く。
- 皿と積み木の初期位置を、実測した机の上に移した：皿 (0.15, 0.18)、積み木 (−0.08, 0.17)。机はロボットの右側に長く伸びているので、皿は右前、積み木は左前に置いた。実物も同じ位置に置く。
- **軽量描画で暗い問題**：背景のガウシアンは描画モードで見た目が変わらないが、シミュレーターの物体（ロボット・皿）は軽量描画だと暗く、白い皿が灰色で半透明に見えた。天井からの平行光（DistantLight 1500、少し暖色）を足すと、軽量描画でも balanced とほぼ同じ見た目になった。明るさは `PAI_LIGHT_SCALE` で調整できる。
- GUI の視点を、ロボットの少し後ろの上に変えた（画面の左右が操作者のリーダーと一致する）。
- 背景あり・天井の光ありで通し試験：記録は 29.9 Hz を保ち、2 エピソード・306 フレームのデータセットができた。

### GR00T N1.7 の試走（手元、`local/e2e` の 2 エピソード）
- HF のログインと利用規約への同意（ユーザー）のあと、`nvidia/GR00T-N1.7-3B` と `nvidia/Cosmos-Reason2-2B` の重みを取得した。
- `scripts/train/train.sh groot` のレシピどおり（学習するパラメータ 1.62 B、うち大半が拡散の行動ヘッド、fp32）だと、データの読み込み、前処理、モデルの読み込み、順伝播までは通り、逆伝播で GPU メモリ不足（16 GB）になった。オプティマイザの状態まで含めて約 26 GB 必要で、B300 では問題にならない。**手元で GR00T の本番学習はできない**。
- 推論経路の確認のため、行動ヘッドを凍結し重みを bf16 にして（`--policy.tune_diffusion_model=false --policy.model_params_fp32=false`、学習するのは 529 M）2 ステップだけ学習し、チェックポイントを作った。
- `so101_pai.policy.runner.load_policy` で読み込み、データセットの 1 フレームで推論した：6 次元のモーター単位の行動が出た。GPU メモリの最大は 10.1 GB。16 ステップのチャンクの推論に約 115 ms、それ以外のステップは約 7 ms。
  - 30 Hz の制御では、16 ステップごとに約 3 フレーム分止まる。
  - Isaac Sim（GPU メモリ 5〜6 GB）と同時に動かすと 16 GB にぎりぎりなので、シミュレーターでの GR00T の評価は B300 で行うか、メモリを見ながら行う。実機での評価は問題ない。

### リーダーでのテレオペ確認（`check01`、ユーザー）
- ユーザーがリーダーアームでシミュレーターのアームを動かせることを確認した。エピソードは開始していないので、データセットにはしていない（`/episode/status` は 0 件）。
- 記録の頻度は、リーダー 59 Hz、カメラと関節 20.8 Hz（116 秒で 2,425 フレーム）。GUI ありだと、headless の通し試験（29.9 Hz）より遅い。変換は 30 fps に揃えるので記録はできるが、本番の収集の前に GUI ありの速度を確認したい。ログには「CPU の性能プロファイルが powersave」という警告が出ていた。
- **シミュレーターの皿が固定されていなかった**：ログに `Could not perform 'modify_rigid_body_properties' on any prims under: '/World/envs/env_0/Plate'` が出ていた。SimReady の皿は剛体の設定をインスタンス化された子 prim（`Geometry/body_01_obj_00`）に持っていて、Isaac Lab はそこを書き換えられない。そのため `kinematic_enabled=True` が効かず、アームで押すと皿が動く状態だった（10/27 から）。タスク設定に `spawn_fixed_plate` を追加し、子 prim のインスタンス化を解いてから剛体の設定を当てるようにした。`smoke_env.py` が剛体ごとに kinematic かどうかを表示するようにして、皿が `kinematic=[True]` になり警告が消えたことを確かめた。見た目は変わらない。
