# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

Workspace for the final project of the Physical AI course (Physical AI 応用編1 知能ロボティクス講座 2026 Summer). The assignment spec is `最終課題（10_19 改訂版）.pdf`. `README.md` explains the layout and usage, `docs/DESIGN.md` the topic contract, environments, evaluation design and what changed from the first plan, `docs/REPRODUCE.md` the exact commands behind the reported results, and `docs/WORKLOG.md` logs what was done and learned (keep it updated, in Japanese).

## Commands

```bash
source /opt/ros/jazzy/setup.bash && .venv/bin/python -m pytest -q   # all tests; no hardware or sim needed
.venv/bin/python -m pytest -q tests/test_episode_control.py          # one file
.venv/bin/ruff check so101_pai sim tests scripts/video && .venv/bin/ruff format so101_pai sim tests scripts/video
scripts/sim/e2e_pipeline.sh          # sim + fake leader -> rosbag -> LeRobot dataset (~2 min, uses the GPU)
scripts/sim/run.sh python ...        # anything that imports Isaac Sim/Lab (conda leisaac, ROS stripped)
```

## Code layout

`so101_pai` (installed editable in both `.venv` and conda `leisaac`) is split by role: `common/` (topic contract, episode events, ROS message conversions, the shared node `run()`, and `episode_control.py`, the sim bridge's episode state machine), `robot/` (hardware nodes + calibration), `teleop/` (keyboard operator, and `expert.py`, the scripted pick-and-place that `sim/ros_bridge.py --expert N` / `scripts/teleop/expert.sh` use to record demos without a human), `dataset/` (bag→LeRobot, Hub push, bag/PNG→MP4 and captions for the project video), `policy/` (`runner.py` inference, `policy_node`, `evaluation.py` + `eval_node`), `recon/` (3DGS background: `align.py` places it in the sim frame from tape measurements of the table, `transfer.py` carries that alignment to a new capture, `crop.py` hides real objects such as the real follower, `color.py` matches the colors to the C920, `usdz.py` resets the transform 3DGRUT puts on the NuRec volume so the USDZ is in COLMAP's frame; run their CLIs in conda `3dgrut` because the checkpoint pickles 3DGRUT's config). Keep decision logic in plain classes with unit tests and the ROS/Isaac code as thin shells; nodes accept `**node_kwargs` so tests pass `parameter_overrides`. `sim/` holds the Isaac side: the task `sim/pai_sim/tasks/place_block`, `sim/ros_bridge.py`, and `sim/tools/`. Launch scripts live in `scripts/{sim,teleop,dataset,train,eval,real,recon,video}` and share `scripts/lib/common.sh`.

The sim loads `assets/scene/workspace2b_v3.usdz` as the background when it exists (`BACKGROUND=none` disables it). Every reported result uses `PAI_LAYOUT=real1031 PAI_PLATE=round PAI_BLOCK_COLOR=wood PAI_BLOCK_SIZE=0.03` (data collection varies the size 0.028-0.032); without them the task falls back to an old layout. `sim/tools/smoke_env.py --view=x,y,z:tx,ty,tz --pose-deg=...` renders debug views. LeIsaac finds its assets via the git root of the cwd, so `scripts/sim/run.sh` sets `LEISAAC_ASSETS_ROOT`.

Process gotchas (all handled in `scripts/lib/common.sh`): background `ros2 bag record` ignores SIGINT, so stop it with SIGTERM; Isaac Sim ignores SIGTERM, so follow with SIGKILL; never `pkill -f` a pattern that also matches the calling shell. The LeIsaac template's HDF5 recorder stalled every reset for 5-6 s and is disabled in the task cfg.

## Assignment constraints (from the PDF)

- **Deadline: 2026-11-02 (Mon) 10:00.**
- Deliverables:
  - A PDF report named `説明資料_PAI最終課題_<omnicampusアカウント名>.pdf`.
  - A **public** GitHub repository holding all code.
  - A 1–3 minute MP4 video on YouTube, titled `PAI最終課題_<アカウント名 or ID>`. If the video is sped up, state the speed (up to 2×).
- Some form of AI is required. Inference alone is enough.
- **No personal information** (name, affiliation, email) may appear in the report, the code, or the video. Submissions may be shared with other course participants.
- Copying a sample project verbatim can fail the course. The report must state what was original. Third-party bases such as LeIsaac must be credited, and the original work must be clearly separated from them.

## Project summary (final, 2026-11-01)

Goal: rebuild the real SO-101 workspace in simulation (real-to-sim), learn pick-and-place from sim data, and test whether the policy transfers to the real SO-101; since it did not transfer zero-shot, test whether sim pretraining helps fine-tuning on 10 real demos. `docs/DESIGN.md` is the reference for the design and the changes from the first plan; `docs/REPRODUCE.md` has the exact commands.

Task: pick up a wooden toy block (about 3 cm) and place it on a white paper plate (about 18 cm). Object assets are limited to **NVIDIA-official sources** and are fetched by `setup/04_task_assets.sh`, never committed (their licenses are unconfirmed):
- Block: Isaac Sim 5.1 `Isaac/Props/Blocks/*_block.usd` (4.7 cm cube), scaled to 2.8-3.2 cm and re-textured as wood.
- Plate: `nvidia/simready-assets` `dishware_plastic_smallPlate_a01`, scaled to about 17.8 cm and made white, kinematic.

Pipeline:

```
phone capture → COLMAP → 3DGRUT (3DGUT) → USDZ background, aligned to the table and with the real arm removed
  + NVIDIA assets + SO-101 → Isaac Sim 5.1 / Isaac Lab v2.3.0 via LeIsaac (custom task)
  → scripted IK expert with DART noise (or leader teleop) → ROS 2 → rosbag2
  → custom rosbag→LeRobot v3.0 converter → ACT (main) / SmolVLA (comparison), LeRobot v0.6.1 (~/lerobot)
  → ROS 2 inference node, the same node for sim and real → evaluation
```

Key design decisions:
- The real SO-101 connects to ROS 2 through custom rclpy nodes that wrap LeRobot's `SO101Follower`/`SO101Leader`; sim and real publish the same topic contract, so recording, conversion, inference and evaluation code is shared.
- **Data is collected only through rosbag**; LeIsaac's LeRobot recorder is not used. The converter writes through the LeRobot API (`LeRobotDataset.create` / `add_frame` / `save_episode`).
- Sim demos come from a scripted expert (`scripts/teleop/expert.sh`), not human teleop.
- Results: ACT trained on 225 sim demos succeeds 13/20 in sim and fails zero-shot on the real robot; with 10 real demos, the sim-pretrained ACT (B) beats the real-only ACT (A) at every checkpoint (3 trials each). Real runs use `ACTION_STEPS=30 REST_POSE=start` and the camera/plate/door layout of the real demos.
- Isaac ROS is out of scope.

## Working style for this project

Write everything addressed to the user in Japanese: replies, progress reports, questions, and summaries, including updates during long-running work. Keep code identifiers and technical terms as they are. Code, comments, and commit messages stay in English, and `docs/WORKLOG.md` stays in Japanese.

Claude does the environment setup and implementation; on 2026-10-27 the user reversed their original plan to type the commands themselves. Keep every setup step as a reproducible script under `setup/`. Ask the user before accepting licenses (e.g. the Isaac Sim EULA), using sudo, editing `~/.bashrc`, publishing anything, or moving the physical robot. The user runs the real robot themselves; Claude may only read joint states.

Reference docs:
- LeIsaac: https://lightwheelai.github.io/leisaac/
- Isaac Lab installation: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html
- Isaac Sim 5.1: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/
- 3DGRUT: https://github.com/nv-tlabs/3DGRUT
- LeRobot: https://huggingface.co/docs/lerobot

## Environment isolation (user requirement)

Never install into the system Python or with apt for this project. No `sudo pip`, no `pip install --user`. Every toolchain gets its own environment; the full table is in `docs/DESIGN.md` §3:
- conda `leisaac` (Python 3.11): Isaac Sim, Isaac Lab, and LeIsaac.
- conda `3dgrut` (`setup/06_3dgrut_env.sh`): 3DGRUT's conda-managed-CUDA route, since there is no system nvcc and its `.venv` route would accept the CUDA EULA silently.
- conda `colmap` (from conda-forge).
- A repo-local uv `.venv` on `/usr/bin/python3.12` with `--system-site-packages`, for the ROS 2 nodes. It reads rclpy from `/opt/ros/jazzy` and installs `~/lerobot` into the venv.

Do not modify the existing conda envs `lerobot`, `tv`, or `unitree_sim_env`.

`~/.bashrc` sources `/opt/ros/jazzy/setup.bash` globally, which leaks Python 3.12 ROS packages into `PYTHONPATH`. Launch Isaac Sim (Python 3.11) from a shell without ROS sourced.

## Machine environment (verified)

- Ubuntu 24.04, ROS 2 Jazzy at `/opt/ros/jazzy`, Gazebo (`gz`) from the ROS vendor package.
- RTX 5060 Ti 16 GB (Blackwell), driver 580. This GPU meets Isaac Sim's 16 GB VRAM minimum but is below the recommended compute, so keep `num_envs=1` and use modest camera resolutions.
- Conda (miniforge) envs: `lerobot`, `tv`, `unitree_sim_env`. Project envs: `leisaac` (Python 3.11, torch 2.7.0+cu128, `isaacsim[all,extscache]==5.1.0`, numpy pinned to 1.26.0), `colmap`, `3dgrut`.
- `~/lerobot` is a LeRobot checkout at v0.6.1, used for writing datasets, training and inference. LeIsaac pins `lerobot==0.4.2`, so LeRobot is not installed into conda `leisaac`.
- Isaac Sim uses Python 3.11 and the system Jazzy uses Python 3.12. Use Isaac Sim's bundled ROS 2 (rclpy) inside sim processes rather than sourcing the system install.
