# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

Workspace for the final project of the Physical AI course (Physical AI 応用編1 知能ロボティクス講座 2026 Summer). The assignment spec is `最終課題（10_19 改訂版）.pdf`. `README.md` explains the layout and usage, and `docs/WORKLOG.md` logs what was done and learned (keep it updated, in Japanese).

## Commands

```bash
source /opt/ros/jazzy/setup.bash && .venv/bin/python -m pytest -q   # all tests; no hardware or sim needed
.venv/bin/python -m pytest -q tests/test_episode_control.py          # one file
.venv/bin/ruff check so101_pai sim tests && .venv/bin/ruff format so101_pai sim tests
scripts/sim/e2e_pipeline.sh          # sim + fake leader -> rosbag -> LeRobot dataset (~2 min, uses the GPU)
scripts/sim/run.sh python ...        # anything that imports Isaac Sim/Lab (conda leisaac, ROS stripped)
```

## Code layout

`so101_pai` (installed editable in both `.venv` and conda `leisaac`) is split by role: `common/` (topic contract, episode events, ROS message conversions, the shared node `run()`, and `episode_control.py`, the sim bridge's episode state machine), `robot/` (hardware nodes + calibration), `teleop/` (keyboard operator), `dataset/` (bag→LeRobot, Hub push), `policy/` (`runner.py` inference, `policy_node`, `evaluation.py` + `eval_node`), `recon/` (3DGS background: `align.py` places it in the sim frame from tape measurements of the table, `crop.py` hides real objects such as the real follower, `usdz.py` resets the transform 3DGRUT puts on the NuRec volume so the USDZ is in COLMAP's frame; run their CLIs in conda `3dgrut` because the checkpoint pickles 3DGRUT's config). Keep decision logic in plain classes with unit tests and the ROS/Isaac code as thin shells; nodes accept `**node_kwargs` so tests pass `parameter_overrides`. `sim/` holds the Isaac side: the task `sim/pai_sim/tasks/place_block`, `sim/ros_bridge.py`, and `sim/tools/`. Launch scripts live in `scripts/{sim,teleop,dataset,train,eval,recon}` and share `scripts/lib/common.sh`.

The sim bridge loads `assets/scene/workspace_crop.usdz` as the background when it exists (`BACKGROUND=none` disables it); `sim/tools/smoke_env.py --view=x,y,z:tx,ty,tz --pose-deg=...` renders debug views. LeIsaac finds its assets via the git root of the cwd, so `scripts/sim/run.sh` sets `LEISAAC_ASSETS_ROOT`.

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

## Project plan (decided 2026-10-27)

`PLAN.md` is the authoritative plan: schedule, decision points, topic contract, evaluation protocol, and deliverables. Keep it updated when decisions change. Two decisions it adds: the real SO-101 connects to ROS 2 through custom rclpy nodes that wrap LeRobot's `SO101Follower`/`SO101Leader` (the leader node is shared by sim and real), and conversion, training, and inference all use LeRobot v0.6.1 (`~/lerobot`).

Goal: rebuild the real SO-101 workspace in simulation (real-to-sim), learn pick-and-place from sim-only teleop data, then measure whether the policy transfers **zero-shot** to the real SO-101.

Task: pick up a toy block and place it on a white paper plate. Object assets are limited to **NVIDIA-official sources**. All are Z-up with metersPerUnit=1.
- Block: the Isaac Sim 5.1 asset library, `Isaac/Props/Blocks/{red,blue,green,yellow}_block.usd`. Each is a 4.7 cm cube with mass 0.02 kg. Base URL: `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/`.
- Place target: `nvidia/simready-assets` → `dishware_plastic_smallPlate_a01` (package metadata says CC-BY-4.0). SimReady has no white round plate, so the sim scales this round 9.9 cm plate by (1.8, 1.8, 0.4) to about 17.8 cm × 1.8 cm and overrides its material to white, to match the real ~18 cm white paper plate (decided 2026-10-27; the blue square `bigPlate_a01` is still fetched). Fix the real plate to the table and make the sim plate kinematic.
- Buy real objects to match these sizes. Do not commit asset files to the public repo: the Isaac Sim asset license and the simready-assets license (its dataset card has none) are unconfirmed. Reference assets by URL or fetch them with a script instead.

Pipeline:

```
phone capture → COLMAP → 3DGRUT (3DGUT) → USDZ background
  + SimReady objects + SO-101 → Isaac Sim 5.1 / Isaac Lab v2.3.0 via LeIsaac (custom task)
  → teleop with SO-101 leader arm → Isaac Sim ROS 2 bridge → rosbag2
  → custom rosbag→LeRobot v3.0 converter → private HF Hub dataset
  → SmolVLA fine-tuned locally (RTX 5060 Ti); GR00T N1.7 (3B) fine-tuned on a B300 cloud, streaming from HF Hub
    (ACT stays available as a small from-scratch baseline; decided 2026-10-27)
  → ROS 2 inference node, the same node for sim and real → evaluation
```

Key design decisions:
- **Data is collected only through rosbag.** Sim and real publish the same ROS 2 topic contract: follower joint states as state, leader joints as action, front and wrist camera images, and an episode start/end/success topic. LeIsaac's built-in LeRobot recorder is deliberately not used, so the converter is on the critical path.
- The converter should write datasets through the LeRobot API (`LeRobotDataset.create` / `add_frame` / `save_episode`), not by hand-writing parquet files.
- Evaluation uses three conditions with 20 trials each: sim success rate, real zero-shot, and real with about 10 real episodes mixed in.
- Fallback: if the 3DGS background does not render in Isaac Lab camera images, switch to LeIsaac's DigitalTwin Env (green-screen with a real background), then to a stock scene.
- Isaac ROS is out of scope for this project.

## Working style for this project

Claude does the environment setup and implementation; on 2026-10-27 the user reversed their original plan to type the commands themselves. Keep every setup step as a reproducible script under `setup/`. Ask the user before accepting licenses (e.g. the Isaac Sim EULA), using sudo, editing `~/.bashrc`, publishing anything, or moving the physical robot.

Reference docs:
- LeIsaac: https://lightwheelai.github.io/leisaac/
- Isaac Lab installation: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html
- Isaac Sim 5.1: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/
- 3DGRUT: https://github.com/nv-tlabs/3DGRUT
- LeRobot: https://huggingface.co/docs/lerobot

## Environment isolation (user requirement)

Never install into the system Python or with apt for this project. No `sudo pip`, no `pip install --user`. Every toolchain gets its own environment; the full table is in `PLAN.md` §2.5:
- conda `leisaac` (Python 3.11): Isaac Sim, Isaac Lab, and LeIsaac.
- conda `3dgrut` (`setup/06_3dgrut_env.sh`): 3DGRUT's conda-managed-CUDA route, since there is no system nvcc and its `.venv` route would accept the CUDA EULA silently.
- `.venv-train` on the B300 (`setup/07_cloud_train_env.sh`).
- conda `colmap` (from conda-forge).
- A repo-local uv `.venv` on `/usr/bin/python3.12` with `--system-site-packages`, for the ROS 2 nodes. It reads rclpy from `/opt/ros/jazzy` and installs `~/lerobot` into the venv.

Do not modify the existing conda envs `lerobot`, `tv`, or `unitree_sim_env`.

`~/.bashrc` sources `/opt/ros/jazzy/setup.bash` globally, which leaks Python 3.12 ROS packages into `PYTHONPATH`. Launch Isaac Sim (Python 3.11) from a shell without ROS sourced.

## Machine environment (verified)

- Ubuntu 24.04, ROS 2 Jazzy at `/opt/ros/jazzy`, Gazebo (`gz`) from the ROS vendor package.
- RTX 5060 Ti 16 GB (Blackwell), driver 580. This GPU meets Isaac Sim's 16 GB VRAM minimum but is below the recommended compute, so keep `num_envs=1` and use modest camera resolutions.
- Conda (miniforge) envs: `lerobot`, `tv`, `unitree_sim_env`. Project envs: `leisaac` (Python 3.11, torch 2.7.0+cu128, `isaacsim[all,extscache]==5.1.0`, numpy pinned to 1.26.0), `colmap`, `3dgrut`.
- `~/lerobot` is a LeRobot checkout at v0.6.1, which already includes `StreamingLeRobotDataset` and `--dataset.streaming=true`. LeIsaac pins `lerobot==0.4.2`, and its async-inference docs target v0.3.3. Pick one LeRobot version for writing datasets and training.
- Isaac Sim uses Python 3.11 and the system Jazzy uses Python 3.12. Use Isaac Sim's bundled ROS 2 (rclpy) inside sim processes rather than sourcing the system install.
