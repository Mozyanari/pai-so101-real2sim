"""Headless smoke test: build a LeIsaac task, hold the arm still for N steps, save camera frames.

scripts/sim/run.sh python sim/smoke_env.py --task LeIsaac-SO101-PickOrange-v0 --headless --enable_cameras
"""

import argparse
import time
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", default="LeIsaac-SO101-PickOrange-v0")
parser.add_argument("--steps", type=int, default=120)
parser.add_argument("--out", type=Path, default=Path("logs/smoke"))
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import imageio.v2 as imageio  # noqa: E402
import leisaac  # noqa: E402,F401  (registers LeIsaac tasks)
import pai_sim  # noqa: E402,F401  (registers PAI tasks)
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402


def log(message: str) -> None:
    """Kit may swallow stdout, so also append to a file next to the images."""
    print(f"[smoke] {message}", flush=True)
    args.out.mkdir(parents=True, exist_ok=True)
    with open(args.out / "smoke.log", "a") as f:
        f.write(f"{args.task}: {message}\n")


def main() -> None:
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
    env_cfg.use_teleop_device("so101leader")  # absolute joint-position actions, as in leader teleop
    env = gym.make(args.task, cfg=env_cfg).unwrapped
    obs, _ = env.reset()

    robot = env.scene["robot"]
    log(f"joints: {robot.data.joint_names}")
    log(f"action dim: {env.action_manager.total_action_dim}")
    hold = robot.data.joint_pos.clone()

    start = time.time()
    for _ in range(args.steps):
        obs, *_ = env.step(hold)
    elapsed = time.time() - start
    log(f"{args.steps} steps in {elapsed:.1f}s ({args.steps / elapsed:.1f} steps/s)")

    origin = env.scene.env_origins[0]
    log(f"robot base: {(robot.data.root_pos_w[0] - origin).tolist()}")
    for name, rigid in env.scene.rigid_objects.items():
        log(f"{name}: {(rigid.data.root_pos_w[0] - origin).tolist()}")
    if "ee_frame" in env.scene.keys():
        frame = env.scene["ee_frame"]
        for i, name in enumerate(frame.data.target_frame_names):
            log(f"ee {name}: {(frame.data.target_pos_w[0, i] - origin).tolist()}")

    args.out.mkdir(parents=True, exist_ok=True)
    for key, value in obs["policy"].items():
        if getattr(value, "ndim", 0) == 4 and value.shape[-1] == 3:
            path = args.out / f"{args.task}_{key}.png"
            imageio.imwrite(path, value[0].cpu().numpy())
            log(f"saved {path} {tuple(value.shape)}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
