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
parser.add_argument(
    "--view",
    action="append",
    help="debug: render the front camera from 'x,y,z:tx,ty,tz' (eye : look-at, meters, env frame); repeatable",
)
AppLauncher.add_app_launcher_args(parser)
parser.add_argument(
    "--pose-deg",
    action="append",
    help="debug: drive the arm to 6 comma-separated joint angles (deg) and save both cameras; repeatable",
)
args = parser.parse_args()
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import imageio.v2 as imageio  # noqa: E402
import leisaac  # noqa: E402,F401  (registers LeIsaac tasks)
import pai_sim  # noqa: E402,F401  (registers PAI tasks)
import torch  # noqa: E402
from isaaclab.sim.utils import find_matching_prims  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402


def log(message: str) -> None:
    """Kit may swallow stdout, so also append to a file next to the images."""
    print(f"[smoke] {message}", flush=True)
    args.out.mkdir(parents=True, exist_ok=True)
    with open(args.out / "smoke.log", "a") as f:
        f.write(f"{args.task}: {message}\n")


def look_at_quat(eye, target, up=(0.0, 0.0, 1.0)):
    """(w, x, y, z) of an OpenGL-convention camera (looks along -Z, +Y up) at `eye` looking at `target`."""
    import numpy as np

    from so101_pai.recon.align import _quat_wxyz

    forward = np.subtract(target, eye) / np.linalg.norm(np.subtract(target, eye))
    up = np.asarray(up, dtype=float)
    if abs(forward @ up) > 0.99:  # looking straight down: keep +y up in the image
        up = np.array([0.0, 1.0, 0.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    return tuple(_quat_wxyz(np.column_stack([right, np.cross(right, forward), -forward])))


def main() -> None:
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
    views = [tuple(tuple(float(v) for v in part.split(",")) for part in view.split(":")) for view in args.view or []]
    if views:
        eye, target = views[0]
        env_cfg.scene.front.prim_path = "{ENV_REGEX_NS}/ViewCamera"
        env_cfg.scene.front.offset.pos = eye
        env_cfg.scene.front.offset.rot = look_at_quat(eye, target)
        env_cfg.scene.front.offset.convention = "opengl"
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
        root = find_matching_prims(rigid.cfg.prim_path)[0]
        bodies = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        kinematic = [bool(p.GetAttribute("physics:kinematicEnabled").Get()) for p in bodies]
        log(f"{name}: {(rigid.data.root_pos_w[0] - origin).tolist()} kinematic={kinematic}")
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
    for i, pose in enumerate(args.pose_deg or []):
        target = torch.deg2rad(torch.tensor([[float(v) for v in pose.split(",")]], device=env.device))
        for _ in range(60):  # ~2 s at 30 Hz to get there
            obs, *_ = env.step(target)
        for key in ("front", "wrist"):
            path = args.out / f"pose_{i:02d}_{key}.png"
            imageio.imwrite(path, obs["policy"][key][0].cpu().numpy())
        log(f"saved pose {i}: {pose} deg")
    camera = env.scene["front"]
    for i, (eye, target) in enumerate(views):
        camera.set_world_poses_from_view(
            torch.tensor([eye], device=env.device) + origin, torch.tensor([target], device=env.device) + origin
        )
        for _ in range(3):  # let the renderer settle on the new pose
            obs, *_ = env.step(hold)
        path = args.out / f"view_{i:02d}.png"
        imageio.imwrite(path, obs["policy"]["front"][0].cpu().numpy())
        log(f"saved {path} from {eye} looking at {target}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
