"""Render the sim scene (3DGS background + SO-101 + block and plate) from a camera that circles the table.

scripts/sim/run.sh python sim/tools/orbit_video.py --headless --enable_cameras --out logs/orbit
Frames go to <out>/frame_XXXX.png (full HD by default); encode them with the .venv (PyAV, H.264):
.venv/bin/python -m so101_pai.dataset.frames2video logs/orbit outputs/video/orbit.mp4

The camera moves on an arc around --center (env frame: robot base at the origin, +x right, +y forward, table z=0),
from --start-deg to --end-deg (yaw from +x towards +y) at --radius and --cam-height, always looking at --center.
With --pick the arm does one scripted pick-and-place (so101_pai.teleop.expert, as in sim/ros_bridge.py --expert) while
the camera moves, and the clip lasts as long as the motion.
"""

import argparse
import math
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", default="PAI-SO101-PlaceBlock-v0")
parser.add_argument("--out", type=Path, default=Path("logs/orbit"))
parser.add_argument("--center", default="0.03,0.15,0.02", help="look-at point x,y,z (m)")
parser.add_argument("--radius", type=float, default=0.75)
parser.add_argument("--cam-height", type=float, default=0.40, help="camera height above the table (m)")
parser.add_argument("--start-deg", type=float, default=-40.0)
parser.add_argument("--end-deg", type=float, default=130.0)
parser.add_argument("--frames", type=int, default=300, help="10 s at 30 fps")
parser.add_argument("--size", default="1920x1080", help="WxH of the rendered frames (full HD)")
parser.add_argument("--stills", type=int, default=0, help="only render this many evenly spaced frames (preview)")
parser.add_argument("--pick", action="store_true", help="the arm picks the block and places it on the plate")
parser.add_argument("--seed", type=int, default=0, help="--pick: seed of the expert's plan variations")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import imageio.v2 as imageio  # noqa: E402
import leisaac  # noqa: E402,F401  (registers LeIsaac tasks)
import numpy as np  # noqa: E402
import pai_sim  # noqa: E402,F401  (registers PAI tasks)
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

from so101_pai.common import contract  # noqa: E402
from so101_pai.teleop import expert  # noqa: E402


class PickDriver:
    """One scripted pick-and-place: the IK tracking of sim/ros_bridge.py's ExpertDriver, without ROS."""

    GAIN, MAX_STEP, ROT_WEIGHT = 0.4, 0.08, 0.3  # same as ExpertDriver

    def __init__(self, env, seed: int) -> None:
        self.env, self.robot = env, env.scene["robot"]
        self.joint_ids = [self.robot.data.joint_names.index(n) for n in contract.JOINT_NAMES]
        self.body = self.robot.data.body_names.index("gripper")
        self.limits = self.robot.data.soft_joint_pos_limits[0, self.joint_ids].cpu().numpy()
        origin = env.scene.env_origins[0]
        block, plate = env.scene["block"].data, env.scene["plate"].data
        w, x, y, z = block.root_quat_w[0].tolist()
        yaw = float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        self.plan = expert.plan_pick_place(
            (block.root_pos_w[0] - origin).cpu().numpy(),
            yaw,
            (plate.root_pos_w[0] - origin).cpu().numpy(),
            np.random.default_rng(seed),
            block_half=getattr(env.cfg, "block_size", 2 * expert.BLOCK_HALF) / 2,
        )
        pos, rot = self.tcp_pose()
        self.start = expert.Waypoint(pos, rot, float(self.robot.data.joint_pos[0, self.joint_ids[-1]]), 0.0)
        self.command = self.robot.data.joint_pos[0, self.joint_ids].cpu().numpy().astype(np.float64)
        self.duration = expert.plan_duration(self.plan)

    def tcp_pose(self):
        data = self.robot.data
        body_pos = (data.body_pos_w[0, self.body] - self.env.scene.env_origins[0]).cpu().numpy()
        w, x, y, z = data.body_quat_w[0, self.body].tolist()
        rot = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )
        return body_pos + rot @ expert.TCP_OFFSET, rot

    def action(self, t: float) -> torch.Tensor:
        target = expert.sample(self.plan, self.start, min(t, self.duration))
        pos, rot = self.tcp_pose()
        jacobian = self.robot.root_physx_view.get_jacobians()[0, self.body - 1].cpu().numpy().astype(np.float64)
        jacobian = expert.tcp_jacobian(jacobian[:, self.joint_ids[:5]], rot @ expert.TCP_OFFSET)
        weights = np.array([1.0, 1.0, 1.0, self.ROT_WEIGHT, self.ROT_WEIGHT, self.ROT_WEIGHT])
        error = expert.pose_error(pos, rot, target.pos, target.rot)
        step = expert.damped_least_squares(weights[:, None] * jacobian, weights * error)
        self.command[:5] += np.clip(self.GAIN * step, -self.MAX_STEP, self.MAX_STEP)
        self.command[5] = target.gripper
        self.command = np.clip(self.command, self.limits[:, 0], self.limits[:, 1])
        return torch.tensor([self.command], dtype=torch.float32, device=self.env.device)


def main() -> None:
    width, height = (int(v) for v in args.size.split("x"))
    center = [float(v) for v in args.center.split(",")]
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
    camera_cfg = env_cfg.scene.front
    camera_cfg.prim_path = "{ENV_REGEX_NS}/OrbitCamera"  # a free camera, not the one the policy uses
    camera_cfg.width, camera_cfg.height = width, height
    for name, term in vars(env_cfg.events).items():  # keep the scene still: no camera jitter
        if getattr(term, "func", None) is not None and term.func.__name__ == "randomize_camera_uniform":
            setattr(env_cfg.events, name, None)
    if args.pick:  # a success or timeout would reset the scene mid-clip and put the block back
        for name in list(vars(env_cfg.terminations)):
            setattr(env_cfg.terminations, name, None)
    env_cfg.use_teleop_device("so101leader")  # absolute joint-position actions: hold the reset pose
    env = gym.make(args.task, cfg=env_cfg).unwrapped
    env.reset()
    robot = env.scene["robot"]
    hold = robot.data.joint_pos.clone()
    for _ in range(30):  # let the arm settle
        env.step(hold)

    origin = env.scene.env_origins[0]
    camera = env.scene["front"]
    target = torch.tensor([center], device=env.device) + origin
    driver = PickDriver(env, args.seed) if args.pick else None
    if driver is not None and not args.stills:
        args.frames = int((driver.duration + 1.0) / env.step_dt)  # the whole motion and a second of rest
    count = args.stills or args.frames
    args.out.mkdir(parents=True, exist_ok=True)
    for i in range(count):
        t = i / max(1, count - 1)
        yaw = math.radians(args.start_deg + (args.end_deg - args.start_deg) * (0.5 - 0.5 * math.cos(math.pi * t)))
        eye = [center[0] + args.radius * math.cos(yaw), center[1] + args.radius * math.sin(yaw), args.cam_height]
        camera.set_world_poses_from_view(torch.tensor([eye], device=env.device) + origin, target)
        action = driver.action(i * env.step_dt) if driver is not None else hold
        obs, *_ = env.step(action)
        if args.stills or i == 0:  # the first frame after a jump renders the old pose; step once more
            obs, *_ = env.step(hold)
        imageio.imwrite(args.out / f"frame_{i:04d}.png", obs["policy"]["front"][0].cpu().numpy())
    print(f"[orbit] wrote {count} frames to {args.out} (step_dt {env.step_dt:.4f} s)", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    app.close()
