"""Time env.reset() and the env steps right after it, and print a cProfile of those steps.

    scripts/sim/run.sh python sim/tools/profile_reset.py --headless --enable_cameras --rendering_mode performance

Writes logs/profile_reset.txt. Used on 2026-10-27 to find the 5-6 s reset stall (it was the HDF5 recorder).
"""

import argparse
import cProfile
import io
import pstats
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", default="PAI-SO101-PlaceBlock-v0")
parser.add_argument("--steps", type=int, default=60)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import leisaac  # noqa: E402,F401
import pai_sim  # noqa: E402,F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402


def timed(fn) -> float:
    start = time.perf_counter()
    fn()
    return time.perf_counter() - start


def main() -> None:
    cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
    cfg.use_teleop_device("so101leader")
    env = gym.make(args.task, cfg=cfg).unwrapped
    env.reset()
    robot = env.scene["robot"]

    def hold() -> None:
        env.step(robot.data.joint_pos[:, :6].clone())

    lines = [f"reset {i}: {timed(env.reset):.3f} s" for i in range(3)]
    for k in range(2):
        env.reset()
        times = [timed(hold) for _ in range(args.steps)]
        slow = [f"{i}:{t:.2f}" for i, t in enumerate(times) if t > 0.1]
        lines.append(f"after reset {k}: {sum(times):.2f} s for {args.steps} steps, slow steps {slow}")

    env.reset()
    profiler = cProfile.Profile()
    profiler.enable()
    for _ in range(args.steps):
        hold()
    profiler.disable()
    stats = io.StringIO()
    pstats.Stats(profiler, stream=stats).sort_stats("cumulative").print_stats(25)
    with open("logs/profile_reset.txt", "w") as f:
        f.write("\n".join(lines) + "\n" + stats.getvalue())
    env.close()


if __name__ == "__main__":
    main()
    app.close()
