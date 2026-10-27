"""Run a PAI Isaac Lab task as a ROS 2 robot that follows so101_pai.common.contract.

    LEISAAC_ROS=1 scripts/sim/run.sh python sim/ros_bridge.py --enable_cameras --rendering_mode performance

Subscribes: /leader/joint_states (absolute joint angles, rad),
            /episode/command ("start" | "success" | "fail" | "reset").
Publishes:  /joint_states, /cam_front/image_raw, /cam_wrist/image_raw (rgb8, 30 Hz), /episode/status.
Episode rules live in so101_pai.common.episode_control: an episode ends with success=true when the task's success
term fires or on "success", and success=false on timeout or "fail". The scene resets after every end (~6 s stall).
"""

import argparse
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", default="PAI-SO101-PlaceBlock-v0")
parser.add_argument("--step_hz", type=int, default=30, help="control rate (env steps per second)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import leisaac  # noqa: E402,F401
import pai_sim  # noqa: E402,F401
import rclpy  # noqa: E402
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from so101_pai.common import contract, episode  # noqa: E402
from so101_pai.common.episode_control import Decision, EpisodeControl  # noqa: E402

PROFILE_PERIOD_S = 5.0


class SimBridge(Node):
    def __init__(self, env, task_text: str) -> None:
        super().__init__("so101_sim")
        self.env = env
        self.robot = env.scene["robot"]
        self.joint_ids = [self.robot.data.joint_names.index(n) for n in contract.JOINT_NAMES]
        self.control = EpisodeControl(task_text)
        self.target = None
        self.commands: list[str] = []

        self.state_pub = self.create_publisher(JointState, contract.TOPIC_STATE, 10)
        self.status_pub = self.create_publisher(String, contract.TOPIC_EPISODE, 10)
        self.image_pubs = {
            cam: self.create_publisher(Image, topic, 10) for cam, topic in contract.CAMERA_TOPICS.items()
        }
        self.create_subscription(JointState, contract.TOPIC_ACTION, self.on_action, 10)
        self.create_subscription(String, contract.TOPIC_EPISODE_COMMAND, self.on_command, 10)

    # --- ROS callbacks (only store; the main loop acts between env steps) ---
    def on_action(self, msg: JointState) -> None:
        # Lenient on purpose: a malformed command must not raise inside the sim's main loop.
        by_name = dict(zip(msg.name, msg.position, strict=False))
        if all(name in by_name for name in contract.JOINT_NAMES):
            self.target = [by_name[name] for name in contract.JOINT_NAMES]

    def on_command(self, msg: String) -> None:
        self.commands.append(msg.data.strip())

    # --- main loop helpers ---
    def action(self) -> torch.Tensor:
        """Leader command in env action order (contract joint order); hold the current pose until one arrives."""
        if self.target is None:
            return self.robot.data.joint_pos[:, self.joint_ids].clone()
        return torch.tensor([self.target], dtype=torch.float32, device=self.env.device)

    def handle_commands(self) -> None:
        while self.commands:
            command = self.commands.pop(0)
            try:
                self.apply(self.control.on_command(command))
            except ValueError as error:
                self.get_logger().warning(str(error))

    def after_step(self, terminated, truncated) -> None:
        if terminated.any() or truncated.any():
            success = bool(self.env.termination_manager.get_term("success")[0])
            self.apply(self.control.on_env_done(success))

    def apply(self, decision: Decision) -> None:
        for event in decision.events:
            self.status_pub.publish(String(data=episode.encode(event)))
            self.get_logger().info(f"episode {event.event} success={event.success}")
        if decision.restart_timer:
            self.env.episode_length_buf[:] = 0
        if decision.reset_scene:
            self.env.reset()

    def publish_observation(self, obs) -> None:
        stamp = self.get_clock().now().to_msg()
        state = JointState()
        state.header.stamp = stamp
        state.name = list(contract.JOINT_NAMES)
        state.position = self.robot.data.joint_pos[0, self.joint_ids].tolist()
        self.state_pub.publish(state)
        for cam, publisher in self.image_pubs.items():
            rgb = obs["policy"][cam][0].to(torch.uint8).cpu().numpy()
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = f"{cam}_camera"
            msg.height, msg.width = rgb.shape[:2]
            msg.encoding, msg.step = "rgb8", rgb.shape[1] * 3
            msg.data = rgb.tobytes()
            publisher.publish(msg)


class RateProfile:
    """Logs loop rate and where the time goes, every PROFILE_PERIOD_S."""

    def __init__(self, logger) -> None:
        self.logger = logger
        self._reset(time.perf_counter())

    def add(self, step_s: float, publish_s: float) -> None:
        self.steps += 1
        self.step_s += step_s
        self.publish_s += publish_s
        now = time.perf_counter()
        if now - self.since >= PROFILE_PERIOD_S:
            n = self.steps
            self.logger.info(
                f"rate {n / (now - self.since):.1f} Hz | step {1e3 * self.step_s / n:.1f} ms"
                f" | publish {1e3 * self.publish_s / n:.1f} ms"
            )
            self._reset(now)

    def _reset(self, now: float) -> None:
        self.since, self.steps, self.step_s, self.publish_s = now, 0, 0.0, 0.0


def main() -> None:
    env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
    env_cfg.use_teleop_device("so101leader")
    env = gym.make(args.task, cfg=env_cfg).unwrapped
    obs, _ = env.reset()

    rclpy.init()
    bridge = SimBridge(env, env_cfg.task_description)
    profile = RateProfile(bridge.get_logger())
    publish_every = max(1, round(args.step_hz / contract.FPS))
    period = 1.0 / args.step_hz
    step = 0
    while app.is_running():
        tick = time.perf_counter()
        rclpy.spin_once(bridge, timeout_sec=0.0)
        bridge.handle_commands()
        obs, _, terminated, truncated, _ = env.step(bridge.action())
        bridge.after_step(terminated, truncated)
        stepped = time.perf_counter()
        if step % publish_every == 0:
            bridge.publish_observation(obs)
        published = time.perf_counter()
        step += 1
        profile.add(stepped - tick, published - stepped)
        time.sleep(max(0.0, period - (published - tick)))

    bridge.destroy_node()
    rclpy.try_shutdown()
    env.close()


if __name__ == "__main__":
    main()
    app.close()
