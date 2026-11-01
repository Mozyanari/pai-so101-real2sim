"""Run a PAI Isaac Lab task as a ROS 2 robot that follows so101_pai.common.contract.

    LEISAAC_ROS=1 scripts/sim/run.sh python sim/ros_bridge.py --enable_cameras --rendering_mode performance

Subscribes: /leader/joint_states (absolute joint angles, rad),
            /episode/command ("start" | "success" | "fail" | "reset").
Publishes:  /joint_states, /cam_front/image_raw, /cam_wrist/image_raw (rgb8, 30 Hz), /episode/status.
Episode rules live in so101_pai.common.episode_control: an episode ends with success=true when the task's success
term fires or on "success", and success=false on timeout or "fail". The scene resets after every end.

--expert N: no operator and no leader. The scripted expert (so101_pai.teleop.expert) starts each episode once the
recorder subscribes, plays the pick-and-place, publishes its joint commands on /leader/joint_states as the leader
would, and the bridge quits after N successful episodes (or 2N attempts).
"""

import argparse
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", default="PAI-SO101-PlaceBlock-v0")
parser.add_argument("--step_hz", type=int, default=30, help="control rate (env steps per second)")
parser.add_argument("--expert", type=int, default=0, help="record N successful scripted-expert episodes, then quit")
parser.add_argument("--seed", type=int, default=0, help="expert jitter seed")
parser.add_argument("--noise-deg", type=float, default=0.0, help="expert: std of the DART noise on arm joints (deg)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import leisaac  # noqa: E402,F401
import numpy as np  # noqa: E402
import pai_sim  # noqa: E402,F401
import rclpy  # noqa: E402
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from so101_pai.common import contract, episode  # noqa: E402
from so101_pai.common.episode_control import Decision, EpisodeControl  # noqa: E402
from so101_pai.common.ros_msgs import joint_state_msg  # noqa: E402
from so101_pai.teleop import expert  # noqa: E402

PROFILE_PERIOD_S = 5.0


class SimBridge(Node):
    def __init__(self, env, task_text: str) -> None:
        super().__init__("so101_sim")
        self.env = env
        self.robot = env.scene["robot"]
        self.joint_ids = [self.robot.data.joint_names.index(n) for n in contract.JOINT_NAMES]
        self.control = EpisodeControl(task_text)
        self.target = None
        self.reset_ns = 0  # commands stamped before the last scene reset are stale
        self.commands: list[str] = []
        self.ended: list[bool] = []  # success flag of every episode end, in order

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
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns >= self.reset_ns and all(name in by_name for name in contract.JOINT_NAMES):
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
            self.hold_reset_pose()  # the env has reset itself inside step()

    def apply(self, decision: Decision) -> None:
        for event in decision.events:
            self.status_pub.publish(String(data=episode.encode(event)))
            if event.event == episode.END:
                self.ended.append(bool(event.success))
            self.get_logger().info(f"episode {event.event} success={event.success}")
        if decision.restart_timer:
            self.env.episode_length_buf[:] = 0
        if decision.reset_scene:
            self.env.reset()
            self.hold_reset_pose()

    def hold_reset_pose(self) -> None:
        """After any scene reset, hold the reset pose until a command stamped after the reset arrives.

        Otherwise the arm drives back to the last command of a policy that went idle at the episode end, and the next
        episode starts there, not at rest (the policy stamps actions with the observation they came from).
        """
        self.target = None
        self.reset_ns = self.get_clock().now().nanoseconds

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


class ExpertDriver:
    """--expert: plays the operator (episode start/fail) and the leader (/leader/joint_states) for the bridge.

    Joint commands come from integrating damped-least-squares steps on the TCP pose error, so the commanded pose
    (like a leader arm's) leads the follower, which sags a little under gravity with the sim's soft gains.
    """

    SETTLE_STEPS = 15  # after a reset, before starting the next episode
    GAIN = 0.4  # fraction of the DLS step applied per control step
    MAX_STEP = 0.08  # rad per control step
    ROT_WEIGHT = 0.3  # rotation error (rad) vs position error (m)
    GRACE_S = 1.5  # after the plan ends, wait this long for the success check before calling it a failure

    NOISE_TAU_S = 0.5  # correlation time of the DART noise

    def __init__(self, bridge: SimBridge, episodes: int, seed: int, dt: float, noise_deg: float = 0.0) -> None:
        self.bridge, self.env, self.robot = bridge, bridge.env, bridge.robot
        self.episodes, self.dt = episodes, dt
        self.rng = np.random.default_rng(seed)
        self.body = self.robot.data.body_names.index("gripper")
        self.limits = self.robot.data.soft_joint_pos_limits[0, bridge.joint_ids].cpu().numpy()
        self.leader_pub = bridge.create_publisher(JointState, contract.TOPIC_ACTION, 10)
        self.plan: list[expert.Waypoint] | None = None
        self.idle_steps = 0
        # Arm joints only: noise on the gripper would loosen the grasp.
        sigma = np.radians([noise_deg] * 5 + [0.0])
        self.noise = expert.SmoothNoise(sigma, self.NOISE_TAU_S, dt, self.rng)

    def done(self) -> bool:
        ended = self.bridge.ended
        return sum(ended) >= self.episodes or len(ended) >= 2 * self.episodes

    def recorder_ready(self) -> bool:
        publishers = [self.bridge.status_pub, *self.bridge.image_pubs.values()]
        return all(pub.get_subscription_count() > 0 for pub in publishers)

    def action(self) -> torch.Tensor:
        if self.plan is not None and not self.bridge.control.recording:  # the env ended the episode
            self.plan = None
            self.idle_steps = 0
        if self.plan is None:
            self.bridge.target = None  # ignore our own /leader/joint_states echo: hold the reset pose
            self.idle_steps += 1
            if self.idle_steps < self.SETTLE_STEPS or not self.recorder_ready():
                return self.bridge.action()
            self.begin()
        self.t += self.dt
        if self.t > expert.plan_duration(self.plan) + self.GRACE_S:
            self.bridge.apply(self.bridge.control.on_command("fail"))
            self.plan = None
            self.idle_steps = 0
            return self.bridge.action()
        command = self.track(expert.sample(self.plan, self.start, self.t))
        self.leader_pub.publish(joint_state_msg(self.bridge.get_clock().now().to_msg(), command))
        executed = np.clip(np.asarray(command) + self.noise.step(), self.limits[:, 0], self.limits[:, 1])
        return torch.tensor([executed], dtype=torch.float32, device=self.env.device)

    def begin(self) -> None:
        origin = self.env.scene.env_origins[0]
        block, plate = self.env.scene["block"].data, self.env.scene["plate"].data
        w, x, y, z = block.root_quat_w[0].tolist()
        yaw = float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        self.plan = expert.plan_pick_place(
            (block.root_pos_w[0] - origin).cpu().numpy(),
            yaw,
            (plate.root_pos_w[0] - origin).cpu().numpy(),
            self.rng,
            block_half=getattr(self.env.cfg, "block_size", 2 * expert.BLOCK_HALF) / 2,
        )
        pos, rot = self.tcp_pose()
        gripper = float(self.robot.data.joint_pos[0, self.bridge.joint_ids[-1]])
        self.start = expert.Waypoint(pos, rot, gripper, 0.0)
        self.command = self.robot.data.joint_pos[0, self.bridge.joint_ids].cpu().numpy().astype(np.float64)
        self.t = 0.0
        self.noise.reset()
        # The converter takes each frame's action from the last command at or before it; send one before "start".
        self.leader_pub.publish(joint_state_msg(self.bridge.get_clock().now().to_msg(), self.command))
        self.bridge.apply(self.bridge.control.on_command("start"))

    def tcp_pose(self) -> tuple[np.ndarray, np.ndarray]:
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

    def track(self, target: expert.Waypoint) -> list[float]:
        """Joint command (contract order) one control step closer to the target TCP pose and gripper."""
        pos, rot = self.tcp_pose()
        # Fixed base: PhysX's Jacobian has no row for the root body, so link i is at index i - 1.
        jacobian = self.robot.root_physx_view.get_jacobians()[0, self.body - 1].cpu().numpy().astype(np.float64)
        arm = self.bridge.joint_ids[:5]
        jacobian = expert.tcp_jacobian(jacobian[:, arm], rot @ expert.TCP_OFFSET)
        weights = np.array([1.0, 1.0, 1.0, self.ROT_WEIGHT, self.ROT_WEIGHT, self.ROT_WEIGHT])
        error = expert.pose_error(pos, rot, target.pos, target.rot)
        step = expert.damped_least_squares(weights[:, None] * jacobian, weights * error)
        self.command[:5] += np.clip(self.GAIN * step, -self.MAX_STEP, self.MAX_STEP)
        self.command[5] = target.gripper
        self.command = np.clip(self.command, self.limits[:, 0], self.limits[:, 1])
        return self.command.tolist()


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
    driver = ExpertDriver(bridge, args.expert, args.seed, env.step_dt, args.noise_deg) if args.expert else None
    profile = RateProfile(bridge.get_logger())
    publish_every = max(1, round(args.step_hz / contract.FPS))
    period = 1.0 / args.step_hz
    step = 0
    while app.is_running():
        tick = time.perf_counter()
        rclpy.spin_once(bridge, timeout_sec=0.0)
        bridge.handle_commands()
        obs, _, terminated, truncated, _ = env.step(driver.action() if driver else bridge.action())
        bridge.after_step(terminated, truncated)
        stepped = time.perf_counter()
        if step % publish_every == 0:
            bridge.publish_observation(obs)
        published = time.perf_counter()
        step += 1
        profile.add(stepped - tick, published - stepped)
        time.sleep(max(0.0, period - (published - tick)))
        if driver and driver.done():
            ended = bridge.ended
            bridge.get_logger().info(f"expert done: {sum(ended)} successes in {len(ended)} episodes")
            break

    bridge.destroy_node()
    rclpy.try_shutdown()
    env.close()


if __name__ == "__main__":
    main()
    app.close()
