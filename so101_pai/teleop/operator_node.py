"""Keyboard operator for teleop recording (see so101_pai.teleop.operator for the key bindings).

python -m so101_pai.teleop.operator_node --ros-args -p mode:=sim
python -m so101_pai.teleop.operator_node --ros-args -p mode:=real -p task:="Put the red block on the plate."
python -m so101_pai.teleop.operator_node --ros-args -p mode:=real -p rest:=true   # eval: r moves the arm to rest
"""

from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from so101_pai.common import contract, episode
from so101_pai.common.ros_msgs import joint_state_msg, joint_state_to_array
from so101_pai.robot.rest import REST_POSE_DEG, rest_trajectory
from so101_pai.teleop.operator import COMMAND, HELP, REAL, REST, Operator

DEFAULT_TASK = "Put the red block on the plate."


class OperatorNode(Node):
    def __init__(self, **node_kwargs) -> None:
        # node_kwargs (e.g. parameter_overrides) let tests build the node without the command line.
        super().__init__("episode_operator", **node_kwargs)
        mode = self.declare_parameter("mode", "sim").value
        task = self.declare_parameter("task", DEFAULT_TASK).value
        self.operator = Operator(mode, task)
        self.command_pub = self.create_publisher(String, contract.TOPIC_EPISODE_COMMAND, 10)
        self.status_pub = self.create_publisher(String, contract.TOPIC_EPISODE, 10)
        self.create_subscription(String, contract.TOPIC_EPISODE, self.on_status, 10)
        # Only for real eval: with teleop the leader publishes arm commands all the time and would fight the rest move.
        self.rest_enabled = self.declare_parameter("rest", False).value
        # The pose r moves to (deg, JOINT_NAMES order); defaults to the sim demos' rest pose.
        self.rest_pose = list(self.declare_parameter("rest_pose_deg", [float(v) for v in REST_POSE_DEG]).value)
        if len(self.rest_pose) != len(contract.JOINT_NAMES):
            raise ValueError(f"rest_pose_deg needs {len(contract.JOINT_NAMES)} values, got {self.rest_pose}")
        self.rest_plan: list = []
        self.state = None
        if self.rest_enabled:
            self.action_pub = self.create_publisher(JointState, contract.TOPIC_ACTION, 10)
            self.create_subscription(JointState, contract.TOPIC_STATE, self.on_state, 10)
            self.create_timer(1.0 / contract.FPS, self.play_rest)

    def on_state(self, msg: JointState) -> None:
        self.state = joint_state_to_array(msg)

    def play_rest(self) -> None:
        if self.rest_plan and not self.operator.recording:
            self.action_pub.publish(joint_state_msg(self.get_clock().now().to_msg(), self.rest_plan.pop(0)))

    def arm_publishers(self) -> int:
        """Publishers of arm commands other than this node (the policy or the leader)."""
        me = (self.get_name(), self.get_namespace())
        infos = self.get_publishers_info_by_topic(contract.TOPIC_ACTION)
        return sum(1 for info in infos if (info.node_name, info.node_namespace) != me)

    def on_status(self, msg: String) -> None:
        self.operator.on_status(episode.decode(msg.data))
        print(f"\r{self.operator.summary()}  ({HELP})", flush=True)

    def press(self, key: str) -> None:
        # Real mode: /episode/status is not latched, so a start sent before the policy (or leader) node is up is lost
        # to it while the operator already counts the episode as running. Wait until something drives the arm.
        if self.operator.mode == REAL and key == "s" and self.arm_publishers() == 0:
            print("\rnothing publishes arm commands yet (policy still loading?); press s again in a moment", flush=True)
            return
        self.rest_plan = []  # any key stops a rest move in progress
        for out in self.operator.on_key(key):
            if out.channel == REST:
                self.start_rest()
                continue
            publisher = self.command_pub if out.channel == COMMAND else self.status_pub
            publisher.publish(String(data=out.data))

    def start_rest(self) -> None:
        if not self.rest_enabled:
            print("\rr (rest) works only in real eval (scripts/eval/real.sh)", flush=True)
        elif self.state is None:
            print("\rno follower state yet; cannot plan the rest move", flush=True)
        else:
            self.rest_plan = rest_trajectory(self.state, contract.FPS, target_deg=self.rest_pose)
            pose = ", ".join(f"{v:.0f}" for v in self.rest_pose)
            print(f"\rmoving to the rest pose [{pose}] deg ({len(self.rest_plan) / contract.FPS:.1f} s)", flush=True)


def read_key(timeout_s: float) -> str | None:
    ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
    return sys.stdin.read(1) if ready else None


def main() -> None:
    rclpy.init()
    node = OperatorNode()
    print(f"{node.operator.mode} operator: {HELP}", flush=True)
    saved = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(0.02)
            if key in ("q", "\x03"):
                break
            if key:
                node.press(key)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():  # SIGTERM can invalidate the context mid-spin; see common/ros_node.py
            raise
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
