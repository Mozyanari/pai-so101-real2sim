"""Keyboard operator for teleop recording (see so101_pai.teleop.operator for the key bindings).

python -m so101_pai.teleop.operator_node --ros-args -p mode:=sim
python -m so101_pai.teleop.operator_node --ros-args -p mode:=real -p task:="Put the red block on the plate."
"""

from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from so101_pai.common import contract, episode
from so101_pai.teleop.operator import COMMAND, HELP, Operator

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

    def on_status(self, msg: String) -> None:
        self.operator.on_status(episode.decode(msg.data))
        print(f"\r{self.operator.summary()}  ({HELP})", flush=True)

    def press(self, key: str) -> None:
        for out in self.operator.on_key(key):
            publisher = self.command_pub if out.channel == COMMAND else self.status_pub
            publisher.publish(String(data=out.data))


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
