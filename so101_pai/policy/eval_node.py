"""Count policy trials and write a JSON summary (see so101_pai.policy.evaluation).

# sim: drives the bridge by itself (run policy_node alongside)
python -m so101_pai.policy.eval_node --ros-args -p mode:=sim -p trials:=20 -p out:=outputs/eval/A_sim.json
# real: a person starts/judges each trial with operator_node (mode:=real); this node only records
python -m so101_pai.policy.eval_node --ros-args -p mode:=real -p trials:=20 -p out:=outputs/eval/B_real.json
"""

from __future__ import annotations

import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from so101_pai.common import contract, episode
from so101_pai.policy.evaluation import EvalSession, write_summary


class EvalNode(Node):
    def __init__(self, **node_kwargs) -> None:
        # node_kwargs (e.g. parameter_overrides) let tests build the node without the command line.
        super().__init__("policy_eval", **node_kwargs)
        self.mode = self.declare_parameter("mode", "sim").value
        self.out = Path(self.declare_parameter("out", "outputs/eval/summary.json").value)
        self.condition = self.declare_parameter("condition", self.out.stem).value
        self.session = EvalSession(
            trials=self.declare_parameter("trials", 20).value,
            settle_s=self.declare_parameter("settle_s", 2.0).value,
            timeout_s=self.declare_parameter("timeout_s", 45.0).value,
        )
        self.command_pub = self.create_publisher(String, contract.TOPIC_EPISODE_COMMAND, 10)
        self.create_subscription(String, contract.TOPIC_EPISODE, self.on_status, 10)
        if self.mode == "sim":
            self.create_timer(0.1, self.tick)

    def tick(self) -> None:
        # The policy node creates its action publisher once the model is loaded; starting earlier wastes trials.
        if not self.session.results and self.count_publishers(contract.TOPIC_ACTION) == 0:
            return
        command = self.session.next_command(time.monotonic())
        if command:
            self.command_pub.publish(String(data=command))

    def on_status(self, msg: String) -> None:
        event = episode.decode(msg.data)
        self.session.on_status(event, time.monotonic())
        if event.event == episode.END and self.session.results:
            last = self.session.results[-1]
            self.get_logger().info(
                f"trial {last.index + 1}/{self.session.trials}: {'success' if last.success else 'fail'} "
                f"({last.duration_s:.1f} s)"
            )
            self.save()  # keep results even if the run is interrupted

    def save(self) -> dict:
        return write_summary(self.out, self.session.results, {"condition": self.condition, "mode": self.mode})


def main() -> None:
    rclpy.init()
    node = EvalNode()
    try:
        while rclpy.ok() and not node.session.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():  # SIGTERM can invalidate the context mid-spin; see common/ros_node.py
            raise
    finally:
        summary = node.save()
        node.get_logger().info(
            f"{summary['successes']}/{summary['trials']} successes ({summary['success_rate']:.0%}) -> {node.out}"
        )
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
