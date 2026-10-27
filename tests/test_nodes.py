"""In-process tests of the thin ROS node shells (no hardware, no sim)."""

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from so101_pai.common import contract, episode


@pytest.fixture(autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_operator_node_routes_keys_to_topics():
    from so101_pai.teleop.operator_node import OperatorNode

    node = OperatorNode()
    sent = []
    node.command_pub.publish = lambda msg: sent.append(("command", msg.data))
    node.status_pub.publish = lambda msg: sent.append(("status", msg.data))
    node.press("s")
    node.press("x")
    assert sent == [("command", "start")]
    node.destroy_node()


def test_eval_node_counts_trials_and_saves(tmp_path):
    from rclpy.parameter import Parameter

    from so101_pai.policy.eval_node import EvalNode

    out = tmp_path / "eval.json"
    node = EvalNode(parameter_overrides=[Parameter("out", value=str(out)), Parameter("trials", value=1)])
    node.on_status(String(data=episode.encode(episode.EpisodeEvent(episode.START, task="t"))))
    node.on_status(String(data=episode.encode(episode.EpisodeEvent(episode.END, success=True))))
    assert node.session.done
    assert '"success_rate": 1.0' in out.read_text()
    node.destroy_node()


class FakeRunner:
    def __init__(self) -> None:
        self.resets = 0

    def reset(self) -> None:
        self.resets += 1

    def act(self, frame):
        return frame["observation.state"] + 1.0


def test_policy_node_acts_only_during_episodes(monkeypatch):
    from rclpy.parameter import Parameter

    import so101_pai.policy.policy_node as policy_node

    monkeypatch.setattr(policy_node, "load_policy", lambda path, device, task: FakeRunner())
    node = policy_node.PolicyNode(parameter_overrides=[Parameter("policy_path", value="ckpt")])
    sent = []
    node.action_pub.publish = lambda msg: sent.append(np.array(msg.position))

    state = JointState(name=list(contract.JOINT_NAMES), position=[0.0] * 6)
    node.on_state(state)
    for camera in contract.CAMERA_TOPICS:
        node.buffer.images[camera] = np.zeros((480, 640, 3), np.uint8)
    node.tick()
    assert sent == []  # no episode yet

    node.on_status(String(data=episode.encode(episode.EpisodeEvent(episode.START, task="t"))))
    node.tick()
    assert node.runner.resets == 1 and len(sent) == 1
    expected = contract.motor_to_radians(contract.radians_to_motor(np.zeros(6)) + 1.0)
    np.testing.assert_allclose(sent[0], expected, atol=1e-5)

    node.on_status(String(data=episode.encode(episode.EpisodeEvent(episode.END, success=False))))
    node.tick()
    assert len(sent) == 1
    node.destroy_node()
