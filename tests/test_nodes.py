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


def test_real_operator_waits_for_an_arm_command_publisher_before_starting():
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState

    from so101_pai.teleop.operator_node import OperatorNode

    node = OperatorNode(parameter_overrides=[Parameter("mode", value="real")])
    sent = []
    node.status_pub.publish = lambda msg: sent.append(msg.data)
    node.press("s")
    assert sent == [] and not node.operator.recording  # policy not up yet: the start would be lost
    policy_node = rclpy.create_node("policy_stub")
    policy_node.create_publisher(JointState, contract.TOPIC_ACTION, 10)
    node.press("s")
    assert len(sent) == 1 and episode.decode(sent[0]).event == episode.START
    policy_node.destroy_node()
    node.destroy_node()


def test_real_eval_operator_moves_the_arm_to_rest_between_trials():
    from rclpy.parameter import Parameter

    from so101_pai.robot.rest import REST_POSE_DEG
    from so101_pai.teleop.operator_node import OperatorNode

    node = OperatorNode(parameter_overrides=[Parameter("mode", value="real"), Parameter("rest", value=True)])
    assert node.arm_publishers() == 0  # its own rest publisher does not count as the policy
    sent = []
    node.action_pub.publish = lambda msg: sent.append(np.array(msg.position))
    node.press("r")
    assert not node.rest_plan  # no follower state yet
    node.on_state(JointState(name=list(contract.JOINT_NAMES), position=[0.3, 0.0, 0.0, 0.0, 0.0, 0.5]))
    node.press("r")
    while node.rest_plan:
        node.play_rest()
    np.testing.assert_allclose(sent[-1], np.radians(REST_POSE_DEG), atol=1e-6)
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


def test_eval_node_waits_for_the_policy_before_the_first_reset(tmp_path):
    from rclpy.parameter import Parameter

    from so101_pai.policy.eval_node import EvalNode

    node = EvalNode(parameter_overrides=[Parameter("out", value=str(tmp_path / "eval.json"))])
    sent = []
    node.command_pub.publish = lambda msg: sent.append(msg.data)
    node.tick()
    assert sent == []  # no policy publishing actions yet
    policy = node.create_publisher(JointState, contract.TOPIC_ACTION, 10)
    node.tick()
    assert sent == ["reset"]
    node.destroy_publisher(policy)
    node.destroy_node()


class FakeRunner:
    n_action_steps = 4

    def __init__(self) -> None:
        self.resets = 0

    def reset(self) -> None:
        self.resets += 1

    def act(self, frame):
        return frame["observation.state"] + 1.0

    def predict_chunk(self, frame):
        return frame["observation.state"] + np.arange(1.0, 9.0)[:, None]  # 8 actions: state + 1, + 2, ...


def make_policy_node(monkeypatch, asynchronous):
    from rclpy.parameter import Parameter

    import so101_pai.policy.policy_node as policy_node

    monkeypatch.setattr(policy_node, "load_policy", lambda path, device, task, n_action_steps: FakeRunner())
    node = policy_node.PolicyNode(
        parameter_overrides=[
            Parameter("policy_path", value="ckpt"),
            Parameter("asynchronous", value=asynchronous),
            Parameter("lead_steps", value=1),
        ]
    )
    sent = []
    node.action_pub.publish = lambda msg: sent.append(msg)
    state = JointState(name=list(contract.JOINT_NAMES), position=[0.0] * 6)
    state.header.stamp.sec = 42
    node.on_state(state)
    for camera in contract.CAMERA_TOPICS:
        node.buffer.images[camera] = np.zeros((480, 640, 3), np.uint8)
    return node, sent


def status(event, **kwargs):
    return String(data=episode.encode(episode.EpisodeEvent(event, **kwargs)))


def test_policy_node_acts_only_during_episodes(monkeypatch):
    node, sent = make_policy_node(monkeypatch, asynchronous=False)
    node.tick()
    assert sent == []  # no episode yet

    node.on_status(status(episode.START, task="t"))
    node.tick()
    assert node.runner.resets == 1 and len(sent) == 1
    expected = contract.motor_to_radians(contract.radians_to_motor(np.zeros(6)) + 1.0)
    np.testing.assert_allclose(sent[0].position, expected, atol=1e-5)
    assert sent[0].header.stamp.sec == 42  # stamped with the observation it came from

    node.on_status(status(episode.END, success=False))
    node.tick()
    assert len(sent) == 1
    node.destroy_node()


def test_policy_node_async_plays_chunks_while_inferring(monkeypatch):
    node, sent = make_policy_node(monkeypatch, asynchronous=True)
    node.on_status(status(episode.START, task="t"))
    node.tick()  # starts the first inference; nothing to play yet
    node.inference.result()
    assert node.runner.resets == 1  # the reset ran on the worker before the inference
    played = []
    for _ in range(4):
        node.tick()
        if node.inference is not None:
            node.inference.result()  # make the worker deterministic for the test
        played.append(contract.radians_to_motor(np.asarray(sent[-1].position))[0])
    # Chunk from tick 0, collected at tick 1: actions 1..3 (+2..+4), then the next chunk from tick 3 at tick 4.
    np.testing.assert_allclose(np.array(played) - contract.radians_to_motor(np.zeros(6))[0], [2, 3, 4, 2], atol=1e-3)
    assert sent[0].header.stamp.sec == 42

    node.on_status(status(episode.END, success=True))
    count = len(sent)
    node.tick()
    assert len(sent) == count
    node.destroy_node()


def test_rest_node_plays_the_trajectory_once_then_finishes():
    from so101_pai.robot.rest import REST_POSE_DEG, RestNode

    node = RestNode()
    sent = []
    node.action_pub.publish = lambda msg: sent.append(np.array(msg.position))
    node.tick()
    assert not node.done and sent == []  # waits for the follower state
    node.on_state(JointState(name=list(contract.JOINT_NAMES), position=[0.0, -1.0, 1.0, 0.5, 0.0, 0.0]))
    node.on_state(JointState(name=list(contract.JOINT_NAMES), position=[1.0] * 6))  # later states do not replan
    while not node.done:
        node.tick()
    np.testing.assert_allclose(sent[-1], np.radians(REST_POSE_DEG), atol=1e-6)
    node.destroy_node()
