import pytest

from so101_pai.common import episode
from so101_pai.teleop.operator import COMMAND, REAL, REST, SIM, STATUS, Operator, Outgoing


def test_sim_keys_become_bridge_commands():
    op = Operator(SIM, "task")
    assert op.on_key("s") == [Outgoing(COMMAND, "start")]
    assert op.on_key("Y") == [Outgoing(COMMAND, "success")]
    assert op.on_key("n") == [Outgoing(COMMAND, "fail")]
    assert op.on_key("r") == [Outgoing(COMMAND, "reset")]
    assert op.on_key("x") == []


def test_real_mode_publishes_status_and_ignores_out_of_order_keys():
    op = Operator(REAL, "Put the red block on the plate.")
    assert op.on_key("y") == []  # nothing to end yet
    (start,) = op.on_key("s")
    assert start.channel == STATUS
    assert episode.decode(start.data) == episode.EpisodeEvent(episode.START, task="Put the red block on the plate.")
    op.on_status(episode.decode(start.data))
    assert op.on_key("s") == []  # already recording
    assert op.on_key("r") == []  # no rest move while the policy drives the arm
    (end,) = op.on_key("n")
    assert episode.decode(end.data) == episode.EpisodeEvent(episode.END, success=False)


def test_real_mode_r_asks_for_the_rest_pose_between_episodes():
    assert Operator(REAL, "task").on_key("r") == [Outgoing(REST, "")]


def test_counts_follow_status_topic():
    op = Operator(SIM, "task")
    op.on_status(episode.EpisodeEvent(episode.START, task="task"))
    assert op.recording
    op.on_status(episode.EpisodeEvent(episode.END, success=True))
    op.on_status(episode.EpisodeEvent(episode.END, success=False))  # stray end is ignored
    op.on_status(episode.EpisodeEvent(episode.START, task="task"))
    op.on_status(episode.EpisodeEvent(episode.END, success=False))
    assert (op.successes, op.failures, op.recording) == (1, 1, False)
    assert op.summary() == "[idle] success 1 / fail 1"


def test_rejects_unknown_mode():
    with pytest.raises(ValueError):
        Operator("both", "task")
