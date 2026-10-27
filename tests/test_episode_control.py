import pytest

from so101_pai.common import episode
from so101_pai.common.episode_control import Decision, EpisodeControl

START = episode.EpisodeEvent(episode.START, task="task")


def end(success: bool) -> episode.EpisodeEvent:
    return episode.EpisodeEvent(episode.END, success=success)


def test_start_once_and_restart_timer():
    control = EpisodeControl("task")
    assert control.on_command("start") == Decision(events=(START,), restart_timer=True)
    assert control.on_command("start") == Decision()


def test_operator_success_and_fail_end_and_reset():
    control = EpisodeControl("task")
    control.on_command("start")
    assert control.on_command("success") == Decision(events=(end(True),), reset_scene=True)
    # Not recording: no event, but the operator still gets a fresh scene.
    assert control.on_command("fail") == Decision(reset_scene=True)


def test_reset_is_ignored_while_recording():
    control = EpisodeControl("task")
    assert control.on_command("reset") == Decision(reset_scene=True)
    control.on_command("start")
    assert control.on_command("reset") == Decision()


def test_env_done_ends_only_a_running_episode():
    control = EpisodeControl("task")
    assert control.on_env_done(True) == Decision()
    control.on_command("start")
    assert control.on_env_done(False) == Decision(events=(end(False),))
    assert not control.recording


def test_unknown_command():
    with pytest.raises(ValueError):
        EpisodeControl("task").on_command("go")
