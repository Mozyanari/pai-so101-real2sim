import json

from so101_pai.common import episode
from so101_pai.policy.evaluation import DONE, RUNNING, EvalSession, Trial, summarize, write_summary

END_OK = episode.EpisodeEvent(episode.END, success=True)
END_FAIL = episode.EpisodeEvent(episode.END, success=False)


def test_sim_session_resets_settles_then_starts():
    session = EvalSession(trials=2, settle_s=8, timeout_s=35)
    assert session.next_command(0.0) == "reset"
    assert session.next_command(7.9) is None
    assert session.next_command(8.0) == "start"
    assert session.phase == RUNNING
    session.on_status(episode.EpisodeEvent(episode.START, task="t"), 8.5)  # duration counts from the bridge
    session.on_status(END_OK, 20.0)
    assert session.results == [Trial(0, True, 11.5)]
    # The bridge reset itself after the end event: wait to settle, then start the next trial.
    assert session.next_command(27.9) is None
    assert session.next_command(28.0) == "start"


def test_timeout_sends_fail_and_counts_the_end_event():
    session = EvalSession(trials=1, settle_s=1, timeout_s=10)
    session.next_command(0)
    session.next_command(1)
    assert session.next_command(11) == "fail"
    session.on_status(END_FAIL, 11.1)
    assert session.done and session.phase == DONE
    assert session.results[0].success is False
    assert session.next_command(50) is None


def test_summary_file(tmp_path):
    trials = [Trial(0, True, 10.0), Trial(1, False, 30.0), Trial(2, True, 12.5), Trial(3, True, 9.0)]
    assert summarize(trials)["success_rate"] == 0.75
    out = tmp_path / "eval" / "summary.json"
    write_summary(out, trials, {"condition": "A_sim"})
    data = json.loads(out.read_text())
    assert data["condition"] == "A_sim" and data["successes"] == 3 and len(data["results"]) == 4
