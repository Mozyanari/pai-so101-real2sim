from so101_pai.common import episode
from so101_pai.dataset.timeline import Timeline, episode_spans, frame_stamps


def test_zero_order_hold():
    timeline = Timeline()
    timeline.append(100, "a")
    timeline.append(300, "c")
    timeline.append(200, "b")  # out of order insert
    assert timeline.sample(50) is None
    assert timeline.sample(100) == "a"
    assert timeline.sample(250) == "b"
    assert timeline.sample(10_000) == "c"


def test_episode_spans_pairs_and_drops_unclosed():
    start = episode.EpisodeEvent(episode.START, task="t1")
    events = [
        (0, start),
        (10, episode.EpisodeEvent(episode.END, success=True)),
        (20, episode.EpisodeEvent(episode.START, task="dropped")),
        (30, episode.EpisodeEvent(episode.START, task="t2")),
        (40, episode.EpisodeEvent(episode.END, success=False)),
        (50, episode.EpisodeEvent(episode.START, task="unclosed")),
    ]
    spans = episode_spans(events)
    assert [(s.start_ns, s.end_ns, s.task, s.success) for s in spans] == [(0, 10, "t1", True), (30, 40, "t2", False)]


def test_frame_stamps_fixed_rate():
    stamps = frame_stamps(0, 1_000_000_000, 30)
    assert len(stamps) == 31
    assert stamps[1] == 33_333_333
