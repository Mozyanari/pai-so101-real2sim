from so101_pai.common import episode
from so101_pai.dataset.timeline import EpisodeSplitter, Timeline, episode_spans, frame_stamps


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


def test_splitter_holds_one_episode_and_the_last_message_before_it():
    splitter = EpisodeSplitter(["state"])
    for t in range(0, 100, 10):  # idle: only the latest message is kept
        splitter.add("state", t, t)
    assert splitter.streams["state"].values == [90]
    assert splitter.event(100, episode.EpisodeEvent(episode.START, task="t")) is None
    for t in range(110, 200, 10):
        splitter.add("state", t, t)
    span = splitter.event(200, episode.EpisodeEvent(episode.END, success=True))
    assert (span.start_ns, span.end_ns, span.task, span.success) == (100, 200, "t", True)
    timeline = splitter.streams["state"]
    assert timeline.sample(100) == 90 and timeline.sample(195) == 190 and len(timeline.values) == 10
    splitter.add("state", 210, 210)  # back to idle: the finished episode is dropped
    assert timeline.values == [210]


def test_frame_stamps_fixed_rate():
    stamps = frame_stamps(0, 1_000_000_000, 30)
    assert len(stamps) == 31
    assert stamps[1] == 33_333_333


def test_close_ends_an_open_episode_as_failed():
    splitter = EpisodeSplitter(["state"])
    assert splitter.close(5) is None  # nothing open
    splitter.event(100, episode.EpisodeEvent(episode.START, task="t"))
    splitter.add("state", 150, 150)
    span = splitter.close(180)  # e.g. the operator stopped the run with Ctrl-C
    assert (span.start_ns, span.end_ns, span.task, span.success) == (100, 180, "t", False)
    assert splitter.close(190) is None
