import numpy as np
import pytest

from so101_pai.policy.chunking import ChunkScheduler


def chunk(start: int, length: int = 10) -> np.ndarray:
    """Chunk whose action i is [start + i] so tests can read which action played."""
    return np.arange(start, start + length, dtype=float)[:, None]


def test_first_chunk_asks_for_inference_and_holds_nothing_until_it_arrives():
    sched = ChunkScheduler(n_action_steps=6, lead_steps=2)
    assert sched.should_infer()
    obs_tick = sched.started()
    assert not sched.should_infer()  # one inference at a time
    assert sched.step() is None and sched.step() is None
    sched.receive(chunk(0), obs_tick)
    # Two ticks passed during inference, so actions 0 and 1 are stale; play 2..5 (n_action_steps = 6).
    assert [sched.step()[0] for _ in range(4)] == [2, 3, 4, 5]


def test_next_inference_starts_lead_steps_early_and_replaces_the_queue():
    sched = ChunkScheduler(n_action_steps=6, lead_steps=2)
    sched.receive(chunk(0), sched.started())
    played = [sched.step()[0] for _ in range(4)]
    assert played == [0, 1, 2, 3] and sched.should_infer()  # 2 left
    obs_tick = sched.started()
    assert sched.step()[0] == 4  # keeps playing during inference
    sched.receive(chunk(100), obs_tick)  # predicted at tick 4, now tick 5: skip its action 0
    assert [sched.step()[0] for _ in range(5)] == [101, 102, 103, 104, 105]


def test_late_chunk_keeps_one_action_and_empty_queue_holds_the_last():
    sched = ChunkScheduler(n_action_steps=3, lead_steps=1)
    obs_tick = sched.started()
    for _ in range(5):
        sched.step()
    sched.receive(chunk(0), obs_tick)  # 5 ticks late, n_action_steps 3
    assert sched.step()[0] == 5
    assert sched.step()[0] == 5  # held
    assert sched.should_infer()


def test_reset_forgets_everything():
    sched = ChunkScheduler(n_action_steps=4, lead_steps=1)
    sched.receive(chunk(0), sched.started())
    sched.step()
    sched.started()
    sched.reset()
    assert sched.should_infer() and sched.step() is None and sched.tick == 1


def test_lead_steps_must_leave_room_in_the_chunk():
    with pytest.raises(ValueError):
        ChunkScheduler(n_action_steps=5, lead_steps=5)
