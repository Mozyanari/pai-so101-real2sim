import numpy as np

from so101_pai.recon.align import _quat_to_matrix, _quat_wxyz, apply
from so101_pai.recon.transfer import compose, fit_between, similarity


def random_similarity(rng):
    rot = _quat_to_matrix(rng.normal(size=4))
    return float(rng.uniform(0.2, 3.0)), rot, rng.normal(size=3)


def test_similarity_recovers_a_known_transform():
    rng = np.random.default_rng(0)
    s, r, t = random_similarity(rng)
    src = rng.normal(size=(20, 3))
    s2, r2, t2 = similarity(src, s * src @ r.T + t)
    assert np.isclose(s2, s) and np.allclose(r2, r) and np.allclose(t2, t)


def test_compose_equals_mapping_to_old_then_old_alignment():
    rng = np.random.default_rng(1)
    s, r, t = random_similarity(rng)
    old = {"pos": [0.1, 0.2, 0.3], "rot": _quat_wxyz(_quat_to_matrix(rng.normal(size=4))).tolist(), "scale": 0.2}
    points = rng.normal(size=(10, 3))
    np.testing.assert_allclose(apply(compose(old, s, r, t), points), apply(old, s * points @ r.T + t), atol=1e-9)


def test_fit_between_matches_frames_by_prefix():
    rng = np.random.default_rng(2)
    s, r, t = random_similarity(rng)
    joint = {f"old/f{i}.jpg": p for i, p in enumerate(rng.normal(size=(6, 3)))}
    joint["new/f0.jpg"] = np.zeros(3)  # other capture: ignored
    model = {name.split("/")[1]: s * p @ r.T + t for name, p in joint.items() if name.startswith("old/")}
    (s2, _, _), residuals = fit_between(joint, model, "old")
    assert np.isclose(s2, s) and len(residuals) == 6 and residuals.max() < 1e-9
