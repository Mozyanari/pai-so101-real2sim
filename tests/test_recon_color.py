import numpy as np

from so101_pai.recon.color import SH_C0, fit_gain_offset, recolor_dc


def test_recolor_dc_maps_base_color_affinely():
    features = np.array([[0.1, -0.5, 1.2], [0.0, 0.0, 0.0]])
    gain, offset = np.array([1.1, 0.9, 1.0]), np.array([-0.2, -0.05, -0.1])
    out = recolor_dc(features, gain, offset)
    np.testing.assert_allclose(SH_C0 * out + 0.5, gain * (SH_C0 * features + 0.5) + offset)


def test_fit_gain_offset_recovers_the_mapping():
    rng = np.random.default_rng(0)
    sim = rng.uniform(0, 255, (50, 3))
    gain, offset = np.array([1.13, 0.91, 1.01]), np.array([-74.0, -16.6, -48.8])
    g, o = fit_gain_offset(sim, sim * gain + offset)
    np.testing.assert_allclose(g, gain)
    np.testing.assert_allclose(o, offset)
