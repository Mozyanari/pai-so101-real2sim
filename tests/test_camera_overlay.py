import numpy as np

from so101_pai.robot.camera_overlay import blend


def test_blend_mixes_and_edges_mark_reference_outlines_in_red():
    live = np.zeros((40, 40, 3), np.uint8)
    reference = np.zeros((40, 40, 3), np.uint8)
    reference[10:30, 10:30] = 255
    np.testing.assert_array_equal(blend(live, reference, 0.5, edges=False)[20, 20], [128, 128, 128])
    edged = blend(live, reference, 0.5, edges=True)
    assert (edged[..., 2] == 255).any() and edged[20, 20].tolist() == [0, 0, 0]  # outline only
    assert live.max() == 0  # input untouched
