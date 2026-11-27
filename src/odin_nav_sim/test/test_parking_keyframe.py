import numpy as np
import pytest

from parking_check import decide_lateral_correction
from parking_keyframe import ParkingKeyframe, relative_pose, transform


def test_offset_in_station_axes_and_weak_geometry():
    y, z = np.meshgrid(np.linspace(-1, 1, 35), np.linspace(-0.8, 0.8, 25))
    wall = np.column_stack((np.full(y.size, 3.), y.ravel(), z.ravel()))
    assert not ParkingKeyframe(wall).check(wall, np.zeros(3))['accepted']

    side = np.column_stack((y.ravel() + 2, np.full(y.size, 1.), z.ravel()))
    corner = np.vstack([wall, side])
    offset = np.array([0.09, -0.05, -0.075])
    query = transform(corner, relative_pose(offset, [0., 0., 0.]))
    result = ParkingKeyframe(corner).check(query, np.zeros(3))
    assert result['accepted']
    np.testing.assert_allclose(result['station_offset_m_rad'], offset, atol=0.002)


@pytest.mark.parametrize(('ys', 'ages_ms', 'rejected', 'expected'), [
    ([-0.101, -0.100, -0.099], [400, 280, 160], False, 'right_offset_confirmed'),
    ([-0.091, -0.090, -0.089], [400, 280, 160], False, 'right_offset_below_threshold'),
    ([-0.14, -0.10, -0.10], [400, 280, 160], False, 'unstable_lateral_offset'),
    ([-0.10] * 3, [400, 400, 160], False, 'need_three_distinct_frames'),
    ([-0.10] * 3, [2001, 280, 160], False, 'stale_decision_batch'),
    ([-0.10] * 3, [400, 280, -1], False, 'stale_decision_batch'),
    ([-0.10] * 3, [400, 280, 160], True, 'low_overlap'),
])
def test_lateral_decision_uses_only_fresh_consistent_fits(ys, ages_ms, rejected, expected):
    now_ns = 5_000_000_000
    frames = [
        dict(accepted=True, cloud_stamp_ns=now_ns - age * 1_000_000,
             station_offset_m_rad=[0.0, y, 0.2]) for y, age in zip(ys, ages_ms)
    ]
    if rejected:
        frames[1].update(accepted=False, reason='low_overlap')
    result = decide_lateral_correction(frames, now_ns, 1_000_000_000)
    assert result['reason'] == expected
    if expected == 'right_offset_confirmed':
        assert result['body_left_step_m'] == 0.05
    else:
        assert 'body_left_step_m' not in result
