import math

import pytest

from ranger_delta_bringup.protocol import (
    CHASSIS_MOTION,
    MOTION_MODE,
    SYSTEM_STATUS,
    OdometryEstimate,
    body_velocity,
    decode_frame,
    replay_frames,
)


def test_delta_status_uses_tenths_of_a_volt():
    assert decode_frame(SYSTEM_STATUS, bytes.fromhex('0000010900000000')) == (0, 0, 26.5, 0)
    assert decode_frame(SYSTEM_STATUS, bytes.fromhex('0201010900010080')) == (2, 1, 26.5, 0x10080)


def test_motion_feedback_decodes_signed_values_and_rejects_truncated_frames():
    motion = bytes.fromhex('FF3801F40000F9DD')
    assert decode_frame(CHASSIS_MOTION, motion) == (-0.2, 0.5, -1.571)
    assert decode_frame(CHASSIS_MOTION, motion[:-1]) is None
    assert decode_frame(MOTION_MODE, bytes.fromhex('010100')) == (1, 1, 0)


def test_replay_preserves_feedback_order_and_timestamp():
    lines = [
        '(10.001000) can0 221#0000000000000000',
        '(10.002000) can0 111#0123456789ABCDEF',
        '(10.002100) can0 00000291#000000',
        '(10.002200) can0 291#0000',
        '(10.003000) can0 291#000000',
    ]
    assert list(replay_frames(lines)) == [
        (10.001, CHASSIS_MOTION, bytes(8)),
        (10.003, MOTION_MODE, bytes(3)),
    ]


def test_body_velocity_follows_motion_mode():
    assert body_velocity(0, (0.3, 0.0, 0.0), 0.378, 0.338) == (0.3, 0.0, 0.0)
    assert body_velocity(1, (0.3, 0.5, math.pi / 2), 0.378, 0.338) == pytest.approx((0.0, 0.3, 0.0))
    assert body_velocity(2, (0.3, 0.5, 0.0), 0.378, 0.338) == (0.0, 0.0, 0.5)
    assert body_velocity(3, (0.3, 0.5, 0.0), 0.378, 0.338) == (0.0, 0.0, 0.0)
    assert body_velocity(4, (0.3, 0.5, 0.0), 0.378, 0.338) is None


def test_odometry_does_not_integrate_across_missing_feedback():
    estimate = OdometryEstimate()
    velocity = (0.3, 0.0, 0.0)
    estimate.update(10.0, velocity, 0.2)
    estimate.update(10.1, velocity, 0.2)
    assert estimate.x == pytest.approx(0.03)

    estimate.update(11.0, velocity, 0.2)
    assert estimate.x == pytest.approx(0.03)

    estimate.update(11.1, (0.0, 0.0, 0.0), 0.2)
    estimate.update(11.2, (0.0, 0.0, 0.0), 0.2)
    assert (estimate.x, estimate.y, estimate.yaw) == pytest.approx((0.06, 0.0, 0.0))
