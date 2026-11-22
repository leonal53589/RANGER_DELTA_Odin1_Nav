import math
from types import SimpleNamespace
from unittest.mock import Mock

import PyKDL
import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.clock import ClockType
from rclpy.time import Time
from tf2_ros import Buffer, ExtrapolationException

from odin_localization import OdinLocalization
from odin_reference_cloud import MapCorrectionCheck
from odin_spin_guard import ChassisMotion, SpinAlignmentGuard


def time_ms(milliseconds):
    return Time(nanoseconds=milliseconds * 1_000_000, clock_type=ClockType.ROS_TIME)


def transform(parent, child, timestamp, x=0.0):
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.header.stamp = time_ms(timestamp).to_msg()
    msg.transform.translation.x = x
    msg.transform.rotation.w = 1.0
    return msg


def test_newer_wheel_pose_can_use_recent_map_correction():
    odin_tf, robot_tf = Buffer(), Buffer()
    robot_tf.set_transform_static(transform('base_link', 'lidar', 0, x=0.195), 'test')
    clock = Mock()
    node = SimpleNamespace(
        odin_tf=odin_tf, robot_tf=robot_tf, map_from_odin_map=PyKDL.Frame(),
        map_check=None, spin_guard=None, max_pose_age=0.5, last_stamp=None, get_clock=lambda: clock,
        get_logger=Mock(), broadcaster=Mock(),
    )

    for stamp in (800, 900, 1000, 1100):
        odin_tf.set_transform(transform('odin_odom', 'odin_map', stamp), 'test')
        odin_tf.set_transform(transform('odin_odom', 'lidar', stamp, x=1.195), 'test')
        robot_tf.set_transform(transform('odom', 'base_link', stamp, x=0.5), 'test')
        clock.now.return_value = time_ms(stamp + 110)
        OdinLocalization.publish_alignment(node)
        correction = node.broadcaster.sendTransform.call_args.args[0]
        assert correction.header.stamp == time_ms(stamp + 300).to_msg()
        robot_tf.set_transform(correction, 'test')

    # The controller's wheel pose arrives 130 ms after the last Odin sample.
    robot_tf.set_transform(transform('odom', 'base_link', 1230, x=0.52), 'test')
    actual = robot_tf.lookup_transform('map', 'base_link', time_ms(1230))
    assert actual.transform.translation.x == pytest.approx(1.02)

    # Interpolate the same correction over the validity window used by TF2.
    within_window = robot_tf.lookup_transform('map', 'odom', time_ms(1360))
    assert within_window.transform.translation.x == pytest.approx(0.5)

    clock.now.return_value = time_ms(1700)
    OdinLocalization.publish_alignment(node)
    assert node.broadcaster.sendTransform.call_count == 4
    with pytest.raises(ExtrapolationException):
        robot_tf.lookup_transform('map', 'odom', time_ms(1700))


def planar(x=0.0, yaw=0.0):
    return PyKDL.Frame(PyKDL.Rotation.RotZ(yaw), PyKDL.Vector(x, 0.0, 0.0))


def test_spin_guard_holds_position_and_bounds_recovery():
    guard = SpinAlignmentGuard()
    start = planar()
    guard.update(0, start, start, None)
    spin = ChassisMotion(2, 0, 0.0, 0.10)
    wheels = planar(yaw=0.01)
    raw = planar(x=0.06, yaw=0.17)
    output = guard.update(100_000_000, raw, wheels, spin) * wheels
    assert guard.state == 'holding'
    assert output.p.x() == pytest.approx(0)
    assert output.M.GetRPY()[2] == pytest.approx(0.014)

    stop = guard.update(200_000_000, raw, wheels, None) * wheels
    assert stop.p.x() == pytest.approx(0)
    drive = ChassisMotion(0, 0, 0.04, 0.0)
    moved = wheels * planar(x=0.004)
    recovered = guard.update(300_000_000, raw, moved, drive) * moved
    prediction = stop * planar(x=0.004)
    assert guard.state == 'recovering'
    assert (recovered.p - prediction.p).Norm() == pytest.approx(0.0005)
    assert abs(recovered.M.GetRPY()[2] - prediction.M.GetRPY()[2]) <= 0.00121

    # A reset of the wheel coordinate system must not be interpreted as physical motion.
    reset = planar(x=1.0)
    output = guard.update(400_000_000, raw, reset, drive)
    assert guard.state == 'tracking'
    assert output.p.x() == pytest.approx(raw.p.x())


def test_chassis_feedback_rejects_stale_or_future_samples():
    node = SimpleNamespace(
        motion_modes=[(600_000_000, (2, 0)), (1_020_000_000, (2, 0))],
        wheel_velocities=[(950_000_000, (0.0, 0.10))],
    )
    assert OdinLocalization.chassis_motion_at(node, 1_000_000_000) is None
    node.motion_modes.insert(1, (900_000_000, (2, 0)))
    assert OdinLocalization.chassis_motion_at(node, 1_000_000_000) == ChassisMotion(2, 0, 0.0, 0.10)
    assert math.isclose(OdinLocalization.chassis_motion_at(node, 1_100_000_000).yaw_rate, 0.10)


def test_map_jump_requires_point_cloud_support(tmp_path):
    # A sparse, nonperiodic reference lets both poses be scored with real nearest neighbours.
    import numpy as np
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    rng = np.random.default_rng(16)
    points = rng.uniform([0.6, -2., -.5], [3.5, 2., 1.], (1800, 3)).astype('<f4')
    filename = tmp_path / '1.pcd'
    with filename.open('wb') as stream:
        stream.write(b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nDATA binary\n')
        points.tofile(stream)
    check = MapCorrectionCheck(filename)
    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='lidar'), points.tolist())
    local = PyKDL.Frame()
    initial = PyKDL.Frame()
    changed = PyKDL.Frame(PyKDL.Vector(.13, 0., 0.))

    for sample in (1000, 1100, 1200):
        check.observe(initial, local, cloud, time_ms(sample))
    assert check.alignment(time_ms(1200)).p.x() == pytest.approx(0.)

    assert check.needs_scan(changed, local, time_ms(1300))
    for sample in (1300, 1500, 1800, 2100):
        check.observe(changed, local, cloud, time_ms(sample))
        assert check.alignment(time_ms(sample)).p.x() == pytest.approx(0.)
    assert check.alignment(time_ms(2900)) is None

    better = points.copy()
    better[:, 0] -= .13
    better_cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='lidar'), better.tolist())
    for sample in (3000, 3100, 3200):
        check.observe(changed, local, better_cloud, time_ms(sample))
    assert check.alignment(time_ms(3200)).p.x() == pytest.approx(.13)

    # The recorded low-overlap scene supported a small update relative to the
    # accepted pose, while the same evidence should not initialize a new map.
    check.reference.score = lambda points, pose: (
        (0.048, 0.515) if pose.p.x() > 0.15 else (0.061, 0.445)
    )
    small = PyKDL.Frame(PyKDL.Vector(.17, 0., 0.))
    assert check.needs_scan(small, local, time_ms(3300))
    for sample in (3300, 3400, 3500):
        check.observe(small, local, cloud, time_ms(sample))
    assert check.alignment(time_ms(3500)).p.x() == pytest.approx(.17)

    check.accepted = None
    for sample in (3600, 3700, 3800):
        check.observe(small, local, cloud, time_ms(sample))
    assert check.alignment(time_ms(3800)) is None
