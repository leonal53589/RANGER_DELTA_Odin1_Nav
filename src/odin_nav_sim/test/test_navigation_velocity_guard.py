"""Exercise the real guard callbacks and TF buffer without DDS or a CAN device."""
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import Buffer

import navigation_velocity_guard as guard_module
from navigation_velocity_guard import NavigationVelocityGuard


class Fixture:
    def __init__(self, monkeypatch):
        self.now = 100.
        self.g = Mock(spec=NavigationVelocityGuard)
        g = self.g
        g.cloud_timeout, g.odom_timeout, g.map_timeout, g.command_timeout = .3, .2, .5, .2
        g.radius, g.margin, g.reaction, g.decel = .32, .04, .35, .5
        g.low, g.high, g.map_lead, g.allow_static_map = .12, 1.35, .3, False
        g.tf = Buffer()
        g.cloud_stamp = g.wheel_stamp = 0.
        g.command = g.wheel = g.points = g.resume_time = g.reason = None
        g.received_cloud = g.received_wheel = g.received_command = g.rejected_command_time = -1e6
        g.output, g.state = Mock(), Mock()
        g.blocked_velocity = None
        g.inhibition.side_effect = lambda now, wall: NavigationVelocityGuard.inhibition(g, now, wall)
        g.get_clock().now.side_effect = lambda: Time(seconds=self.now)
        monkeypatch.setattr(guard_module.time, 'monotonic', lambda: self.now)
        self.transform('base_link', 'lidar', 0., static=True)
        self.fill()
        self.tick()
        self.now += .1
        self.fill()
        assert self.tick() > 0.

    def transform(self, parent, child, stamp, static=False):
        t = TransformStamped()
        t.header = Header(frame_id=parent, stamp=Time(seconds=stamp).to_msg())
        t.child_frame_id = child
        t.transform.rotation.w = 1.
        if static:
            t.transform.translation.z = 1.125
            self.g.tf.set_transform_static(t, 'fixture')
        else:
            self.g.tf.set_transform(t, 'fixture')

    def fill(self, omit='', stamp=None, points=None):
        g = self.g
        if omit != 'map':
            self.transform('map', 'odom', self.now + .3)
        if omit != 'wheel_tf':
            self.transform('odom', 'base_link', self.now)
        if omit != 'odom':
            m = Odometry()
            m.header = Header(frame_id='odom', stamp=Time(seconds=self.now).to_msg())
            m.child_frame_id = 'base_link'
            NavigationVelocityGuard.odom(g, m)
        if omit != 'cloud':
            h = Header(frame_id='lidar', stamp=Time(seconds=self.now if stamp is None else stamp).to_msg())
            m = create_cloud_xyz32(h, points if points is not None else [[2., 0., -.5]] * 4)
            NavigationVelocityGuard.scan(g, m)
        if omit != 'command':
            v = Twist()
            v.linear.x = .3
            NavigationVelocityGuard.velocity(g, v)

    def tick(self):
        NavigationVelocityGuard.tick(self.g)
        return self.g.output.publish.call_args.args[0].linear.x


@pytest.mark.parametrize('omit,dt,reason', [
    ('cloud', .31, 'lidar missing or stale'), ('odom', .21, 'wheel odometry missing or stale'),
    ('wheel_tf', .21, 'wheel transform stale'),
    ('map', .51, 'map alignment stale'), ('command', .21, 'waiting for fresh velocity'),
])
def test_dropouts(monkeypatch, omit, dt, reason):
    f = Fixture(monkeypatch)
    f.now += dt
    f.fill(omit)
    assert f.tick() == 0 and f.g.reason == reason


def test_frozen_stamp_and_future_stamp(monkeypatch):
    f = Fixture(monkeypatch)
    f.now += .31
    for stamp in (f.g.cloud_stamp, f.now + 10):
        f.fill(stamp=stamp)
        assert f.tick() == 0 and f.g.reason == 'lidar missing or stale'


def test_stop_and_release_requires_new_command(monkeypatch):
    f = Fixture(monkeypatch)
    f.now += .01
    f.fill(points=[[.45, .01, -.5]] * 3)
    assert f.tick() == 0 and f.g.reason == 'near obstacle'
    f.now += .01
    f.fill('command')
    assert f.tick() == 0 and f.g.reason == 'waiting for fresh velocity'
    f.now += .01
    f.fill()
    assert f.tick() == 0 and f.g.reason is None
    f.now += .025
    f.fill()
    assert 0 < f.tick() < .02


def test_floor_filtered_only_after_mount_transform(monkeypatch):
    f = Fixture(monkeypatch)
    f.now += .01
    f.fill(points=[[.4, 0., -1.125]] * 3)
    assert f.tick() > 0
    f.now += .01
    f.fill(points=[[0., 0., 0.]] * 3)
    assert f.tick() == 0 and f.g.reason == 'lidar missing or stale'


def test_slowing_down_does_not_release_original_obstacle(monkeypatch):
    f = Fixture(monkeypatch)
    points = [[.5, 0., -.5]] * 3
    f.now += .01
    f.fill(points=points)
    assert f.tick() == 0 and f.g.reason == 'near obstacle'
    f.now += .025
    f.fill(points=points)
    v = Twist()
    v.linear.x = .05
    NavigationVelocityGuard.velocity(f.g, v)
    assert f.tick() == 0 and f.g.reason == 'near obstacle'
