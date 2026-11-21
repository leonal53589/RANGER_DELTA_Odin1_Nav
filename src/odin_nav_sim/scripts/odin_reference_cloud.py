"""Check ODIN1 map corrections against the PCD exported during mapping."""

from collections import deque
import math

import numpy as np
from scipy.spatial import cKDTree
from sensor_msgs_py.point_cloud2 import read_points


def voxel_sample(points, size):
    _, first = np.unique(np.floor(points / size).astype(np.int32), axis=0, return_index=True)
    return points[first]


class ReferenceCloud:
    def __init__(self, filename):
        with open(filename, 'rb') as stream:
            header = {}
            for line in stream:
                fields = line.decode('ascii').split()
                if not fields or fields[0].startswith('#'):
                    continue
                header[fields[0]] = fields[1:]
                if fields[0] == 'DATA':
                    break
            xyz_pcd = (
                header.get('FIELDS') == ['x', 'y', 'z']
                and header.get('TYPE') == ['F', 'F', 'F']
                and header.get('SIZE') == ['4', '4', '4']
                and header.get('COUNT', ['1', '1', '1']) == ['1', '1', '1']
                and header.get('DATA') == ['binary']
            )
            if not xyz_pcd:
                raise ValueError(f'{filename}: expected ODIN1 binary PCD with float32 xyz fields')
            points = np.fromfile(stream, dtype='<f4').reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) < 100:
            raise ValueError(f'{filename}: too few finite reference points')
        self.tree = cKDTree(voxel_sample(points, 0.028))

    def scan_points(self, message):
        points = read_points(message, field_names=('x', 'y', 'z'))
        xyz = np.column_stack([points[name] for name in ('x', 'y', 'z')])
        visible = (
            np.isfinite(xyz).all(axis=1)
            & (xyz[:, 0] > 0.30) & (xyz[:, 0] < 4.7)
            & (xyz[:, 2] > -0.82) & (xyz[:, 2] < 1.50)
        )
        return voxel_sample(xyz[visible], 0.04)

    def score(self, points, map_from_lidar):
        matrix = np.array([[map_from_lidar.M[i, j] for j in range(3)] for i in range(3)])
        offset = np.array([map_from_lidar.p[i] for i in range(3)])
        distances, _ = self.tree.query(points @ matrix.T + offset)
        return float(np.median(distances)), float(np.mean(distances < 0.05))


def correction_delta(accepted, candidate, odin_odom_from_lidar):
    """Measure the effect of a correction at the robot, rather than at the map origin."""
    prev = accepted * odin_odom_from_lidar
    new = candidate * odin_odom_from_lidar
    yaw = math.remainder(new.M.GetRPY()[2] - prev.M.GetRPY()[2], 2 * math.pi)
    return (new.p - prev.p).Norm(), abs(yaw)


def correction_changed(accepted, candidate, odin_odom_from_lidar):
    distance, yaw = correction_delta(accepted, candidate, odin_odom_from_lidar)
    return distance > 0.025 or yaw > 0.025


class MapCorrectionCheck:
    """Keep local ODIN odometry live while checking significant global corrections."""

    def __init__(self, filename):
        self.reference = ReferenceCloud(filename)
        self.clouds = deque(maxlen=12)
        self.accepted = None
        self.pending = None
        self.confirmations = 0
        self.last_scan = None
        self.last_supported = None

    def needs_scan(self, candidate, odin_odom_from_lidar, stamp):
        if self.accepted is None:
            return True
        if not correction_changed(self.accepted, candidate, odin_odom_from_lidar):
            self.last_supported = stamp
            self.pending = None
            self.confirmations = 0
            return False
        return True

    def observe(self, candidate, odin_odom_from_lidar, cloud, stamp):
        points = self.reference.scan_points(cloud)
        if len(points) < 400:
            return
        proposed = self.reference.score(points, candidate * odin_odom_from_lidar)
        previous = None
        if self.accepted is not None:
            previous = self.reference.score(points, self.accepted * odin_odom_from_lidar)
            if previous[0] <= 0.055 and previous[1] >= 0.50:
                self.last_supported = stamp
        good_match = proposed[0] <= 0.045 and proposed[1] >= 0.55
        if previous is not None and not good_match:
            distance, yaw = correction_delta(self.accepted, candidate, odin_odom_from_lidar)
            # A small correction with clear relative improvement can be supported
            # even when part of the live scene is absent from the saved map.
            good_match = (
                distance <= 0.06 and yaw <= 0.05
                and proposed[0] <= 0.055 and proposed[1] >= 0.50
            )
        improves_match = previous is None or (
            proposed[0] + 0.006 <= previous[0] and proposed[1] >= previous[1] + 0.03
        )
        if not (good_match and improves_match):
            self.pending = None
            self.confirmations = 0
            return
        if self.pending is None or correction_changed(self.pending, candidate, odin_odom_from_lidar):
            self.confirmations = 0
        self.pending = candidate
        self.confirmations += 1
        if self.confirmations >= 3:
            self.accepted = candidate
            self.last_supported = stamp
            self.pending = None
            self.confirmations = 0

    def alignment(self, stamp):
        if self.last_supported is None or (stamp - self.last_supported).nanoseconds > 700_000_000:
            return None
        return self.accepted
