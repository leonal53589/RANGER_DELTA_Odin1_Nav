"""Fixed structural planes visible from station A on the November 24 site map."""
import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from parking_keyframe import relative_pose, transform

# Straight sections identified from the horizontal PCD slices around A.
# Exclude corners, wall returns, furniture below, and the ceiling.
WALLS = {
    'front': (0, 1, 5.24, 0.115, -3.4, 1.4),
    'left': (1, 0, 4.79, -0.28, 2.45, 4.10),
}
DATUM = -1.125


def plane(points, axis):
    """Fit coordinate axis as a function of the other two coordinates, in metres."""
    if len(points) < 50:
        raise ValueError(f'only {len(points)} points for plane on axis {axis}')
    axes = [i for i in range(3) if i != axis]
    features = np.column_stack([points[:, axes], np.ones(len(points))])
    values = points[:, axis]
    initial = [0., 0., np.median(values)]
    fit = least_squares(lambda c: features @ c - values, initial, loss='soft_l1', f_scale=0.015)
    normal = np.eye(3)[axis]
    normal[axes] = -fit.x[:2]
    norm = np.linalg.norm(normal)
    return normal / norm, fit.x[2] / norm


def floor(points, expected_height):
    ground = points[(abs(points[:, 2] - expected_height) < 0.18)]
    return plane(ground, 2)


def floor_level(cloud, measured_floor, target_floor):
    normal, offset = measured_floor
    up = np.array([0., 0., 1.])
    cross = np.cross(normal, up)
    angle = math.atan2(np.linalg.norm(cross), normal @ up)
    leveling = Rotation.from_rotvec(cross * (angle / max(np.linalg.norm(cross), 1e-12)))
    return leveling.apply(cloud) + [0., 0., target_floor - offset]


def wall_points(points, half_width):
    result = {}
    for name, (axis, along, intercept, slope, low, high) in WALLS.items():
        keep = ((points[:, along] > low) & (points[:, along] < high)
                & (abs(points[:, axis] - intercept - points[:, along] * slope) < half_width)
                & (points[:, 2] > -0.65) & (points[:, 2] < 1.20))
        result[name] = points[keep]
    return result


def reference_planes(groups):
    return {name: plane(points, WALLS[name][0]) for name, points in groups.items()}


def fit_walls(cloud, guess, planes):
    # Freeze correspondences by wall identity. Balance walls by sample count.
    groups = wall_points(transform(cloud, guess), 0.42)
    if any(len(v) < 50 for v in groups.values()):
        return dict(reason='missing_wall_coverage',
                    counts={name: len(points) for name, points in groups.items()})
    groups = {name: transform(points, relative_pose(guess, np.zeros(3)))
              for name, points in groups.items()}

    def residuals(estimate):
        return {name: transform(points, estimate) @ planes[name][0] - planes[name][1]
                for name, points in groups.items()}

    count = min(len(v) for v in groups.values())
    fit = least_squares(lambda q: np.concatenate([r * math.sqrt(count / len(r))
                                                for r in residuals(q).values()]),
                        guess, loss='soft_l1', f_scale=0.020)
    if not fit.success:
        raise ValueError(fit.message)
    singular = np.linalg.svd(fit.jac / [1, 1, 2], compute_uv=False)
    stats = {name: dict(n=len(r), median_cm=float(100 * np.median(r)),
                        p90_abs_cm=float(100 * np.percentile(abs(r), 90)))
             for name, r in residuals(fit.x).items()}
    return dict(pose=fit.x.tolist(), residuals=stats, singular_ratio=float(singular[-1] / singular[0]))



class StationWalls:
    """Estimate base_link relative to the captured A pose using its two observed walls."""
    def __init__(self, points, map_pose):
        self.map_pose = np.asarray(map_pose, dtype=float)
        anchor = self.level(points)
        self.planes = reference_planes(wall_points(transform(anchor, self.map_pose), 0.42))
        self.origin = self.match(anchor, self.map_pose)

    @staticmethod
    def level(points):
        normal, height = floor(points, 0.)
        if normal[2] < math.cos(math.radians(10)) or abs(height) > 0.15:
            raise ValueError('floor outside station envelope')
        return floor_level(points, (normal, height), DATUM)

    def match(self, cloud, guess):
        fit = fit_walls(cloud, guess, self.planes)
        if 'pose' not in fit or not fit['singular_ratio'] > 0.08:
            raise ValueError('insufficient independent wall coverage')
        for name, stats in fit['residuals'].items():
            if (stats['n'] < 150 or abs(stats['median_cm']) > 2.5 or
                    stats['p90_abs_cm'] > {'front': 11., 'left': 6.}[name]):
                raise ValueError('poor fit to ' + name + ' wall')
        return np.asarray(fit['pose'])

    def offset(self, points, guess):
        seed = relative_pose(self.map_pose, guess)
        if np.linalg.norm(seed[:2]) > 0.40 or abs(seed[2]) > math.radians(25):
            raise ValueError('outside station acquisition region')
        solved = self.match(self.level(points), guess)
        delta = relative_pose(guess, solved)
        offset = relative_pose(self.origin, solved)
        if (np.linalg.norm(delta[:2]) > 0.25 or abs(delta[2]) > math.radians(12)
                or np.linalg.norm(offset[:2]) > 0.30 or abs(offset[2]) > math.radians(25)
                or not np.isfinite(offset).all()):
            raise ValueError('station fit outside correction envelope')
        return offset
