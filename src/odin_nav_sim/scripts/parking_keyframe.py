"""Planar registration of parked base-frame point clouds. Distances in m, angles in rad."""
import math
from itertools import combinations

import numpy as np
from scipy.spatial import cKDTree


def rotation(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def relative_pose(start, end):
    delta = np.asarray(end, dtype=float) - start
    delta[:2] = rotation(-start[2]) @ delta[:2]
    delta[2] = math.remainder(delta[2], math.tau)
    return delta


def base_cloud(xyz, mount_rotation, mount_translation):
    xyz = np.asarray(xyz, dtype=float)
    keep = np.isfinite(xyz).all(axis=1) & (xyz[:, 0] > 0.75)
    cloud = xyz[keep & (np.linalg.norm(xyz, axis=1) < 7.0)]
    _, indices = np.unique(np.floor(cloud / 0.05), axis=0, return_index=True)
    if len(indices) < 30:
        raise ValueError('too_few_voxels')
    return cloud[indices] @ mount_rotation.T + mount_translation


def transform(cloud, pose):
    moved = cloud.copy()
    moved[:, :2] = cloud[:, :2] @ rotation(pose[2]).T + pose[:2]
    return moved


class ParkingKeyframe:
    def __init__(self, reference):
        if (reference.ndim != 2 or reference.shape[1] != 3 or len(reference) < 30
                or not np.isfinite(reference).all()):
            raise ValueError('invalid_station_cloud')
        self.reference = reference
        self.tree = cKDTree(reference)
        _, idx = self.tree.query(reference, k=16)
        neighbors = reference[idx]
        centered = neighbors - neighbors.mean(axis=1, keepdims=True)
        eigenvalues, vectors = np.linalg.eigh(centered.transpose(0, 2, 1) @ centered)
        self.normals = vectors[:, :, 0]
        self.planar = (eigenvalues[:, 0] < 0.20 * eigenvalues[:, 1]) & (abs(self.normals[:, 2]) < 0.80)

    def fit(self, source, seed, *, whole_map=False):
        pose = seed.copy()
        for _ in range(70 if whole_map else 35):
            moved = transform(source, pose)
            distances, idx = self.tree.query(moved, distance_upper_bound=0.30)
            # cKDTree returns index len(reference) for a missing neighbor.
            idx = np.minimum(idx, len(self.reference)-1)
            valid = self.planar[idx] & (distances < 0.30)
            if valid.sum() < 30:
                return None
            n = self.normals[idx[valid]]
            residual = ((moved[valid] - self.reference[idx[valid]]) * n).sum(axis=1)
            arm = moved[valid, :2] - pose[:2]
            jac = np.column_stack((n[:, :2], arm[:, 0] * n[:, 1] - arm[:, 1] * n[:, 0]))
            weight = np.sqrt(0.03 / np.maximum(abs(residual), 0.03))
            # Express yaw as tangential displacement at a 2 m reference distance.
            jac[:, 2] /= 2
            step, _, rank, singular = np.linalg.lstsq(jac * weight[:, None], -residual * weight, rcond=None)
            if rank < 3:
                return None
            step[2] /= 2
            pose += step
            translation_tol, angle_tol = (0.0005, 0.00025) if whole_map else (0.0002, 0.0001)
            if np.linalg.norm(step[:2]) < translation_tol and abs(step[2]) < angle_tol:
                moved = transform(source, pose)
                forward, _ = self.tree.query(moved, distance_upper_bound=0.08)
                overlap = float(np.mean(forward < 0.08))
                overlap_name = 'source_overlap_8cm'
                if not whole_map:
                    reverse, _ = cKDTree(moved).query(self.reference)
                    overlap = min(overlap, np.mean(reverse < 0.08))
                    overlap_name = 'bilateral_overlap_8cm'
                weighted = jac * weight[:, None]
                _, axes = np.linalg.eigh(weighted.T @ weighted)
                return dict(pose=pose.tolist(), **{overlap_name: overlap},
                    singular_ratio=singular[-1] / singular[0],
                    weakest_axis_xy_2m_yaw=axes[:, 0].tolist())
        return None

    def check(self, source, guess, *, whole_map=False):
        fits = []
        for offset in (-6, 0, 6):
            fit = self.fit(source, guess + [0, 0, np.deg2rad(offset)], whole_map=whole_map)
            if fit is not None:
                fits.append(dict(seed_deg=offset, **fit))
        report = dict(accepted=False, fits=fits)
        if len(fits) != 3:
            return dict(report, reason='insufficient_converged_fits')
        overlap = 'source_overlap_8cm' if whole_map else 'bilateral_overlap_8cm'
        if min(f[overlap] for f in fits) < 0.65:
            return dict(report, reason='low_overlap')
        if min(f['singular_ratio'] for f in fits) < 0.03:
            return dict(report, reason='weak_geometry')
        deltas = [relative_pose(a['pose'], b['pose']) for a, b in combinations(fits, 2)]
        report['seed_spread_m'] = max(np.linalg.norm(d[:2]) for d in deltas)
        report['seed_spread_rad'] = max(abs(d[2]) for d in deltas)
        if report['seed_spread_m'] > 0.025 or report['seed_spread_rad'] > math.radians(2):
            return dict(report, reason='ambiguous_fit')
        pose = fits[1]['pose']
        innovation = relative_pose(guess, pose)
        if np.linalg.norm(innovation[:2]) > 0.35 or abs(innovation[2]) > math.radians(20):
            return dict(report, reason='large_innovation')
        return dict(report, accepted=True, reason='ok', station_offset_m_rad=pose)
