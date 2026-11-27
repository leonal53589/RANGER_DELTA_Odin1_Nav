#!/usr/bin/env python3
"""Offline 6-DoF LiDAR extrinsic + base_link spin-center solver from calibration bags and tape."""

import argparse
import json
import math
import sqlite3
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from nav_msgs.msg import Odometry
from agx_bringup.msg import SteeringAngles


def voxel_downsample(pts: np.ndarray, size: float) -> np.ndarray:
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def fit_floor_plane(pts: np.ndarray):
    """Fit z = a*x + b*y + c on floor region in lidar frame."""
    mask = (pts[:, 0] > 0.35) & (pts[:, 0] < 4.0) & (np.abs(pts[:, 1]) < 2.2) & (pts[:, 2] < -0.80) & (pts[:, 2] > -1.45)
    gpts = voxel_downsample(pts[mask], 0.02)
    if len(gpts) < 100:
        raise RuntimeError(f"Too few floor points: {len(gpts)}")
    A = np.column_stack([gpts[:, 0], gpts[:, 1], np.ones(len(gpts))])
    fit = least_squares(lambda c: A @ c - gpts[:, 2], [0.0, 0.0, np.median(gpts[:, 2])], loss='soft_l1', f_scale=0.005)
    a, b, c_z = fit.x
    n = np.array([-a, -b, 1.0])
    n /= np.linalg.norm(n)
    res = np.abs((A @ fit.x - gpts[:, 2]) * n[2])
    roll = math.atan2(n[1], n[2])
    pitch = math.atan2(-n[0], math.hypot(n[1], n[2]))
    z_height = -c_z * n[2]
    return roll, pitch, z_height, float(np.median(res)), float(np.percentile(res, 90)), len(gpts)


def level_and_filter_walls(pts: np.ndarray, R_rp: np.ndarray, z_height: float, voxel=0.03):
    """Rotate cloud by R_y(pitch)*R_x(roll), shift +z_height, keep vertical structure points."""
    leveled = pts @ R_rp.T + np.array([0.0, 0.0, z_height])
    # Keep structure above floor (0.18m to 2.1m), exclude very close self-hits (<0.30m) and far >5.5m
    dist_xy = np.hypot(leveled[:, 0], leveled[:, 1])
    mask = (leveled[:, 2] > 0.18) & (leveled[:, 2] < 2.10) & (dist_xy > 0.30) & (dist_xy < 5.50)
    return voxel_downsample(leveled[mask], voxel)


def compute_normals(pts: np.ndarray, k=14):
    tree = cKDTree(pts)
    dists, idx = tree.query(pts, k=min(k, len(pts)))
    nbrs = pts[idx]  # (N, k, 3)
    centered = nbrs - nbrs.mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', centered, centered) / max(1, centered.shape[1] - 1)
    eigvals, eigvecs = np.linalg.eigh(cov)  # ascending eigenvalues
    normals = eigvecs[:, :, 0]
    # Keep only well-defined planar patches (smallest eigval << middle eigval)
    planarity = (eigvals[:, 1] - eigvals[:, 0]) / np.maximum(eigvals[:, 2], 1e-9)
    # Orient normals roughly toward origin for consistency
    flip = np.sum(normals * pts, axis=1) > 0
    normals[flip] *= -1.0
    valid = (dists[:, -1] < 0.25) & (planarity > 0.15)
    return pts[valid], normals[valid]


def compose_se2(a, b):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return np.array([
        a[0] + ca * b[0] - sa * b[1],
        a[1] + sa * b[0] + ca * b[1],
        math.remainder(a[2] + b[2], math.tau),
    ])


def transform_se2(pts: np.ndarray, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    out = pts.copy()
    out[:, 0] = c * pts[:, 0] - s * pts[:, 1] + x
    out[:, 1] = s * pts[:, 0] + c * pts[:, 1] + y
    return out


def rot2d_vecs(vecs: np.ndarray, yaw: float):
    c, s = math.cos(yaw), math.sin(yaw)
    out = vecs.copy()
    out[:, 0] = c * vecs[:, 0] - s * vecs[:, 1]
    out[:, 1] = s * vecs[:, 0] + c * vecs[:, 1]
    return out


def icp_se2_p2plane(src_pts, src_normals, tgt_tree, tgt_pts, tgt_normals, init_pose, radii=(0.35, 0.15, 0.08)):
    pose = np.array(init_pose, dtype=np.float64)
    for r in radii:
        for _ in range(35):
            moved = transform_se2(src_pts, pose)
            moved_n = rot2d_vecs(src_normals, pose[2])
            dists, idx = tgt_tree.query(moved, distance_upper_bound=r)
            valid = dists < r
            if np.count_nonzero(valid) < 80:
                return None, 0.0, 0
            idx_v = idx[valid]
            n = tgt_normals[idx_v]
            # Require normal alignment
            align = np.abs(np.sum(moved_n[valid] * n, axis=1)) > 0.80
            if np.count_nonzero(align) < 60:
                return None, 0.0, 0
            m_pts = moved[valid][align]
            t_pts = tgt_pts[idx_v][align]
            n = n[align]
            res = np.sum((m_pts - t_pts) * n, axis=1)
            # Jacobian wrt [dx, dy, dyaw]
            rx = m_pts[:, 0] - pose[0]
            ry = m_pts[:, 1] - pose[1]
            # d(transform)/dyaw = [-ry, rx, 0]
            j_yaw = -ry * n[:, 0] + rx * n[:, 1]
            J = np.column_stack([n[:, 0], n[:, 1], j_yaw])
            w = np.sqrt(0.02 / np.maximum(np.abs(res), 0.02))
            Jw = J * w[:, None]
            rw = -res * w
            delta, _, rank, _ = np.linalg.lstsq(Jw, rw, rcond=None)
            if rank < 3:
                return None, 0.0, 0
            # Step clamp
            step_xy = np.hypot(delta[0], delta[1])
            if step_xy > 0.15:
                delta[:2] *= 0.15 / step_xy
            if abs(delta[2]) > 0.15:
                delta[2] = math.copysign(0.15, delta[2])
            pose += delta
            if np.hypot(delta[0], delta[1]) < 2e-4 and abs(delta[2]) < 1e-4:
                break
    moved = transform_se2(src_pts, pose)
    d_final, idx_f = tgt_tree.query(moved, distance_upper_bound=0.06)
    vf = d_final < 0.06
    overlap = float(np.mean(vf))
    # Compute 2x2 xy translational Hessian conditioning (min/max eigenvalue ratio)
    cond_xy = 0.0
    if np.count_nonzero(vf) >= 60:
        nf = tgt_normals[idx_f[vf]][:, :2]
        H_xy = nf.T @ nf
        ev = np.linalg.eigvalsh(H_xy)
        cond_xy = float(ev[0] / max(ev[1], 1e-9))
    return pose, overlap, (int(np.count_nonzero(vf)), cond_xy)


def read_bag_frames(bag_dir: Path, R_rp: np.ndarray, z_height: float, step_deg=12.0):
    db = list(bag_dir.glob("*.db3"))[0]
    conn = sqlite3.connect(str(db))
    rows = conn.execute(
        "SELECT t.name, m.timestamp, m.data FROM messages m JOIN topics t ON t.id = m.topic_id "
        "WHERE t.name IN ('/odin1/cloud_raw', '/odom', '/steering_angles') ORDER BY m.timestamp"
    ).fetchall()
    conn.close()

    odom_t, odom_xy, odom_yaw = [], [], []
    steerings = []
    raw_clouds = []
    for topic, ts, data in rows:
        if topic == '/odom':
            msg = deserialize_message(data, Odometry)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            odom_t.append(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
            odom_xy.append((p.x, p.y))
            odom_yaw.append(yaw)
        elif topic == '/steering_angles':
            msg = deserialize_message(data, SteeringAngles)
            steerings.append([msg.steering_01, msg.steering_02, msg.steering_03, msg.steering_04])
        elif topic == '/odin1/cloud_raw':
            msg = deserialize_message(data, PointCloud2)
            raw_clouds.append((msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec, msg))

    odom_t = np.array(odom_t, dtype=np.float64)
    odom_yaw_unw = np.unwrap(np.array(odom_yaw))
    odom_xy = np.array(odom_xy)

    selected = []
    last_yaw = None
    last_xy = None
    for ts, msg in raw_clouds:
        if ts < odom_t[0] or ts > odom_t[-1]:
            continue
        yaw_i = float(np.interp(ts, odom_t, odom_yaw_unw))
        xy_i = np.array([float(np.interp(ts, odom_t, odom_xy[:, 0])), float(np.interp(ts, odom_t, odom_xy[:, 1]))])
        if last_yaw is None or abs(math.degrees(yaw_i - last_yaw)) >= step_deg or np.linalg.norm(xy_i - last_xy) >= 0.25:
            f = read_points(msg, field_names=('x', 'y', 'z'))
            xyz = np.column_stack([f[k] for k in ('x', 'y', 'z')]).astype(np.float32)
            xyz = xyz[np.isfinite(xyz).all(axis=1) & (xyz != 0).any(axis=1)]
            pts = level_and_filter_walls(xyz, R_rp, z_height, voxel=0.03)
            p_n, n_v = compute_normals(pts)
            selected.append((ts, yaw_i, xy_i, p_n, n_v))
            last_yaw = yaw_i
            last_xy = xy_i
    return selected, np.array(steerings) if steerings else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calib-dir', required=True, type=Path)
    args = parser.parse_args()
    cdir = args.calib_dir.resolve()

    # 1. Multi-stop floor calibration (roll, pitch, z)
    stops = ['stop_0', 'stop_1_p90', 'stop_2_m90', 'stop_3_0', 'drive_start', 'drive_end']
    floor_fits = {}
    raw_stops = {}
    print("=== 1. FLOOR PLANE CALIBRATION (roll, pitch, z) ===")
    for name in stops:
        d = np.load(cdir / f"{name}.npz")
        pts = d['merged_xyz']
        raw_stops[name] = d
        r, p, z, med_r, p90_r, n_g = fit_floor_plane(pts)
        floor_fits[name] = (r, p, z)
        print(f"  {name:12s}: roll={math.degrees(r):+.3f}°, pitch={math.degrees(p):+.3f}°, z={z:.4f} m "
              f"(med={med_r*1000:.2f}mm, p90={p90_r*1000:.2f}mm, n={n_g})")

    rolls = [v[0] for v in floor_fits.values()]
    pitches = [v[1] for v in floor_fits.values()]
    zs = [v[2] for v in floor_fits.values()]
    sensor_roll = float(np.median(rolls))
    sensor_pitch = float(np.median(pitches))
    sensor_z = float(np.median(zs))
    print(f"  -> MEDIAN: roll={math.degrees(sensor_roll):+.4f}° (std={math.degrees(np.std(rolls)):.3f}°), "
          f"pitch={math.degrees(sensor_pitch):+.4f}° (std={math.degrees(np.std(pitches)):.3f}°), "
          f"z={sensor_z:.4f} m (std={np.std(zs)*1000:.2f}mm)")

    R_rp = Rotation.from_euler('xyz', [sensor_roll, sensor_pitch, 0.0]).as_matrix()

    # Prepare leveled planar clouds for all stops
    stop_clouds = {}
    for name in stops:
        pts = level_and_filter_walls(raw_stops[name]['merged_xyz'], R_rp, sensor_z, voxel=0.025)
        p_n, n_v = compute_normals(pts)
        stop_clouds[name] = (p_n, n_v, cKDTree(p_n))

    # 2. Spin calibration: build unified leveled room map by chaining spin_0_to_p90, spin_p90_to_m90, spin_m90_to_0
    print("\n=== 2. MULTI-ANGLE SPIN REGISTRATION ===")
    bag_0_p90, _ = read_bag_frames(cdir / "spin_0_to_p90/bag", R_rp, sensor_z, step_deg=10.0)
    bag_p90_m90, _ = read_bag_frames(cdir / "spin_p90_to_m90/bag", R_rp, sensor_z, step_deg=10.0)
    bag_m90_0, _ = read_bag_frames(cdir / "spin_m90_to_0/bag", R_rp, sensor_z, step_deg=10.0)

    # Collect keyframes along full spin starting from stop_0 as world origin [0, 0, 0]
    # Note: we track incremental pose using odom yaw delta as initial guess
    kf_list = [('stop_0', 0, stop_clouds['stop_0'][0], stop_clouds['stop_0'][1], 0.0)]
    prev_odom_yaw = float(raw_stops['stop_0']['odom'][2])

    for seg_id, (seg_name, frames) in enumerate([('0_to_p90', bag_0_p90), ('p90_to_m90', bag_p90_m90), ('m90_to_0', bag_m90_0)]):
        for i, (ts, oyaw, oxy, pts, nrms) in enumerate(frames):
            kf_list.append((f"{seg_name}_{i}", seg_id, pts, nrms, oyaw))
        if seg_id == 0:
            kf_list.append(('stop_1_p90', 0, stop_clouds['stop_1_p90'][0], stop_clouds['stop_1_p90'][1], float(raw_stops['stop_1_p90']['odom'][2])))
        elif seg_id == 1:
            kf_list.append(('stop_2_m90', 1, stop_clouds['stop_2_m90'][0], stop_clouds['stop_2_m90'][1], float(raw_stops['stop_2_m90']['odom'][2])))
        elif seg_id == 2:
            kf_list.append(('stop_3_0', 2, stop_clouds['stop_3_0'][0], stop_clouds['stop_3_0'][1], float(raw_stops['stop_3_0']['odom'][2])))

    # Sequential chaining pass 1 (solve relative pose in previous frame, then compose)
    poses = [np.array([0.0, 0.0, 0.0])]
    for i in range(1, len(kf_list)):
        _, _, pts, nrms, oyaw = kf_list[i]
        _, _, p_pts, p_nrms, p_oyaw = kf_list[i - 1]
        dyaw = math.remainder(oyaw - p_oyaw, math.tau)
        init_rel = np.array([0.0, 0.0, dyaw])
        tgt_tree = cKDTree(p_pts)
        rel_est, ov, _ = icp_se2_p2plane(pts, nrms, tgt_tree, p_pts, p_nrms, init_rel, radii=(0.25, 0.12, 0.06))
        if rel_est is None:
            print(f"  WARN: seq chain fallback at {kf_list[i][0]}")
            rel_est = init_rel
        poses.append(compose_se2(poses[-1], rel_est))

    # Pass 2 & 3: Build global merged static room map from stops + every 2nd frame, re-register all stops & frames
    for it in range(2):
        map_pts_list, map_nrm_list = [], []
        for idx, (label, _, pts, nrms, _) in enumerate(kf_list):
            if label.startswith('stop_') or idx % 2 == 0:
                map_pts_list.append(transform_se2(pts, poses[idx]))
                map_nrm_list.append(rot2d_vecs(nrms, poses[idx][2]))
        merged_pts = np.vstack(map_pts_list)
        merged_nrm = np.vstack(map_nrm_list)
        # Voxel downsample merged map at 2.5cm keeping first point & normal
        keys = np.floor(merged_pts / 0.025).astype(np.int64)
        _, u_idx = np.unique(keys, axis=0, return_index=True)
        merged_pts, merged_nrm = merged_pts[u_idx], merged_nrm[u_idx]
        map_tree = cKDTree(merged_pts)

        new_poses = []
        conds_xy = []
        for idx, (label, _, pts, nrms, _) in enumerate(kf_list):
            p_est, ov, info_c = icp_se2_p2plane(pts, nrms, map_tree, merged_pts, merged_nrm, poses[idx], radii=(0.15, 0.08, 0.05))
            new_poses.append(p_est if p_est is not None else poses[idx])
            conds_xy.append(info_c[1] if isinstance(info_c, tuple) else 0.0)
        # Anchor stop_0 at [0,0,0]
        anchor = new_poses[0].copy()
        c0, s0 = math.cos(-anchor[2]), math.sin(-anchor[2])
        R0 = np.array([[c0, -s0], [s0, c0]])
        for k in range(len(new_poses)):
            dxy = R0 @ (new_poses[k][:2] - anchor[:2])
            dy = math.remainder(new_poses[k][2] - anchor[2], math.tau)
            new_poses[k] = np.array([dxy[0], dxy[1], dy])
        poses = new_poses

    # Extract stop poses and check loop closure stop_0 vs stop_3_0
    stop_poses = {}
    for idx, (label, _, _, _, _) in enumerate(kf_list):
        if label.startswith('stop_'):
            stop_poses[label] = poses[idx]
            print(f"  {label:12s} in L0: x={poses[idx][0]*100:+6.2f} cm, y={poses[idx][1]*100:+6.2f} cm, yaw={math.degrees(poses[idx][2]):+7.2f}°")

    # Solve X_L,k = c_W + R(yaw_k) * r_L across ALL spin frames!
    # Also check with per-segment center vs single center
    A_lin, b_lin = [], []
    for p in poses:
        c, s = math.cos(p[2]), math.sin(p[2])
        A_lin.append([1, 0, c, -s])
        b_lin.append(p[0])
        A_lin.append([0, 1, s, c])
        b_lin.append(p[1])
    A_lin, b_lin = np.array(A_lin), np.array(b_lin)
    sol_single, _, _, _ = np.linalg.lstsq(A_lin, b_lin, rcond=None)
    res_single = (A_lin @ sol_single - b_lin).reshape(-1, 2)
    err_single = np.linalg.norm(res_single, axis=1)
    r_L_single = sol_single[2:4]
    print(f"  -> Single-center spin fit ({len(poses)} frames): c_W=({sol_single[0]*100:+.2f}, {sol_single[1]*100:+.2f}) cm, "
          f"r_L=({r_L_single[0]*100:+.2f}, {r_L_single[1]*100:+.2f}) cm, rms={np.sqrt(np.mean(err_single**2))*1000:.2f}mm, max={np.max(err_single)*1000:.2f}mm")

    # Per-segment center fit filtered by non-degenerate XY Hessian conditioning (cond_xy >= 0.08)
    A_seg, b_seg, used_idx = [], [], []
    for idx, (lbl, seg_id, _, _, _) in enumerate(kf_list):
        if conds_xy[idx] < 0.08:
            print(f"  Skipping degenerate frame {lbl:14s} (yaw={math.degrees(poses[idx][2]):+.1f}°, cond_xy={conds_xy[idx]:.3f})")
            continue
        used_idx.append(idx)
        p = poses[idx]
        c, s = math.cos(p[2]), math.sin(p[2])
        row_x = [0]*6 + [c, -s]
        row_y = [0]*6 + [s, c]
        row_x[2*seg_id] = 1
        row_y[2*seg_id + 1] = 1
        A_seg.append(row_x); b_seg.append(p[0])
        A_seg.append(row_y); b_seg.append(p[1])
    A_seg, b_seg = np.array(A_seg), np.array(b_seg)
    sol_init, _, _, _ = np.linalg.lstsq(A_seg, b_seg, rcond=None)
    fit_rob = least_squares(lambda x: A_seg @ x - b_seg, sol_init, loss='soft_l1', f_scale=0.005)
    sol_seg = fit_rob.x
    res_seg = (A_seg @ sol_seg - b_seg).reshape(-1, 2)
    err_seg = np.linalg.norm(res_seg, axis=1)
    r_L = sol_seg[6:8]
    print(f"  -> Robust per-segment center fit ({len(used_idx)}/{len(kf_list)} frames): "
          f"r_L=({r_L[0]*100:+.2f}, {r_L[1]*100:+.2f}) cm, "
          f"med={np.median(err_seg)*1000:.2f}mm, rms={np.sqrt(np.mean(err_seg**2))*1000:.2f}mm, max={np.max(err_seg)*1000:.2f}mm")

    # 3. Straight Drive registration -> sensor_yaw
    print("\n=== 3. STRAIGHT DRIVE REGISTRATION (sensor_yaw) ===")
    # Register drive_start to merged room map too, and drive_end to drive_start + drive_bag frames!
    ds_pts, ds_nrm, ds_tree = stop_clouds['drive_start']
    de_pts, de_nrm, de_tree = stop_clouds['drive_end']
    bag_drv, steer_drv = read_bag_frames(cdir / "drive_straight/bag", R_rp, sensor_z, step_deg=5.0)
    print(f"  Loaded drive_straight bag: {len(bag_drv)} keyframes, steering samples={len(steer_drv) if steer_drv is not None else 0}")
    if steer_drv is not None:
        print(f"  Mean wheel steering angles during straight drive (deg): {np.degrees(np.mean(steer_drv, axis=0))}")

    # Chain drive_start -> intermediate frames -> drive_end (relative ICP then compose)
    drv_kfs = [(ds_pts, ds_nrm, np.array([0.0, 0.0]))] + [(p, n, xy - raw_stops['drive_start']['odom'][:2]) for _, _, xy, p, n in bag_drv] + [(de_pts, de_nrm, raw_stops['drive_end']['odom'][:2] - raw_stops['drive_start']['odom'][:2])]
    drv_poses = [np.array([0.0, 0.0, 0.0])]
    for i in range(1, len(drv_kfs)):
        pts, nrms, xy = drv_kfs[i]
        p_pts, p_nrms, p_xy = drv_kfs[i - 1]
        d_odom = np.linalg.norm(xy - p_xy)
        init_rel = np.array([d_odom, 0.0, 0.0])
        rel_est, ov, _ = icp_se2_p2plane(pts, nrms, cKDTree(p_pts), p_pts, p_nrms, init_rel, radii=(0.30, 0.15, 0.07))
        if rel_est is None:
            rel_est = init_rel
        drv_poses.append(compose_se2(drv_poses[-1], rel_est))

    # Merge drive_start + intermediate + drive_end into a corridor map and re-register drive_end cleanly
    corr_pts = np.vstack([transform_se2(drv_kfs[i][0], drv_poses[i]) for i in range(len(drv_kfs))])
    corr_nrm = np.vstack([rot2d_vecs(drv_kfs[i][1], drv_poses[i][2]) for i in range(len(drv_kfs))])
    keys = np.floor(corr_pts / 0.025).astype(np.int64)
    _, u_idx = np.unique(keys, axis=0, return_index=True)
    corr_pts, corr_nrm = corr_pts[u_idx], corr_nrm[u_idx]
    corr_tree = cKDTree(corr_pts)

    p_ds, ov_ds, _ = icp_se2_p2plane(ds_pts, ds_nrm, corr_tree, corr_pts, corr_nrm, [0.0, 0.0, 0.0], radii=(0.15, 0.08, 0.04))
    p_de, ov_de, _ = icp_se2_p2plane(de_pts, de_nrm, corr_tree, corr_pts, corr_nrm, drv_poses[-1], radii=(0.15, 0.08, 0.04))

    # Relative pose of drive_end in drive_start frame
    c_s, s_s = math.cos(-p_ds[2]), math.sin(-p_ds[2])
    R_s = np.array([[c_s, -s_s], [s_s, c_s]])
    dxy_L = R_s @ (p_de[:2] - p_ds[:2])
    dyaw_drv = math.remainder(p_de[2] - p_ds[2], math.tau)
    print(f"  drive_start->drive_end in L_ds: dxy=({dxy_L[0]:+.4f}, {dxy_L[1]:+.4f}) m (dist={np.linalg.norm(dxy_L):.4f} m), dyaw_drv={math.degrees(dyaw_drv):+.3f}°")

    # Compute spin center displacement in L_ds using r_L: c_L = p_L - R(yaw)*r_L
    # c_end - c_start = dxy_L - (R(dyaw_drv) - I)*r_L
    R_dy = np.array([[math.cos(dyaw_drv), -math.sin(dyaw_drv)], [math.sin(dyaw_drv), math.cos(dyaw_drv)]])
    dc_L = dxy_L - (R_dy - np.eye(2)) @ r_L
    # For constant curvature arc with small dyaw_drv, chord angle is (theta_start + theta_end)/2 = theta_start + dyaw_drv/2
    chord_angle = math.atan2(dc_L[1], dc_L[0])
    # So true chassis +X direction at start in L_ds frame is chord_angle - dyaw_drv/2
    chassis_x_in_lidar = chord_angle - dyaw_drv / 2.0
    # Wait: base_to_lidar yaw = -chassis_x_in_lidar (R_z(yaw) maps lidar -> base_link)
    sensor_yaw = math.remainder(-chassis_x_in_lidar, math.tau)
    c_y, s_y = math.cos(sensor_yaw), math.sin(sensor_yaw)
    sensor_x, sensor_y = np.array([[c_y, -s_y], [s_y, c_y]]) @ r_L

    print(f"  Spin center straight translation in L_ds: dc_L=({dc_L[0]:+.4f}, {dc_L[1]:+.4f}) m, chord={math.degrees(chord_angle):+.3f}°")
    print(f"  -> CALIBRATED base_to_lidar 6-DoF:")
    print(f"     sensor_x     = {sensor_x:+.4f} m")
    print(f"     sensor_y     = {sensor_y:+.4f} m")
    print(f"     sensor_z     = {sensor_z:+.4f} m")
    print(f"     sensor_roll  = {sensor_roll:+.5f} rad ({math.degrees(sensor_roll):+.3f}°)")
    print(f"     sensor_pitch = {sensor_pitch:+.5f} rad ({math.degrees(sensor_pitch):+.3f}°)")
    print(f"     sensor_yaw   = {sensor_yaw:+.5f} rad ({math.degrees(sensor_yaw):+.3f}°)")

    # 4. Cross-check Tape Spin Center vs LiDAR Spin Center & R/F angle vs true chassis +X
    print("\n=== 4. TAPE vs LIDAR CROSS-CHECK & R/F MARK ALIGNMENT ===")
    tape = json.loads((cdir / "tape.json").read_text())
    t_stops = [tape["stop_0"], tape["stop_1_p90"], tape["stop_2_m90"], tape["stop_3_0"]]
    A_t, b_t = [], []
    for p in t_stops:
        th = math.radians(p["heading_deg"])
        ct, st = math.cos(th), math.sin(th)
        A_t.append([1, 0, ct, -st]); b_t.append(p["mid_cm"][0])
        A_t.append([0, 1, st, ct]); b_t.append(p["mid_cm"][1])
    sol_t, _, _, _ = np.linalg.lstsq(np.array(A_t), np.array(b_t), rcond=None)
    res_t = np.linalg.norm((np.array(A_t) @ sol_t - np.array(b_t)).reshape(-1, 2), axis=1)

    # Register drive_start to merged room map to link drive_start yaw with stop_0/stop_2_m90!
    p_ds_in_room, _, _ = icp_se2_p2plane(ds_pts, ds_nrm, map_tree, merged_pts, merged_nrm, stop_poses['stop_2_m90'], radii=(0.25, 0.12, 0.05))
    # In room L0 frame, true chassis +X at stop_0 had heading = p_ds_in_room[2] + chassis_x_in_lidar + (stop_0 - drive_start)?
    # Better: at any stop s in {stop_0, stop_1_p90, stop_2_m90, stop_3_0},
    # lidar heading in room L0 is stop_poses[s][2], so true chassis +X in room L0 is stop_poses[s][2] + chassis_x_in_lidar.
    # Meanwhile, R->F heading in ground frame is tape[s]['heading_deg'].
    # Notice: (true chassis +X in L0) - (R->F heading in ground) = constant offset between ground frame and L0 + (true_X - RF)!
    # Wait: we don't know ground_X vs room_L0 directly without comparing displacements!
    # We CAN compare ground_X vs room_L0 using the displacements (stop_1 - stop_0) and (stop_2 - stop_0) of any fixed point, OR:
    # Even simpler! Look at the vector of the R/F midpoint displacement between stop_1 and stop_2 in ground vs lidar!
    # In room L0, the spin center is fixed; at stop_0 vs stop_3_0, compare tape vs LiDAR spin center!
    # Also: at drive_start, lidar pose in L0 is p_ds_in_room.
    # Check angle difference between stop_2_m90 and drive_start in L0:
    print(f"  Tape 4-stop spin center: ground=({sol_t[0]:.2f}, {sol_t[1]:.2f}) cm, "
          f"r_RF_mid_in_RF_frame=({sol_t[2]:.2f}, {sol_t[3]:.2f}) cm, max_res={np.max(res_t):.2f} cm")
    # Convert r_L from stop_0 lidar frame into ground frame? Wait:
    # From stop_0, stop_1_p90, stop_2_m90:
    # R->F headings in ground: theta_RF = [-3.27, +84.86, -87.82, -0.55] deg
    # LiDAR headings in L0:    psi_L    = [0.00, +88.0?, -84.5?, ...] deg
    dtheta_rf_1 = math.remainder(tape["stop_1_p90"]["heading_deg"] - tape["stop_0"]["heading_deg"], 360.0)
    dtheta_rf_2 = math.remainder(tape["stop_2_m90"]["heading_deg"] - tape["stop_0"]["heading_deg"], 360.0)
    dtheta_rf_3 = math.remainder(tape["stop_3_0"]["heading_deg"] - tape["stop_0"]["heading_deg"], 360.0)
    dpsi_L_1 = math.degrees(stop_poses["stop_1_p90"][2])
    dpsi_L_2 = math.degrees(stop_poses["stop_2_m90"][2])
    dpsi_L_3 = math.degrees(stop_poses["stop_3_0"][2])
    print(f"  Heading change comparison (Tape vs LiDAR):")
    print(f"    0 -> +90: Tape={dtheta_rf_1:+.2f}°, LiDAR={dpsi_L_1:+.2f}° (diff={dtheta_rf_1-dpsi_L_1:+.2f}°)")
    print(f"    0 -> -90: Tape={dtheta_rf_2:+.2f}°, LiDAR={dpsi_L_2:+.2f}° (diff={dtheta_rf_2-dpsi_L_2:+.2f}°)")
    print(f"    0 ->  0 : Tape={dtheta_rf_3:+.2f}°, LiDAR={dpsi_L_3:+.2f}° (diff={dtheta_rf_3-dpsi_L_3:+.2f}°)")

    # Save result
    out = {
        "base_to_lidar": {
            "sensor_x_m": float(sensor_x),
            "sensor_y_m": float(sensor_y),
            "sensor_z_m": float(sensor_z),
            "sensor_roll_rad": float(sensor_roll),
            "sensor_pitch_rad": float(sensor_pitch),
            "sensor_yaw_rad": float(sensor_yaw),
            "sensor_roll_deg": math.degrees(sensor_roll),
            "sensor_pitch_deg": math.degrees(sensor_pitch),
            "sensor_yaw_deg": math.degrees(sensor_yaw),
        },
        "spin_fit_lidar": {
            "r_L_m": [float(r_L[0]), float(r_L[1])],
            "per_seg_rms_mm": float(np.sqrt(np.mean(err_seg**2))*1000),
            "per_seg_max_mm": float(np.max(err_seg)*1000),
            "single_center_rms_mm": float(np.sqrt(np.mean(err_single**2))*1000),
        },
        "spin_fit_tape": {
            "ground_center_cm": [float(sol_t[0]), float(sol_t[1])],
            "r_RF_mid_cm": [float(sol_t[2]), float(sol_t[3])],
            "max_res_cm": float(np.max(res_t)),
        },
        "straight_drive": {
            "dxy_L_m": [float(dxy_L[0]), float(dxy_L[1])],
            "dyaw_deg": math.degrees(dyaw_drv),
            "dc_L_m": [float(dc_L[0]), float(dc_L[1])],
        },
        "floor_fits": {k: {"roll_deg": math.degrees(v[0]), "pitch_deg": math.degrees(v[1]), "z_m": float(v[2])}
                       for k, v in floor_fits.items()},
    }
    (cdir / "calibration_result.json").write_text(json.dumps(out, indent=2, ensure_ascii=False))
    print(f"\nSaved full calibration result to {cdir / 'calibration_result.json'}")

if __name__ == '__main__':
    main()
