#!/usr/bin/env python3
"""
PCD 3D 点云转 Nav2 2D 占用栅格地图转换工具
(3D PCD Point Cloud to Nav2 2D Occupancy Grid Map Converter)

功能:
1. 解析标准 PCD 点云文件 (支持 ASCII 与 Binary 格式)。
2. 依据高度区间对障碍物进行切片投影：
   - 地面点 (z < floor_z): 投影标记为空闲可通行区域 (Free, 像素值 254)。
   - 障碍物点 (z_min <= z <= z_max): 投影标记为障碍物占用区域 (Occupied, 像素值 0)。
   - 未探测区域: 标记为未知区域 (Unknown, 像素值 205)。
3. 导出符合 ROS 2 Nav2 map_server 标准的 `<name>.pgm` 与 `<name>.yaml`。
"""

import argparse
import os
import sys

import numpy as np


def parse_pcd(file_path):
    with open(file_path, 'rb') as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                break
            line_str = line.decode('ascii', errors='ignore').strip()
            if line_str.startswith('#'):
                continue
            parts = line_str.split()
            if not parts:
                continue
            key = parts[0].upper()
            val = parts[1:]
            header[key] = val
            if key == 'DATA':
                break

        fields = [f.lower() for f in header.get('FIELDS', ['x', 'y', 'z'])]
        sizes = [int(s) for s in header.get('SIZE', [4]*len(fields))]
        types = header.get('TYPE', ['F']*len(fields))
        points_count = int(header.get('POINTS', [0])[0])
        data_format = header.get('DATA', ['ascii'])[0].lower()

        # Find x, y, z indices
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            raise ValueError(f"PCD 缺少必要字段 x, y, z. 当前字段: {fields}")
        ix, iy, iz = fields.index('x'), fields.index('y'), fields.index('z')

        if data_format == 'binary':
            raw_bytes = f.read()
            # If standard 3 floats (x, y, z)
            if len(fields) == 3 and sizes == [4, 4, 4] and types == ['F', 'F', 'F']:
                pts = np.frombuffer(raw_bytes, dtype=np.float32).reshape(-1, 3)
                return pts[:, 0], pts[:, 1], pts[:, 2]
            else:
                # Structured record
                dtype_list = []
                for fname, fsize, ftype in zip(fields, sizes, types):
                    tchar = 'f' if ftype == 'F' else ('i' if ftype == 'I' else 'u')
                    dtype_list.append((fname, f'<{tchar}{fsize}'))
                record_arr = np.frombuffer(raw_bytes, dtype=np.dtype(dtype_list), count=points_count)
                return record_arr['x'].astype(np.float32), record_arr['y'].astype(np.float32), record_arr['z'].astype(np.float32)
        else:
            # ASCII format
            rest = f.read().decode('ascii', errors='ignore')
            lines = rest.strip().splitlines()
            pts = []
            for line in lines:
                parts = line.strip().split()
                if len(parts) >= max(ix, iy, iz) + 1:
                    pts.append((float(parts[ix]), float(parts[iy]), float(parts[iz])))
            arr = np.array(pts, dtype=np.float32)
            return arr[:, 0], arr[:, 1], arr[:, 2]


def pcd_to_grid(xs, ys, zs, resolution=0.05, z_min=-0.7, z_max=0.8, floor_z=-0.85, margin=1.0):
    if len(xs) == 0:
        raise ValueError("PCD 中未检测到有效点云！")

    # Determine bounding box
    x_min_raw, x_max_raw = float(np.min(xs)), float(np.max(xs))
    y_min_raw, y_max_raw = float(np.min(ys)), float(np.max(ys))

    x_min = np.floor((x_min_raw - margin) / resolution) * resolution
    x_max = np.ceil((x_max_raw + margin) / resolution) * resolution
    y_min = np.floor((y_min_raw - margin) / resolution) * resolution
    y_max = np.ceil((y_max_raw + margin) / resolution) * resolution

    width = int(round((x_max - x_min) / resolution))
    height = int(round((y_max - y_min) / resolution))

    print(f"  [地图几何] 范围: X=[{x_min:.2f}, {x_max:.2f}]m, Y=[{y_min:.2f}, {y_max:.2f}]m")
    print(f"  [栅格尺寸] 分辨率={resolution}m/cell, 宽度={width} cells, 高度={height} cells (共 {width*height} cells)")

    # Default trinary: 205 = Unknown, 254 = Free, 0 = Occupied
    # In ROS OccupancyGrid PGM (trinary mode):
    # 0 = Occupied (black)
    # 254 = Free (white)
    # 205 = Unknown (gray)
    grid = np.full((height, width), 205, dtype=np.uint8)

    # 1. Floor points -> mark as Free space (254)
    floor_mask = (zs < floor_z)
    if np.any(floor_mask):
        floor_xs = xs[floor_mask]
        floor_ys = ys[floor_mask]
        col_f = np.clip(((floor_xs - x_min) / resolution).astype(np.int32), 0, width - 1)
        row_f = np.clip(((floor_ys - y_min) / resolution).astype(np.int32), 0, height - 1)
        grid[row_f, col_f] = 254

    # 2. Obstacle points -> mark as Occupied (0)
    obs_mask = (zs >= z_min) & (zs <= z_max)
    obs_count = int(np.sum(obs_mask))
    print(f"  [点云分类] 障碍点数量: {obs_count} (高度在 [{z_min:+.2f}, {z_max:+.2f}]m 内)")
    if obs_count > 0:
        obs_xs = xs[obs_mask]
        obs_ys = ys[obs_mask]
        col_o = np.clip(((obs_xs - x_min) / resolution).astype(np.int32), 0, width - 1)
        row_o = np.clip(((obs_ys - y_min) / resolution).astype(np.int32), 0, height - 1)
        grid[row_o, col_o] = 0

    return grid, x_min, y_min, resolution


def save_nav2_map(grid, x_min, y_min, resolution, output_dir, map_name="map"):
    os.makedirs(output_dir, exist_ok=True)
    pgm_path = os.path.join(output_dir, f"{map_name}.pgm")
    yaml_path = os.path.join(output_dir, f"{map_name}.yaml")

    height, width = grid.shape

    # PGM P5 binary writer
    # ROS map origin is bottom-left (y=y_min), but image row 0 is top.
    # Therefore, we flip vertically so row 0 is y_max.
    flipped = np.flipud(grid)

    with open(pgm_path, 'wb') as f:
        header = f"P5\n{width} {height}\n255\n".encode('ascii')
        f.write(header)
        f.write(flipped.tobytes())

    # Gray 205 has occupancy (255 - 205) / 255, which must stay above free_thresh.
    yaml_content = f"""image: {map_name}.pgm
resolution: {resolution}
origin: [{x_min:.4f}, {y_min:.4f}, 0.0000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.19
mode: trinary
"""
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)

    print(f"\n✔ 成功导出 Nav2 栅格地图:")
    print(f"  - 图像描述: {pgm_path} ({os.path.getsize(pgm_path)/1024:.1f} KB)")
    print(f"  - 坐标配置: {yaml_path}")


def main():
    parser = argparse.ArgumentParser(description="PCD 3D点云转 Nav2 2D 占用栅格地图工具")
    parser.add_argument("--input", "-i", required=True, help="输入的 3D PCD 点云文件路径 (如 maps/1.pcd)")
    parser.add_argument("--output-dir", "-d", default="maps", help="输出地图目录 (默认: maps)")
    parser.add_argument("--name", "-n", default="real_map", help="输出地图基本名称 (默认: real_map)")
    parser.add_argument("--resolution", "-r", type=float, default=0.05, help="栅格地图分辨率 (米/像素, 默认: 0.05)")
    parser.add_argument("--z-min", type=float, default=-0.7, help="障碍物判定最小高度 (米, 默认: -0.7)")
    parser.add_argument("--z-max", type=float, default=0.8, help="障碍物判定最大高度 (米, 默认: 0.8)")
    parser.add_argument("--floor-z", type=float, default=-0.85, help="地面判定高度阈值 (米, 小于此值视为空闲地面, 默认: -0.85)")
    parser.add_argument("--margin", type=float, default=1.0, help="边界留白安全裕度 (米, 默认: 1.0)")
    args = parser.parse_args()

    print("\033[1;36m" + "=" * 70)
    print("  3D PCD 点云转换为 Nav2 2D 占用栅格地图 (PCD to GridMap)")
    print("=" * 70 + "\033[0m")
    print(f"输入文件: {args.input}")

    if not os.path.isfile(args.input):
        print(f"\033[31m错误: 文件不存在: {args.input}\033[0m")
        sys.exit(1)

    xs, ys, zs = parse_pcd(args.input)
    print(f"  读取到 {len(xs)} 个有效点云坐标")

    grid, x_min, y_min, res = pcd_to_grid(
        xs, ys, zs,
        resolution=args.resolution,
        z_min=args.z_min,
        z_max=args.z_max,
        floor_z=args.floor_z,
        margin=args.margin,
    )

    save_nav2_map(grid, x_min, y_min, res, args.output_dir, args.name)
    print("\n地图转换完成。用 Nav2 map_server 加载后检查空闲区是否连续。")


if __name__ == '__main__':
    main()
