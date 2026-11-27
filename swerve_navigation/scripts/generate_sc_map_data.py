#!/usr/bin/env python3
"""Generate simulated mapping output (patches + poses + SC database) from a PCD map.

This script takes an ASCII PCD map (like the one from mock_navigation) and
produces the directory structure that icp_registration expects for SC+NDT+ICP
relocalization:

    <out_dir>/
        map.pcd          # copy of the input PCD
        poses.txt        # keyframe poses
        patches/         # per-keyframe local point clouds (binary PCD)
        sc_data/         # ScanContext database
            metadata.txt
            polarcontexts.bin
            invkeys.bin
            vkeys.bin
            invkeys_mat.bin

Usage:
    python3 generate_sc_map_data.py --pcd /tmp/swerve_simple_room_map.pcd --out /tmp/swerve_sc_test
"""

import argparse
import math
import os
import shutil
import struct
import sys
from typing import List, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# ScanContext parameters (must match C++ Scancontext.h)
# ---------------------------------------------------------------------------
PC_NUM_RING = 20
PC_NUM_SECTOR = 60
PC_MAX_RADIUS = 80.0
LIDAR_HEIGHT = 2.0
PC_UNIT_SECTORANGLE = 360.0 / PC_NUM_SECTOR  # 6.0 deg
PC_UNIT_RINGGAP = PC_MAX_RADIUS / PC_NUM_RING  # 4.0 m

# Sensor simulation parameters
SENSOR_RANGE = 8.0  # metres – match simple_world_sim defaults


# ---------------------------------------------------------------------------
# PCD I/O
# ---------------------------------------------------------------------------
def load_ascii_pcd(path: str) -> np.ndarray:
    """Load an ASCII PCD file, return Nx4 float64 array (x, y, z, intensity)."""
    points: List[Tuple[float, float, float, float]] = []
    in_data = False
    with open(path, 'r', encoding='utf-8') as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            if in_data:
                vals = line.split()
                if len(vals) < 3:
                    continue
                intensity = float(vals[3]) if len(vals) > 3 else 100.0
                points.append((float(vals[0]), float(vals[1]),
                               float(vals[2]), intensity))
                continue
            if line.upper().startswith('DATA'):
                if 'ascii' not in line.lower():
                    raise RuntimeError(f'Only ASCII PCD supported: {path}')
                in_data = True
    if not points:
        raise RuntimeError(f'No points loaded from {path}')
    return np.array(points, dtype=np.float64)


def write_binary_pcd(path: str, pts: np.ndarray) -> None:
    """Write a PCL-compatible binary PCD (XYZI float32)."""
    n = pts.shape[0]
    data = pts[:, :4].astype(np.float32)
    with open(path, 'wb') as f:
        header = (
            '# .PCD v0.7 - Point Cloud Data file format\n'
            'VERSION 0.7\n'
            'FIELDS x y z intensity\n'
            'SIZE 4 4 4 4\n'
            'TYPE F F F F\n'
            'COUNT 1 1 1 1\n'
            f'WIDTH {n}\n'
            'HEIGHT 1\n'
            'VIEWPOINT 0 0 0 1 0 0 0\n'
            f'POINTS {n}\n'
            'DATA binary\n'
        )
        f.write(header.encode('ascii'))
        f.write(data.tobytes())


# ---------------------------------------------------------------------------
# ScanContext computation (numpy replica of C++ makeScancontext)
# ---------------------------------------------------------------------------
def xy2theta(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Replicate the C++ xy2theta: returns azimuth in [0, 360) degrees."""
    theta = np.zeros_like(x)
    # Quadrant I: x >= 0, y >= 0
    q1 = (x >= 0) & (y >= 0)
    theta[q1] = np.degrees(np.arctan2(y[q1], x[q1]))
    # Quadrant II: x < 0, y >= 0
    q2 = (x < 0) & (y >= 0)
    theta[q2] = 180.0 - np.degrees(np.arctan2(y[q2], -x[q2]))
    # Quadrant III: x < 0, y < 0
    q3 = (x < 0) & (y < 0)
    theta[q3] = 180.0 + np.degrees(np.arctan2(y[q3], x[q3]))
    # Quadrant IV: x >= 0, y < 0
    q4 = (x >= 0) & (y < 0)
    theta[q4] = 360.0 - np.degrees(np.arctan2(-y[q4], x[q4]))
    return theta


def make_scancontext(pts_xyz: np.ndarray) -> np.ndarray:
    """Compute a ScanContext descriptor (PC_NUM_RING x PC_NUM_SECTOR).

    pts_xyz: Nx3 array in the *sensor local* frame.
    Returns a (20, 60) float64 matrix, same layout as C++ Eigen column-major.
    """
    desc = np.full((PC_NUM_RING, PC_NUM_SECTOR), -1000.0, dtype=np.float64)

    x = pts_xyz[:, 0]
    y = pts_xyz[:, 1]
    z = pts_xyz[:, 2] + LIDAR_HEIGHT  # offset like C++

    azim_range = np.sqrt(x * x + y * y)
    azim_angle = xy2theta(x, y)

    # Filter points outside max radius
    mask = azim_range <= PC_MAX_RADIUS
    azim_range = azim_range[mask]
    azim_angle = azim_angle[mask]
    z = z[mask]

    # Bin indices (1-based, then convert to 0-based)
    ring_idx = np.clip(np.ceil(azim_range / PC_MAX_RADIUS * PC_NUM_RING).astype(int),
                       1, PC_NUM_RING) - 1
    sector_idx = np.clip(np.ceil(azim_angle / 360.0 * PC_NUM_SECTOR).astype(int),
                         1, PC_NUM_SECTOR) - 1

    # Fill with max z per bin
    for i in range(len(z)):
        ri, si, zi = ring_idx[i], sector_idx[i], z[i]
        if zi > desc[ri, si]:
            desc[ri, si] = zi

    # Replace sentinel with 0
    desc[desc <= -999.0] = 0.0
    return desc


def make_ringkey(desc: np.ndarray) -> np.ndarray:
    """Row-wise mean → (PC_NUM_RING, 1) float64."""
    return desc.mean(axis=1, keepdims=True)  # (20, 1)


def make_sectorkey(desc: np.ndarray) -> np.ndarray:
    """Column-wise mean → (1, PC_NUM_SECTOR) float64."""
    return desc.mean(axis=0, keepdims=True)  # (1, 60)


# ---------------------------------------------------------------------------
# SC database serialization (match C++ exactly)
# ---------------------------------------------------------------------------
def save_sc_database(sc_dir: str,
                     polarcontexts: List[np.ndarray],
                     invkeys: List[np.ndarray],
                     vkeys: List[np.ndarray],
                     invkeys_mat: List[np.ndarray]) -> None:
    """Save ScanContext database in the C++ binary format."""
    os.makedirs(sc_dir, exist_ok=True)
    n = len(polarcontexts)
    rows, cols = PC_NUM_RING, PC_NUM_SECTOR

    # metadata.txt: "N rows cols"
    with open(os.path.join(sc_dir, 'metadata.txt'), 'w') as f:
        f.write(f'{n} {rows} {cols}\n')

    # polarcontexts.bin: N consecutive (rows x cols) double matrices, column-major
    with open(os.path.join(sc_dir, 'polarcontexts.bin'), 'wb') as f:
        for mat in polarcontexts:
            # Eigen stores column-major: flatten in Fortran order
            f.write(mat.astype(np.float64).flatten(order='F').tobytes())

    # invkeys.bin: N consecutive (rows,) double vectors
    with open(os.path.join(sc_dir, 'invkeys.bin'), 'wb') as f:
        for vec in invkeys:
            # (20,1) Eigen matrix, .data() reads column-major = just the 20 doubles
            f.write(vec.flatten().astype(np.float64).tobytes())

    # vkeys.bin: N consecutive (cols,) double vectors
    with open(os.path.join(sc_dir, 'vkeys.bin'), 'wb') as f:
        for vec in vkeys:
            # (1,60) Eigen matrix, .data() column-major = just the 60 doubles
            f.write(vec.flatten().astype(np.float64).tobytes())

    # invkeys_mat.bin: N consecutive (rows,) float32 vectors
    with open(os.path.join(sc_dir, 'invkeys_mat.bin'), 'wb') as f:
        for vec in invkeys_mat:
            f.write(vec.flatten().astype(np.float32).tobytes())


# ---------------------------------------------------------------------------
# Trajectory generation
# ---------------------------------------------------------------------------
def generate_zigzag_trajectory(
    x_min: float, x_max: float,
    y_min: float, y_max: float,
    spacing: float = 0.8,
    y_lanes: int = 5,
    margin: float = 0.5,
) -> List[Tuple[float, float, float]]:
    """Generate a Z-shaped trajectory covering the rectangular area.

    Returns list of (x, y, z=0) keyframe positions.
    """
    # Inset from walls
    x_lo = x_min + margin
    x_hi = x_max - margin
    y_lo = y_min + margin
    y_hi = y_max - margin

    ys = np.linspace(y_lo, y_hi, y_lanes)
    waypoints: List[Tuple[float, float, float]] = []

    for i, y_val in enumerate(ys):
        if i % 2 == 0:
            xs = np.arange(x_lo, x_hi + spacing * 0.5, spacing)
        else:
            xs = np.arange(x_hi, x_lo - spacing * 0.5, -spacing)
        for x_val in xs:
            waypoints.append((float(x_val), float(y_val), 0.0))

    return waypoints


# ---------------------------------------------------------------------------
# Local patch extraction
# ---------------------------------------------------------------------------
def extract_local_patch(
    global_pts: np.ndarray,
    cx: float, cy: float, cz: float,
    radius: float = SENSOR_RANGE,
) -> np.ndarray:
    """Extract points within `radius` of (cx, cy, cz), return in local frame."""
    dx = global_pts[:, 0] - cx
    dy = global_pts[:, 1] - cy
    dist = np.sqrt(dx * dx + dy * dy)
    mask = dist <= radius
    local = global_pts[mask].copy()
    # Transform to local frame (translation only, no rotation for simplicity)
    local[:, 0] -= cx
    local[:, 1] -= cy
    local[:, 2] -= cz
    return local


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='Generate simulated SC+NDT+ICP map data from an ASCII PCD')
    parser.add_argument('--pcd', required=True,
                        help='Path to input ASCII PCD map file')
    parser.add_argument('--out', required=True,
                        help='Output directory (will be created)')
    parser.add_argument('--spacing', type=float, default=0.8,
                        help='Keyframe spacing in metres (default: 0.8)')
    parser.add_argument('--sensor-range', type=float, default=SENSOR_RANGE,
                        help=f'Simulated sensor range (default: {SENSOR_RANGE})')
    args = parser.parse_args()

    # Load map
    print(f'Loading PCD from {args.pcd} ...')
    global_pts = load_ascii_pcd(args.pcd)
    print(f'  Loaded {global_pts.shape[0]} points')

    # Determine map extent
    x_min, y_min = global_pts[:, 0].min(), global_pts[:, 1].min()
    x_max, y_max = global_pts[:, 0].max(), global_pts[:, 1].max()
    print(f'  Map extent: x=[{x_min:.1f}, {x_max:.1f}] y=[{y_min:.1f}, {y_max:.1f}]')

    # Generate trajectory
    trajectory = generate_zigzag_trajectory(
        x_min, x_max, y_min, y_max, spacing=args.spacing)
    print(f'  Generated {len(trajectory)} keyframe poses')

    # Prepare output directories
    os.makedirs(args.out, exist_ok=True)
    patches_dir = os.path.join(args.out, 'patches')
    os.makedirs(patches_dir, exist_ok=True)
    sc_dir = os.path.join(args.out, 'sc_data')

    # Copy map
    dst_map = os.path.join(args.out, 'map.pcd')
    shutil.copy2(args.pcd, dst_map)
    print(f'  Copied map to {dst_map}')

    # Process each keyframe
    polarcontexts: List[np.ndarray] = []
    invkeys_list: List[np.ndarray] = []
    vkeys_list: List[np.ndarray] = []
    invkeys_mat_list: List[np.ndarray] = []
    pose_lines: List[str] = []

    for idx, (kx, ky, kz) in enumerate(trajectory):
        patch_name = f'{idx}.pcd'

        # Extract local patch
        local_pts = extract_local_patch(global_pts, kx, ky, kz,
                                        radius=args.sensor_range)
        if local_pts.shape[0] < 10:
            print(f'  [WARN] Keyframe {idx} at ({kx:.1f},{ky:.1f}) has only '
                  f'{local_pts.shape[0]} points, skipping')
            continue

        # Save binary PCD patch
        write_binary_pcd(os.path.join(patches_dir, patch_name), local_pts)

        # Compute ScanContext descriptor from local XYZ (no intensity needed)
        sc = make_scancontext(local_pts[:, :3])
        rk = make_ringkey(sc)      # (20, 1)
        sk = make_sectorkey(sc)    # (1, 60)
        rk_f32 = rk.flatten().astype(np.float32)  # (20,)

        polarcontexts.append(sc)
        invkeys_list.append(rk)
        vkeys_list.append(sk)
        invkeys_mat_list.append(rk_f32)

        # Pose: identity rotation (qw=1, qx=qy=qz=0)
        pose_lines.append(
            f'{patch_name} {kx:.6f} {ky:.6f} {kz:.6f} 1.000000 0.000000 0.000000 0.000000')

    # Write poses.txt
    poses_path = os.path.join(args.out, 'poses.txt')
    with open(poses_path, 'w') as f:
        for line in pose_lines:
            f.write(line + '\n')
    print(f'  Wrote {len(pose_lines)} poses to {poses_path}')

    # Write SC database
    save_sc_database(sc_dir, polarcontexts, invkeys_list,
                     vkeys_list, invkeys_mat_list)
    print(f'  Wrote SC database to {sc_dir}/ ({len(polarcontexts)} keyframes)')

    # Summary
    print()
    print('=== Done ===')
    print(f'Output directory: {args.out}')
    print(f'  map.pcd          ({os.path.getsize(dst_map)} bytes)')
    print(f'  poses.txt        ({len(pose_lines)} keyframes)')
    print(f'  patches/         ({len(pose_lines)} PCD files)')
    print(f'  sc_data/         ({len(polarcontexts)} descriptors)')
    print()
    print('To test SC relocalization:')
    print(f'  ros2 launch swerve_navigation mock_navigation.launch.py '
          f'pcd_path:={os.path.abspath(dst_map)}')


if __name__ == '__main__':
    main()
