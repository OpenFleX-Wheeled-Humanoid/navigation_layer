#!/usr/bin/env python3
"""
从 ROS 2 bag 提取标定数据 (图片 + 累积点云 PCD)

用法:
  python3 extract_calib_data.py --bag ~/calib_data/bags/scene0 \
      --out ~/calib_data --index 0

  对每个场景依次运行，index 设为 0, 1, 2, ...
  输出:
    ~/calib_data/image/0.png
    ~/calib_data/pcd/0.pcd

依赖:
  pip install rosbags opencv-python numpy open3d
"""

import argparse
import os
import numpy as np

try:
    import cv2
except ImportError:
    raise ImportError("需要 opencv-python: pip install opencv-python")

try:
    import open3d as o3d
except ImportError:
    raise ImportError("需要 open3d: pip install open3d")

from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores


def extract_image(reader, typestore, image_topic, output_path):
    """从 bag 中提取中间时刻的一帧图片保存为 PNG."""
    connections = [c for c in reader.connections if c.topic == image_topic]
    if not connections:
        raise RuntimeError(f"bag 中未找到 topic: {image_topic}")

    # 收集所有图片消息的时间戳
    timestamps = []
    for conn, timestamp, rawdata in reader.messages(connections=connections):
        timestamps.append(timestamp)

    if not timestamps:
        raise RuntimeError(f"topic {image_topic} 中无消息")

    # 选中间时刻
    mid_idx = len(timestamps) // 2
    target_ts = timestamps[mid_idx]

    # 再次遍历，提取目标帧
    for conn, timestamp, rawdata in reader.messages(connections=connections):
        if timestamp == target_ts:
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            if 'compressed' in conn.msgtype.lower():
                img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            else:
                # sensor_msgs/msg/Image
                encoding = msg.encoding
                h, w = msg.height, msg.width
                data = np.frombuffer(msg.data, dtype=np.uint8)
                if encoding in ('rgb8',):
                    img = data.reshape(h, w, 3)
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                elif encoding in ('bgr8',):
                    img = data.reshape(h, w, 3)
                elif encoding in ('mono8',):
                    img = data.reshape(h, w)
                elif encoding in ('bgra8',):
                    img = data.reshape(h, w, 4)
                    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                elif encoding in ('rgba8',):
                    img = data.reshape(h, w, 4)
                    img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                elif encoding in ('16UC1', 'mono16'):
                    img = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
                    img = (img / 256).astype(np.uint8)
                else:
                    img = data.reshape(h, w, -1)

            cv2.imwrite(output_path, img)
            print(f"  图片已保存: {output_path}  (时间戳索引: {mid_idx}/{len(timestamps)})")
            return target_ts

    raise RuntimeError("未能提取图片")


def extract_pointcloud(reader, typestore, lidar_topic, output_path, target_ts, accumulate_sec=10.0):
    """从 bag 中累积 target_ts 前后 accumulate_sec/2 秒的点云，保存为 PCD."""
    connections = [c for c in reader.connections if c.topic == lidar_topic]
    if not connections:
        raise RuntimeError(f"bag 中未找到 topic: {lidar_topic}")

    half_window_ns = int(accumulate_sec / 2.0 * 1e9)
    all_points = []

    for conn, timestamp, rawdata in reader.messages(connections=connections):
        if abs(timestamp - target_ts) > half_window_ns:
            continue
        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)

        # 解析 PointCloud2
        point_step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.uint8)
        num_points = len(data) // point_step
        if num_points == 0:
            continue

        # 查找 xyz 字段偏移
        field_offsets = {}
        for field in msg.fields:
            field_offsets[field.name] = (field.offset, field.datatype)

        # PointCloud2 datatype: 7 = FLOAT32
        if 'x' not in field_offsets:
            continue

        points_raw = data[:num_points * point_step].reshape(num_points, point_step)
        x_off = field_offsets['x'][0]
        y_off = field_offsets['y'][0]
        z_off = field_offsets['z'][0]

        x = np.frombuffer(points_raw[:, x_off:x_off + 4].tobytes(), dtype=np.float32)
        y = np.frombuffer(points_raw[:, y_off:y_off + 4].tobytes(), dtype=np.float32)
        z = np.frombuffer(points_raw[:, z_off:z_off + 4].tobytes(), dtype=np.float32)

        pts = np.stack([x, y, z], axis=1)
        # 过滤无效点
        valid = np.isfinite(pts).all(axis=1) & (np.linalg.norm(pts, axis=1) > 0.1)
        all_points.append(pts[valid])

    if not all_points:
        raise RuntimeError(f"在时间窗口内未找到有效点云 (topic: {lidar_topic})")

    all_points = np.vstack(all_points)
    print(f"  累积点云: {len(all_points)} 点 (±{accumulate_sec/2:.1f}s 窗口)")

    # 体素下采样去重
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_points.astype(np.float64))
    pcd_down = pcd.voxel_down_sample(voxel_size=0.01)
    print(f"  下采样后: {len(pcd_down.points)} 点")

    o3d.io.write_point_cloud(output_path, pcd_down)
    print(f"  PCD 已保存: {output_path}")


def extract_camera_info(reader, typestore, info_topic):
    """提取相机内参."""
    connections = [c for c in reader.connections if c.topic == info_topic]
    if not connections:
        return None

    for conn, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
        K = np.array(msg.k).reshape(3, 3)
        D = np.array(msg.d)
        print(f"\n  相机内参 K:\n    fx={K[0,0]:.2f}, fy={K[1,1]:.2f}, cx={K[0,2]:.2f}, cy={K[1,2]:.2f}")
        print(f"  畸变系数 D: {D.tolist()}")
        print(f"  分辨率: {msg.width}x{msg.height}")
        return K, D, msg.width, msg.height

    return None


def main():
    parser = argparse.ArgumentParser(description='从 ROS 2 bag 提取标定数据')
    parser.add_argument('--bag', required=True, help='ROS 2 bag 路径 (目录)')
    parser.add_argument('--out', required=True, help='输出根目录')
    parser.add_argument('--index', type=int, required=True, help='场景编号 (0, 1, 2, ...)')
    parser.add_argument('--image-topic', default='/camera/color/image_raw', help='图像 topic')
    parser.add_argument('--lidar-topic', default='/livox/lidar', help='点云 topic')
    parser.add_argument('--info-topic', default='/camera/color/camera_info', help='相机信息 topic')
    parser.add_argument('--accumulate', type=float, default=10.0, help='点云累积时长 (秒)')
    args = parser.parse_args()

    # 创建输出目录
    image_dir = os.path.join(args.out, 'image')
    pcd_dir = os.path.join(args.out, 'pcd')
    os.makedirs(image_dir, exist_ok=True)
    os.makedirs(pcd_dir, exist_ok=True)

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with Reader(args.bag) as reader:
        print(f"\n=== 场景 {args.index}: {args.bag} ===")
        print(f"  topics: {[c.topic for c in reader.connections]}")

        # 1. 提取图片
        image_path = os.path.join(image_dir, f'{args.index}.png')
        target_ts = extract_image(reader, typestore, args.image_topic, image_path)

        # 2. 累积点云
        pcd_path = os.path.join(pcd_dir, f'{args.index}.pcd')
        extract_pointcloud(reader, typestore, args.lidar_topic, pcd_path, target_ts, args.accumulate)

        # 3. 提取相机内参 (仅第一个场景打印)
        if args.index == 0:
            result = extract_camera_info(reader, typestore, args.info_topic)
            if result:
                K, D, w, h = result
                print(f"\n  === 用于 livox_camera_calib 的配置 ===")
                print(f"  camera_matrix: [{K[0,0]:.4f}, 0.0, {K[0,2]:.4f},")
                print(f"                  0.0, {K[1,1]:.4f}, {K[1,2]:.4f},")
                print(f"                  0.0, 0.0, 1.0]")
                print(f"  dist_coeffs: {D.tolist()}")

    print("\n完成!")


if __name__ == '__main__':
    main()
