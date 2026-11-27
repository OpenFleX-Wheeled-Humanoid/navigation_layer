#!/usr/bin/env python3
"""
建图时运行: 多点 GPS+TF 校准，计算 UTM→map 旋转角并记录 datum.yaml。

在建图过程中（FAST-LIO2 运行时），驱动机器人到不同位置，
每次按 Enter 采集 GPS + map→base_link TF 样本对。
至少 2 个点（间隔 ≥5m）后输入 done，脚本计算：
  - GPS datum（第一个点的 GPS 均值）
  - yaw_offset_rad: UTM→map 旋转角 θ
  - map_datum_tx/ty: 第一个校准点在 map 坐标系中的位置

保存到 datum.yaml，导航时 gps_initial_pose.py 使用旋转变换。

用法:
  ros2 run swerve_navigation record_gps_datum.py \
      --ros-args -p output:=/path/to/maps/3d/xxx/datum.yaml -p samples:=5
"""

import math
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import Buffer, TransformListener


# ──────────────────────────────────────────
# WGS84 → UTM 转换 (无外部依赖)
# ──────────────────────────────────────────

_WGS84_A = 6378137.0            # 赤道半径 (m)
_WGS84_F = 1.0 / 298.257223563  # 扁率
_WGS84_E2 = 2 * _WGS84_F - _WGS84_F ** 2  # 第一偏心率平方
_WGS84_E_PRIME2 = _WGS84_E2 / (1 - _WGS84_E2)
_K0 = 0.9996


def _deg2rad(d):
    return d * math.pi / 180.0


def wgs84_to_utm(lat_deg, lon_deg):
    """将 WGS84 (lat, lon) 转换为 UTM (easting, northing, zone)。"""
    if not (-80.0 <= lat_deg <= 84.0):
        raise ValueError(f'纬度 {lat_deg}° 超出 UTM 适用范围 [-80, 84]')

    lat = _deg2rad(lat_deg)
    lon = _deg2rad(lon_deg)

    # 经度归一化到 [-180, 180)，防止 180° 算出非法 zone 61
    lon_norm = (lon_deg + 180.0) % 360.0 - 180.0
    zone_number = int((lon_norm + 180.0) / 6.0) + 1
    zone_number = min(zone_number, 60)

    # 中央经线
    lon0 = _deg2rad((zone_number - 1) * 6 - 180 + 3)

    N = _WGS84_A / math.sqrt(1 - _WGS84_E2 * math.sin(lat) ** 2)
    T = math.tan(lat) ** 2
    C = _WGS84_E_PRIME2 * math.cos(lat) ** 2
    A = math.cos(lat) * (lon - lon0)

    # 子午线弧长
    M = _WGS84_A * (
        (1 - _WGS84_E2 / 4 - 3 * _WGS84_E2 ** 2 / 64 - 5 * _WGS84_E2 ** 3 / 256) * lat
        - (3 * _WGS84_E2 / 8 + 3 * _WGS84_E2 ** 2 / 32 + 45 * _WGS84_E2 ** 3 / 1024) * math.sin(2 * lat)
        + (15 * _WGS84_E2 ** 2 / 256 + 45 * _WGS84_E2 ** 3 / 1024) * math.sin(4 * lat)
        - (35 * _WGS84_E2 ** 3 / 3072) * math.sin(6 * lat)
    )

    easting = _K0 * N * (
        A
        + (1 - T + C) * A ** 3 / 6
        + (5 - 18 * T + T ** 2 + 72 * C - 58 * _WGS84_E_PRIME2) * A ** 5 / 120
    ) + 500000.0

    northing = _K0 * (
        M + N * math.tan(lat) * (
            A ** 2 / 2
            + (5 - T + 9 * C + 4 * C ** 2) * A ** 4 / 24
            + (61 - 58 * T + T ** 2 + 600 * C - 330 * _WGS84_E_PRIME2) * A ** 6 / 720
        )
    )

    if lat_deg < 0:
        northing += 10000000.0  # 南半球偏移

    return easting, northing, zone_number


# ──────────────────────────────────────────


class RecordGpsDatum(Node):
    def __init__(self):
        super().__init__('record_gps_datum')

        self.declare_parameter('output', '')
        self.declare_parameter('samples', 5)

        self.output_path = self.get_parameter('output').get_parameter_value().string_value
        self.num_samples = self.get_parameter('samples').get_parameter_value().integer_value

        if not self.output_path:
            self.get_logger().fatal('必须指定 output 参数，例如 -p output:=/path/to/datum.yaml')
            sys.exit(1)

        # TF2 listener for map→base_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # GPS — 回调写入, 采集线程通过 Event 等待
        self.latest_fix = None
        self.fix_lock = threading.Lock()
        self.fix_event = threading.Event()

        # Calibration points: list of (utm_e, utm_n, map_x, map_y, lat, lon, alt)
        self.cal_points = []

        # 采集状态: 后台线程设 True 表示正在采集，gps_cb 据此投递
        self.collecting = False

        # Subscribe to GPS
        self.sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'GPS+TF 校准工具已启动 (每个点采集 {self.num_samples} 个样本)')
        self.get_logger().info(
            '驱动机器人到不同位置，按 Enter 采集样本对，输入 done 完成校准')

        # Start interactive input thread
        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.input_thread.start()

    def gps_cb(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude) or math.isnan(msg.altitude):
            return
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return
        # Reject fixes with horizontal variance > 100 m^2 (10 m CEP)
        if msg.position_covariance[0] > 100.0 or msg.position_covariance[4] > 100.0:
            return
        if not self.collecting:
            return
        with self.fix_lock:
            self.latest_fix = msg
            self.fix_event.set()

    def _collect_samples(self):
        """Collect N paired GPS + TF samples and return averages.

        Called from background input thread. Does NOT call spin_once —
        all callbacks are driven by the main-thread executor (rclpy.spin).
        """
        lats, lons, alts = [], [], []
        map_xs, map_ys = [], []

        self.get_logger().info(f'开始采集 {self.num_samples} 个样本...')

        # Clear stale fix and enable collection
        with self.fix_lock:
            self.latest_fix = None
            self.fix_event.clear()
        self.collecting = True

        try:
            collected = 0
            while collected < self.num_samples:
                # Wait for gps_cb to deliver a new fix (driven by main thread spin)
                if not self.fix_event.wait(timeout=2.0):
                    self.get_logger().warn('等待 GPS fix...', throttle_duration_sec=2.0)
                    continue

                # Consume the fix
                with self.fix_lock:
                    fix = self.latest_fix
                    self.latest_fix = None
                    self.fix_event.clear()

                if fix is None:
                    continue

                # Get TF map→base_link at GPS fix timestamp for time-synchronized pairing
                fix_time = rclpy.time.Time.from_msg(fix.header.stamp)
                try:
                    tf = self.tf_buffer.lookup_transform(
                        'map', 'base_link', fix_time, timeout=Duration(seconds=0.5))
                except Exception as e:
                    self.get_logger().warn(
                        f'等待 map→base_link TF: {e}', throttle_duration_sec=2.0)
                    continue

                lats.append(fix.latitude)
                lons.append(fix.longitude)
                alts.append(fix.altitude)
                map_xs.append(tf.transform.translation.x)
                map_ys.append(tf.transform.translation.y)
                collected += 1
                self.get_logger().info(f'  样本 {collected}/{self.num_samples}')
        finally:
            self.collecting = False

        avg_lat = sum(lats) / len(lats)
        avg_lon = sum(lons) / len(lons)
        avg_alt = sum(alts) / len(alts)
        avg_mx = sum(map_xs) / len(map_xs)
        avg_my = sum(map_ys) / len(map_ys)

        e, n, zone = wgs84_to_utm(avg_lat, avg_lon)

        self.get_logger().info(
            f'  GPS: lat={avg_lat:.8f}, lon={avg_lon:.8f}\n'
            f'  UTM: E={e:.3f}, N={n:.3f} (zone {zone})\n'
            f'  Map TF: x={avg_mx:.3f}, y={avg_my:.3f}')

        return e, n, avg_mx, avg_my, avg_lat, avg_lon, avg_alt

    def _input_loop(self):
        """Interactive loop running on background thread."""
        try:
            while True:
                line = input(
                    f'\n[点 {len(self.cal_points) + 1}] '
                    f'按 Enter 采集样本对，输入 done 完成: ').strip().lower()

                if line == 'done':
                    if len(self.cal_points) < 2:
                        print(f'至少需要 2 个校准点 (当前 {len(self.cal_points)} 个)')
                        continue
                    self._compute_and_save()
                    os._exit(0)
                else:
                    # Collect samples for this point
                    result = self._collect_samples()
                    utm_e, utm_n, map_x, map_y, lat, lon, alt = result
                    self.cal_points.append(result)

                    # Check separation from first point
                    if len(self.cal_points) >= 2:
                        e0, n0 = self.cal_points[0][0], self.cal_points[0][1]
                        sep = math.sqrt((utm_e - e0) ** 2 + (utm_n - n0) ** 2)
                        self.get_logger().info(f'与第一个点的距离: {sep:.2f} m')
                        if sep < 5.0:
                            self.get_logger().warn(
                                f'距离仅 {sep:.2f}m，建议 ≥5m 以获得准确的旋转角')

                    self.get_logger().info(
                        f'已记录点 {len(self.cal_points)}'
                        f' (至少需要 2 个点，可继续添加更多)')

        except EOFError:
            # Stdin closed
            if len(self.cal_points) >= 2:
                self._compute_and_save()
            else:
                self.get_logger().error('输入已关闭且校准点不足，未保存')
            os._exit(0)

    def _compute_and_save(self):
        """Compute yaw offset from calibration points and save datum.yaml."""
        pts = self.cal_points

        # Datum = first calibration point
        datum_e, datum_n = pts[0][0], pts[0][1]
        datum_mx, datum_my = pts[0][2], pts[0][3]
        datum_lat, datum_lon, datum_alt = pts[0][4], pts[0][5], pts[0][6]

        # Compute yaw offset θ via closed-form 2D rotation
        # θ = atan2(Σ(utm_dx·map_dy - utm_dy·map_dx), Σ(utm_dx·map_dx + utm_dy·map_dy))
        sum_cross = 0.0  # Σ(utm_dx·map_dy - utm_dy·map_dx)
        sum_dot = 0.0    # Σ(utm_dx·map_dx + utm_dy·map_dy)

        for i in range(1, len(pts)):
            utm_dx = pts[i][0] - datum_e
            utm_dy = pts[i][1] - datum_n
            map_dx = pts[i][2] - datum_mx
            map_dy = pts[i][3] - datum_my

            sum_cross += utm_dx * map_dy - utm_dy * map_dx
            sum_dot += utm_dx * map_dx + utm_dy * map_dy

        yaw_offset = math.atan2(sum_cross, sum_dot)

        # Compute and print per-point residuals
        cos_y = math.cos(yaw_offset)
        sin_y = math.sin(yaw_offset)

        max_sep = 0.0
        self.get_logger().info(f'\n--- 校准结果 ---')
        self.get_logger().info(f'旋转角 θ = {yaw_offset:.6f} rad ({math.degrees(yaw_offset):.3f}°)')
        self.get_logger().info(f'Map datum: tx={datum_mx:.4f}, ty={datum_my:.4f}')
        self.get_logger().info(f'\n逐点残差:')

        for i, pt in enumerate(pts):
            utm_dx = pt[0] - datum_e
            utm_dy = pt[1] - datum_n
            pred_mx = cos_y * utm_dx - sin_y * utm_dy + datum_mx
            pred_my = sin_y * utm_dx + cos_y * utm_dy + datum_my
            err_x = pt[2] - pred_mx
            err_y = pt[3] - pred_my
            err = math.sqrt(err_x ** 2 + err_y ** 2)

            sep = math.sqrt(utm_dx ** 2 + utm_dy ** 2)
            if sep > max_sep:
                max_sep = sep

            self.get_logger().info(
                f'  点 {i + 1}: 残差 = {err:.4f} m '
                f'(dx={err_x:.4f}, dy={err_y:.4f}), 距datum {sep:.2f} m')

        # Save datum.yaml
        out_dir = os.path.dirname(self.output_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        with open(self.output_path, 'w') as f:
            f.write(f'datum_latitude: {datum_lat:.10f}\n')
            f.write(f'datum_longitude: {datum_lon:.10f}\n')
            f.write(f'datum_altitude: {datum_alt:.4f}\n')
            f.write(f'yaw_offset_rad: {yaw_offset:.6f}\n')
            f.write(f'map_datum_tx: {datum_mx:.4f}\n')
            f.write(f'map_datum_ty: {datum_my:.4f}\n')
            f.write(f'calibration_points: {len(pts)}\n')
            f.write(f'max_separation_m: {max_sep:.2f}\n')

        self.get_logger().info(
            f'\nDatum 已保存到 {self.output_path}\n'
            f'  latitude:  {datum_lat:.10f}\n'
            f'  longitude: {datum_lon:.10f}\n'
            f'  altitude:  {datum_alt:.4f}\n'
            f'  yaw_offset_rad: {yaw_offset:.6f} ({math.degrees(yaw_offset):.3f}°)\n'
            f'  map_datum_tx: {datum_mx:.4f}\n'
            f'  map_datum_ty: {datum_my:.4f}\n'
            f'  calibration_points: {len(pts)}\n'
            f'  max_separation_m: {max_sep:.2f}')


def main():
    rclpy.init()
    node = RecordGpsDatum()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
