#!/usr/bin/env python3
"""
导航时运行: GPS → UTM → map 坐标 → /initialpose

读取建图时记录的 datum.yaml，订阅 /gps/fix，
将当前 GPS 转为 UTM 坐标后减去 datum 得到 map 坐标，
发布到 /initialpose 供 icp_registration_node 做精确定位。

数据流:
  [ublox GPS] → /gps/fix → [gps_initial_pose] → /initialpose → [icp_registration] → map→odom TF

用法 (launch 中自动调用):
  ros2 run swerve_navigation gps_initial_pose.py \
      --ros-args -p datum_path:=/path/to/datum.yaml -p samples:=5
"""

import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped

import yaml


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


class GpsInitialPose(Node):
    def __init__(self):
        super().__init__('gps_initial_pose')

        self.declare_parameter('datum_path', '')
        self.declare_parameter('samples', 5)
        self.declare_parameter('publish_count', 3)

        datum_path = self.get_parameter('datum_path').get_parameter_value().string_value
        self.num_samples = self.get_parameter('samples').get_parameter_value().integer_value
        self.publish_count = self.get_parameter('publish_count').get_parameter_value().integer_value

        if not datum_path or not os.path.isfile(datum_path):
            self.get_logger().fatal(f'datum_path 无效或文件不存在: "{datum_path}"')
            sys.exit(1)

        # 加载 datum
        with open(datum_path, 'r') as f:
            datum = yaml.safe_load(f)

        self.datum_lat = float(datum['datum_latitude'])
        self.datum_lon = float(datum['datum_longitude'])

        self.datum_easting, self.datum_northing, self.datum_zone = wgs84_to_utm(
            self.datum_lat, self.datum_lon)

        # Load optional calibration fields (backward-compatible: default to no rotation)
        self.yaw_offset = float(datum.get('yaw_offset_rad', 0.0))
        self.map_datum_tx = float(datum.get('map_datum_tx', 0.0))
        self.map_datum_ty = float(datum.get('map_datum_ty', 0.0))
        self.cos_yaw = math.cos(self.yaw_offset)
        self.sin_yaw = math.sin(self.yaw_offset)

        self.get_logger().info(
            f'Datum 已加载: lat={self.datum_lat:.8f}, lon={self.datum_lon:.8f}, '
            f'UTM zone={self.datum_zone}, '
            f'E={self.datum_easting:.2f}, N={self.datum_northing:.2f}, '
            f'yaw_offset={self.yaw_offset:.4f} rad, '
            f'tx={self.map_datum_tx:.3f}, ty={self.map_datum_ty:.3f}')

        self.lats = []
        self.lons = []
        self.initial_pose_published = False

        # 发布者
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # GPS map position publisher (transient_local so late subscribers get last value)
        gps_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.gps_map_pub = self.create_publisher(
            PointStamped, '/gps/map_position', gps_qos)

        # 订阅 GPS
        self.sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'等待 {self.num_samples} 个 GPS 样本计算初始位姿...')

    def gps_cb(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            self.get_logger().warn('GPS 无定位，跳过', throttle_duration_sec=5.0)
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return
        if msg.position_covariance[0] > 100.0 or msg.position_covariance[4] > 100.0:
            return

        if not self.initial_pose_published:
            # Sampling mode: collect fixes for initial pose
            self.lats.append(msg.latitude)
            self.lons.append(msg.longitude)

            n = len(self.lats)
            self.get_logger().info(f'GPS 样本 {n}/{self.num_samples}')

            if n >= self.num_samples:
                self.compute_and_publish()
        else:
            # Tracking mode: publish each fix as map position
            self._publish_gps_map_position(msg.latitude, msg.longitude)

    def _gps_to_map(self, lat, lon):
        """Convert WGS84 (lat, lon) to map frame (x, y)."""
        easting, northing, zone = wgs84_to_utm(lat, lon)
        if zone != self.datum_zone:
            self.get_logger().warn(
                f'UTM zone mismatch: {zone} vs datum {self.datum_zone}',
                throttle_duration_sec=30.0)
        dx = easting - self.datum_easting
        dy = northing - self.datum_northing
        map_x = self.cos_yaw * dx - self.sin_yaw * dy + self.map_datum_tx
        map_y = self.sin_yaw * dx + self.cos_yaw * dy + self.map_datum_ty
        return map_x, map_y

    def _publish_gps_map_position(self, lat, lon):
        """Publish current GPS position in map frame on /gps/map_position."""
        map_x, map_y = self._gps_to_map(lat, lon)
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'map'
        pt.point.x = map_x
        pt.point.y = map_y
        pt.point.z = 0.0
        self.gps_map_pub.publish(pt)

    def compute_and_publish(self):
        avg_lat = sum(self.lats) / len(self.lats)
        avg_lon = sum(self.lons) / len(self.lons)

        map_x, map_y = self._gps_to_map(avg_lat, avg_lon)

        self.get_logger().info(
            f'GPS 位置: lat={avg_lat:.8f}, lon={avg_lon:.8f}\n'
            f'  Map 坐标: x={map_x:.3f}, y={map_y:.3f}')

        # 构造 PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = map_x
        pose_msg.pose.pose.position.y = map_y
        pose_msg.pose.pose.position.z = 0.0
        # Yaw = identity, ICP 的 yaw_offset 网格搜索处理朝向
        pose_msg.pose.pose.orientation.w = 1.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        # 协方差: 置大值表示不确定
        pose_msg.pose.covariance[0] = 25.0   # x 方差 (5m)
        pose_msg.pose.covariance[7] = 25.0   # y 方差
        pose_msg.pose.covariance[35] = 99.0  # yaw 方差

        # 发布多次确保 ICP 收到
        for i in range(self.publish_count):
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.pose_pub.publish(pose_msg)
            self.get_logger().info(
                f'已发布 /initialpose ({i + 1}/{self.publish_count}): '
                f'x={map_x:.3f}, y={map_y:.3f}')

        # Publish initial GPS map position
        self._publish_gps_map_position(avg_lat, avg_lon)

        self.initial_pose_published = True
        self.get_logger().info(
            'GPS 初始位姿发布完成，切换到跟踪模式 (持续发布 /gps/map_position)')


def main():
    rclpy.init()
    node = GpsInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
