#!/usr/bin/env python3

import math

import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudToLivoxBridge(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_to_livox_bridge')

        self.declare_parameter('input_topic', '/sim/mid360/points')
        self.declare_parameter('output_topic', '/livox/lidar')
        self.declare_parameter('scan_period', 0.1)
        self.declare_parameter('min_elevation', -0.1222)
        self.declare_parameter('max_elevation', 0.9076)
        self.declare_parameter('line_count', 16)
        self.declare_parameter('frame_id', 'livox_frame')
        self.declare_parameter('lidar_id', 1)
        self.declare_parameter('default_reflectivity', 100)

        self.input_topic_ = self.get_parameter('input_topic').value
        self.output_topic_ = self.get_parameter('output_topic').value
        self.scan_period_ = float(self.get_parameter('scan_period').value)
        self.min_elevation_ = float(self.get_parameter('min_elevation').value)
        self.max_elevation_ = float(self.get_parameter('max_elevation').value)
        self.line_count_ = max(1, int(self.get_parameter('line_count').value))
        self.frame_id_ = self.get_parameter('frame_id').value
        self.lidar_id_ = int(self.get_parameter('lidar_id').value)
        self.default_reflectivity_ = int(self.get_parameter('default_reflectivity').value)

        self.publisher_ = self.create_publisher(CustomMsg, self.output_topic_, 10)
        self.create_subscription(PointCloud2, self.input_topic_, self._callback, 10)

    def _callback(self, msg: PointCloud2) -> None:
        field_names = {field.name for field in msg.fields}
        use_intensity = 'intensity' in field_names
        requested = ('x', 'y', 'z', 'intensity') if use_intensity else ('x', 'y', 'z')

        raw_points = list(point_cloud2.read_points(msg, field_names=requested, skip_nans=True))
        if not raw_points:
            return

        ordered_points = []
        for point in raw_points:
            if use_intensity:
                x, y, z, intensity = point
            else:
                x, y, z = point
                intensity = self.default_reflectivity_
            ordered_points.append((math.atan2(y, x), x, y, z, intensity))
        ordered_points.sort(key=lambda item: item[0])

        livox_msg = CustomMsg()
        livox_msg.header.stamp = msg.header.stamp
        livox_msg.header.frame_id = self.frame_id_ or msg.header.frame_id
        livox_msg.timebase = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        livox_msg.lidar_id = self.lidar_id_
        livox_msg.point_num = len(ordered_points)
        livox_msg.points = []

        scan_period_ns = int(self.scan_period_ * 1_000_000_000)
        denominator = max(1, len(ordered_points) - 1)
        elev_span = max(1e-6, self.max_elevation_ - self.min_elevation_)

        for index, (_, x, y, z, intensity) in enumerate(ordered_points):
            point = CustomPoint()
            point.x = float(x)
            point.y = float(y)
            point.z = float(z)
            point.reflectivity = max(0, min(255, int(intensity)))
            point.tag = 0x10

            elevation = math.atan2(z, math.hypot(x, y))
            normalized = (elevation - self.min_elevation_) / elev_span
            line = int(normalized * self.line_count_)
            point.line = max(0, min(self.line_count_ - 1, line))

            point.offset_time = int(index * scan_period_ns / denominator)
            livox_msg.points.append(point)

        self.publisher_.publish(livox_msg)


def main() -> None:
    rclpy.init()
    node = PointCloudToLivoxBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
