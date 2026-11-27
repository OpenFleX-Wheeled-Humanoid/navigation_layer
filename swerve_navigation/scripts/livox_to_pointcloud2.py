#!/usr/bin/env python3

import math

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class LivoxToPointCloud2Bridge(Node):
    def __init__(self) -> None:
        super().__init__('livox_to_pointcloud2_bridge')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/lidar_points')
        self.declare_parameter('filter_invalid', True)
        self.declare_parameter('min_range', 0.0)
        self.declare_parameter('max_range', 100.0)

        self.input_topic_ = self.get_parameter('input_topic').value
        self.output_topic_ = self.get_parameter('output_topic').value
        self.filter_invalid_ = bool(self.get_parameter('filter_invalid').value)
        self.min_range_ = float(self.get_parameter('min_range').value)
        self.max_range_ = float(self.get_parameter('max_range').value)

        self.publisher_ = self.create_publisher(PointCloud2, self.output_topic_, 10)
        self.create_subscription(CustomMsg, self.input_topic_, self._callback, 10)

    def _point_is_valid(self, point) -> bool:
        if not self.filter_invalid_:
            return True
        if point.line >= 4:
            return False
        if not (((point.tag & 0x30) == 0x10) or ((point.tag & 0x30) == 0x00)):
            return False
        if (point.tag & 0x03) != 0x00:
            return False
        distance_sq = point.x * point.x + point.y * point.y + point.z * point.z
        return self.min_range_ * self.min_range_ <= distance_sq <= self.max_range_ * self.max_range_

    def _callback(self, msg: CustomMsg) -> None:
        points = []
        for point in msg.points:
            if not self._point_is_valid(point):
                continue
            intensity = float(point.reflectivity)
            points.append((point.x, point.y, point.z, intensity))

        cloud = point_cloud2.create_cloud(
            header=Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id),
            fields=[
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ],
            points=points,
        )
        self.publisher_.publish(cloud)


def main() -> None:
    rclpy.init()
    node = LivoxToPointCloud2Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
