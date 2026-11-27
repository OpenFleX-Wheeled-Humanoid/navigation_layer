from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # URDF 已有 base_link->mid360_link 静态 TF，无需额外 static_transform_publisher
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/fastlio2/body_cloud'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'mid360_link',
                'transform_tolerance': 0.01,
                'min_height': -0.15,   # 相对 mid360_link，过滤地面
                'max_height': 0.40,    # 底盘高度约 0.47m，保留障碍物
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0043,
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
