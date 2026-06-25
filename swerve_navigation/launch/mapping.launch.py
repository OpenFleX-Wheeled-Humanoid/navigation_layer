"""
建图模式启动（带 PGO 回环检测 + 实时回环修正）

TF 链路:
  map → odom             (FAST-LIVO2 兼容桥接发布，含 PGO 回环修正 + 静止冻结)
  odom → base_link       (底盘控制器发布，enable_odom_tf: true)
  base_link → mid360_link  (robot_state_publisher 静态 TF)

启动顺序:
1. robot_state_publisher — 静态 TF
2. controller_manager — ros2_control (controllers_mapping.yaml, enable_odom_tf: true)
3. joint_state_broadcaster → swerve_drive_controller — 底盘控制 + /odom topic + odom→base_link TF
4. livox_ros_driver2 — MID-360 驱动
5. (可选) realsense2_camera — D435 相机
6. FAST-LIVO2 (建图配置，经兼容桥接) — 发布 map→odom TF (含 PGO 修正)
7. PGO — 回环检测 + 位姿图优化 + /pgo/offset 发布
8. (可选) u-blox GPS — 室外建图时提供 /gps/fix，配合 record_gps_datum.py 校准
9. rviz2 — 可视化

室外建图（启动 GPS）:
  ros2 launch swerve_navigation mapping.launch.py use_gps:=true

如需显式启动相机:
  ros2 launch swerve_navigation mapping.launch.py use_camera:=true

建图完成后保存 PGO 优化后的地图:
  mkdir -p ~/openflex_all/openflex_maps/3d/your_map
  ros2 service call /pgo/save_maps interface/srv/SaveMaps \\
    "{file_path: '~/openflex_all/openflex_maps/3d/your_map', save_patches: true}"
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # ---------- 路径 ----------
    bringup_dir = get_package_share_directory('swerve_bringup')
    desc_dir = get_package_share_directory('swerve_description')
    fast_livo_dir = get_package_share_directory('fast_livo')
    pgo_dir = get_package_share_directory('pgo')

    # ---------- 底盘运动学参数 ----------
    with open(os.path.join(desc_dir, 'config', 'chassis_version_6.0.yaml')) as f:
        _cp = yaml.safe_load(f)['chassis']
    _wheel_radius   = str(_cp['wheel_radius'])
    _half_wheelbase = str(_cp['wheelbase'] / 2.0)
    _half_track     = str(_cp['track_width'] / 2.0)
    nav_dir = get_package_share_directory('swerve_navigation')

    # ---------- Launch 参数 ----------
    use_rviz = LaunchConfiguration('use_rviz')
    use_gps = LaunchConfiguration('use_gps')
    use_camera = LaunchConfiguration('use_camera')
    steering_can = LaunchConfiguration('steering_can_interface')
    driving_can = LaunchConfiguration('driving_can_interface')
    mapping_max_wheel_speed = LaunchConfiguration('mapping_max_wheel_speed')
    mapping_wheel_accel_limit = LaunchConfiguration('mapping_wheel_accel_limit')

    urdf_file = os.path.join(desc_dir, 'urdf', 'swerve.urdf.xacro')
    controllers_yaml = os.path.join(bringup_dir, 'config', 'controllers_mapping.yaml')
    controllers_params = RewrittenYaml(
        source_file=controllers_yaml,
        param_rewrites={
            'max_wheel_speed': mapping_max_wheel_speed,
            'wheel_accel_limit': mapping_wheel_accel_limit,
            'wheel_radius': _wheel_radius,
            'fl_pos_x':  _half_wheelbase,
            'fl_pos_y':  _half_track,
            'fr_pos_x':  _half_wheelbase,
            'fr_pos_y': f'-{_half_track}',
            'bl_pos_x': f'-{_half_wheelbase}',
            'bl_pos_y':  _half_track,
            'br_pos_x': f'-{_half_wheelbase}',
            'br_pos_y': f'-{_half_track}',
        },
        convert_types=True,
    )
    lio_core_config = os.path.join(fast_livo_dir, 'config', 'livo_mapping.yaml')
    lio_bridge_config = os.path.join(fast_livo_dir, 'config', 'bridge_mapping.yaml')
    pgo_config = os.path.join(pgo_dir, 'config', 'pgo.yaml')

    # ---------- URDF ----------
    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' steering_can_interface:=', steering_can,
            ' driving_can_interface:=', driving_can,
        ]),
        value_type=str,
    )

    # ========== 1. robot_state_publisher ==========
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
        }],
        output='screen',
    )

    # ========== 2. controller_manager ==========
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controllers_params,
        ],
        output='screen',
    )

    # ========== 3. joint_state_broadcaster → swerve_drive_controller ==========
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--service-call-timeout', '60.0',
        ],
        output='screen',
    )

    sdc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'swerve_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--service-call-timeout', '60.0',
        ],
        output='screen',
    )
    sdc_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[sdc_spawner],
        )
    )

    # ========== 4. livox_ros_driver2 ==========
    # Import lidar config helper to merge ~/.openflex/lidar_config.yaml
    livox_dir = get_package_share_directory('livox_ros_driver2')
    import sys
    livox_launch_dir = os.path.join(livox_dir, 'launch_ROS2')
    sys.path.insert(0, livox_launch_dir)
    from lidar_config_helper import merge_lidar_config

    base_lidar_config = os.path.join(livox_dir, 'config', 'MID360_config.json')
    merged_lidar_config = merge_lidar_config(base_lidar_config)

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': merged_lidar_config,
            'cmdline_bd_code': 'livox0000000001',
        }],
    )

    # ========== 5. RealSense D435 (LIVO 视觉融合) ==========
    d435_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d435',
        namespace='camera',
        output='screen',
        parameters=[{
            'camera_name': 'd435',
            'base_frame_id': 'd435_link',
            'rgb_camera.color_profile': '1280x720x30',
            'depth_module.depth_profile': '848x480x30',
            'depth_module.infra_profile': '848x480x30',
            'pointcloud.enable': True,
            'pointcloud.ordered_pc': False,
            'align_depth.enable': True,
            'global_time_enabled': True,
        }],
        condition=IfCondition(use_camera),
    )

    # ========== 6. FAST-LIVO2 + compatibility bridge (map→odom, 建图配置) ==========
    lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fast_livo_dir, 'launch', 'swerve_lio.launch.py')),
        launch_arguments={
            'namespace': 'fastlio2',
            'core_params_file': lio_core_config,
            'bridge_params_file': lio_bridge_config,
        }.items(),
    )

    # ========== 7. PGO (回环检测 + 图优化 + /pgo/offset 发布) ==========
    pgo_node = Node(
        package='pgo',
        executable='pgo_node',
        namespace='pgo',
        parameters=[{'config_path': pgo_config}],
        output='screen',
    )

    # ========== 8. u-blox GPS (室外建图, use_gps:=true) ==========
    ublox_config = os.path.join(nav_dir, 'config', 'ublox_gps.yaml')
    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[ublox_config],
        remappings=[('fix', '/gps/fix')],
        condition=IfCondition(use_gps),
    )

    # ========== 9. rviz2 ==========
    rviz_config = os.path.join(pgo_dir, 'rviz', 'pgo.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # --- 声明参数 ---
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否启动 RViz'),
        DeclareLaunchArgument('use_gps', default_value='false',
                              description='是否启动 GPS (室外建图设为 true)'),
        DeclareLaunchArgument('use_camera', default_value='false',
                              description='是否启动 D435 相机'),
        DeclareLaunchArgument('steering_can_interface', default_value='can5',
                              description='转向电机 CAN 接口'),
        DeclareLaunchArgument('driving_can_interface', default_value='can4',
                              description='驱动电机 CAN 接口'),
        DeclareLaunchArgument('mapping_max_wheel_speed', default_value='2.0',
                              description='建图/遥控模式轮速上限 (m/s)'),
        DeclareLaunchArgument('mapping_wheel_accel_limit', default_value='1.2',
                              description='建图/遥控模式轮速变化率上限 (m/s^2)'),

        # --- 启动节点 ---
        robot_state_publisher,
        controller_manager,
        jsb_spawner,
        sdc_after_jsb,
        livox_driver,
        d435_node,
        TimerAction(period=4.0, actions=[lio_node]),
        TimerAction(period=5.0, actions=[pgo_node]),
        ublox_node,
        rviz_node,
    ])
