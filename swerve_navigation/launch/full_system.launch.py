"""
全系统启动 (导航模式)
导航主链路:
1. robot_state_publisher + ros2_control
2. Livox + FAST-LIVO2(经兼容桥接) 提供 /fastlio2/lio_odom
3. safe_odom_mux 在 FAST-LIVO2 桥接里程计 / 轮式里程计之间切换并发布 odom -> base_link
4. ICP 提供 map -> odom
5. Nav2 直接接收 RViz 2D Goal Pose
6. 导航避障只使用 Livox / FAST-LIO 点云，不接入 D435
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

# Import lidar config helper
livox_dir_for_import = get_package_share_directory('livox_ros_driver2')
import sys
sys.path.insert(0, os.path.join(livox_dir_for_import, 'launch_ROS2'))
from lidar_config_helper import merge_lidar_config

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _expand_path(raw: str) -> str:
    return os.path.abspath(os.path.expandvars(os.path.expanduser(raw)))


def _validate_map_yaml(context, *args, **kwargs):
    resolved = _expand_path(LaunchConfiguration('map_yaml').perform(context))
    if not resolved:
        raise RuntimeError('必须提供 map_yaml')
    if not os.path.isfile(resolved):
        raise RuntimeError(f'map_yaml 不存在或不可读: {resolved}')
    return [SetLaunchConfiguration('map_yaml', resolved)]


def _setup_localization_nodes(context, *args, **kwargs):
    use_gps = LaunchConfiguration('use_gps').perform(context).lower() in ('true', '1', 'yes')

    fast_livo_dir = get_package_share_directory('fast_livo')
    nav_dir = get_package_share_directory('swerve_navigation')
    icp_dir = get_package_share_directory('icp_registration')

    nodes = []

    lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fast_livo_dir, 'launch', 'swerve_lio.launch.py')),
        launch_arguments={
            'namespace': 'fastlio2',
            'core_params_file': os.path.join(fast_livo_dir, 'config', 'livo_navigation.yaml'),
            'bridge_params_file': os.path.join(fast_livo_dir, 'config', 'bridge_navigation.yaml'),
        }.items(),
    )
    nodes.append(lio_node)

    safe_odom_mux = Node(
        package='swerve_navigation',
        executable='safe_odom_mux.py',
        name='safe_odom_mux',
        parameters=[{
            'use_sim_time': False,
            'lio_odom_topic': '/fastlio2/lio_odom',
            'wheel_odom_topic': '/odom',
            'output_odom_topic': '/odom_safe',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'publish_tf': True,
        }],
        output='screen',
    )
    nodes.append(safe_odom_mux)

    cmd_vel_safety_gate = Node(
        package='swerve_navigation',
        executable='cmd_vel_safety_gate.py',
        name='cmd_vel_safety_gate',
        parameters=[{
            'use_sim_time': False,
            'input_cmd_topic': '/cmd_vel_safe_in',
            'output_cmd_topic': '/cmd_vel_safe',
            'hold_topic': '/localization_hold',
            'hold_duration': 1.5,
            'cmd_timeout': 0.5,
            'publish_rate': 30.0,
        }],
        output='screen',
    )
    nodes.append(cmd_vel_safety_gate)

    resolved_pcd_path = _expand_path(LaunchConfiguration('pcd_path').perform(context))
    if not resolved_pcd_path:
        raise RuntimeError('必须提供 pcd_path')
    if not os.path.isfile(resolved_pcd_path):
        raise RuntimeError(f'pcd_path 不存在或不可读: {resolved_pcd_path}')

    icp_config = os.path.join(icp_dir, 'config', 'icp.yaml')

    if use_gps:
        resolved_datum_path = _expand_path(LaunchConfiguration('datum_path').perform(context))
        if not resolved_datum_path:
            raise RuntimeError('use_gps=true 时必须提供 datum_path')
        if not os.path.isfile(resolved_datum_path):
            raise RuntimeError(f'datum_path 不存在: {resolved_datum_path}')

        ublox_config = os.path.join(nav_dir, 'config', 'ublox_gps.yaml')
        nodes.append(
            Node(
                package='ublox_gps',
                executable='ublox_gps_node',
                name='ublox_gps_node',
                output='screen',
                parameters=[ublox_config],
                remappings=[('fix', '/gps/fix')],
            )
        )
        nodes.append(
            Node(
                package='swerve_navigation',
                executable='gps_initial_pose.py',
                name='gps_initial_pose',
                parameters=[{'datum_path': resolved_datum_path}],
                output='screen',
            )
        )
        nodes.append(
            Node(
                package='icp_registration',
                executable='icp_registration_node',
                parameters=[
                    icp_config,
                    {
                        'pcd_path': resolved_pcd_path,
                        'use_sim_time': False,
                        'laser_frame_id': 'base_link',
                        'pointcloud_topic': '/fastlio2/body_cloud',
                        'initial_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    },
                ],
                output='screen',
            )
        )
    else:
        import yaml as _yaml

        pose_str = LaunchConfiguration('initial_pose').perform(context)
        try:
            pose_list = _yaml.safe_load(pose_str)
            if not isinstance(pose_list, list) or len(pose_list) != 6:
                raise ValueError
            pose_list = [float(v) for v in pose_list]
        except Exception as exc:
            raise RuntimeError(
                f'initial_pose 必须为 6 元素列表 [x,y,z,roll,pitch,yaw]，当前值: {pose_str}'
            ) from exc

        nodes.append(
            Node(
                package='icp_registration',
                executable='icp_registration_node',
                parameters=[
                    icp_config,
                    {
                        'pcd_path': resolved_pcd_path,
                        'use_sim_time': False,
                        'laser_frame_id': 'base_link',
                        'pointcloud_topic': '/fastlio2/body_cloud',
                        'initial_pose': pose_list,
                    },
                ],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    bringup_dir = get_package_share_directory('swerve_bringup')
    desc_dir = get_package_share_directory('swerve_description')
    nav_dir = get_package_share_directory('swerve_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ---------- 底盘运动学参数 ----------
    with open(os.path.join(desc_dir, 'config', 'chassis_version_6.0.yaml')) as f:
        _cp = yaml.safe_load(f)['chassis']
    _wheel_radius   = str(_cp['wheel_radius'])
    _half_wheelbase = str(_cp['wheelbase'] / 2.0)
    _half_track     = str(_cp['track_width'] / 2.0)

    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map_yaml')
    steering_can = LaunchConfiguration('steering_can_interface')
    driving_can = LaunchConfiguration('driving_can_interface')
    nav_controller_max_wheel_speed = LaunchConfiguration('nav_controller_max_wheel_speed')
    nav_controller_wheel_accel_limit = LaunchConfiguration('nav_controller_wheel_accel_limit')

    urdf_file = os.path.join(desc_dir, 'urdf', 'swerve.urdf.xacro')
    controllers_yaml = os.path.join(bringup_dir, 'config', 'controllers_navigation.yaml')
    controllers_params = RewrittenYaml(
        source_file=controllers_yaml,
        param_rewrites={
            'max_wheel_speed': nav_controller_max_wheel_speed,
            'wheel_accel_limit': nav_controller_wheel_accel_limit,
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
    nav2_params = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    collision_monitor_params = os.path.join(nav_dir, 'config', 'collision_monitor_params.yaml')
    bt_xml = os.path.join(nav_dir, 'behavior_trees', 'navigate_to_pose_smoothed.xml')
    bt_through_poses_xml = os.path.join(nav_dir, 'behavior_trees', 'navigate_through_poses_smoothed.xml')
    rviz_config = os.path.join(nav_dir, 'rviz', 'navigation_debug.rviz')

    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' steering_can_interface:=', steering_can,
            ' driving_can_interface:=', driving_can,
        ]),
        value_type=str,
    )

    configured_nav_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={
            'yaml_filename': map_yaml,
            'default_nav_to_pose_bt_xml': bt_xml,
            'default_nav_through_poses_bt_xml': bt_through_poses_xml,
        },
        convert_types=True,
    )
    map_server_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={'yaml_filename': map_yaml},
        convert_types=True,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
        }],
        output='screen',
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controllers_params,
        ],
        output='screen',
    )

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
        OnProcessExit(target_action=jsb_spawner, on_exit=[sdc_spawner])
    )

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
            'user_config_path': merge_lidar_config(os.path.join(
                get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json')),
            'cmdline_bd_code': 'livox0000000001',
        }],
    )

    livox_pointcloud_relay = Node(
        package='swerve_navigation',
        executable='pointcloud_qos_relay.py',
        name='livox_pointcloud_relay',
        parameters=[{
            'input_topic': '/fastlio2/body_cloud',
            'output_topic': '/livox/body_cloud_nav2',
            'restamp_on_receive': True,
            'filter_min_z': -0.15,
            'filter_max_z': 1.80,
            # Self-box filter: 去除机身+轮子的 LiDAR 回波
            # track_width=0.52 → half=0.26, 轮子半径 0.075, 转向90°时
            # 轮子外缘达 y=0.26+0.075=0.335m, 留余量取 ±0.44
            # z 下限: 轮子底部在 base_link 下方 -0.235m, 留余量取 -0.30
            'self_filter_enabled': True,
            'self_filter_min_x': -0.45,
            'self_filter_max_x': 0.45,
            'self_filter_min_y': -0.44,
            'self_filter_max_y': 0.44,
            'self_filter_min_z': -0.30,
            'self_filter_max_z': 0.45,
        }],
        output='screen',
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/livox/body_cloud_nav2'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': -0.15,
            'max_height': 0.40,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        output='screen',
    )

    local_window_marker = Node(
        package='swerve_navigation',
        executable='local_window_marker.py',
        name='local_window_marker',
        parameters=[{
            'topic': '/local_window_marker',
            'frame_id': 'base_footprint',
            'size_x': 6.0,
            'size_y': 6.0,
            'thickness': 0.01,
            'z_offset': 0.005,
            'alpha': 0.18,
        }],
        output='screen',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[map_server_params],
    )
    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[collision_monitor_params],
    )
    collision_monitor_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
    )

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_sim_time': 'false',
            'params_file': configured_nav_params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

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
        DeclareLaunchArgument('use_rviz', default_value='true', description='是否启动 RViz'),
        DeclareLaunchArgument('use_gps', default_value='false', description='室外导航: GPS 自动提供初始位姿 + ICP 精确定位; 室内设为 false'),
        DeclareLaunchArgument('pcd_path', default_value='', description='ICP 参考地图 PCD 文件路径'),
        DeclareLaunchArgument('datum_path', default_value='', description='GPS 基准点文件'),
        DeclareLaunchArgument('map_yaml', default_value='', description='2D 导航地图 YAML 文件路径'),
        DeclareLaunchArgument('steering_can_interface', default_value='can5', description='转向电机 CAN 接口'),
        DeclareLaunchArgument('driving_can_interface', default_value='can4', description='驱动电机 CAN 接口'),
        DeclareLaunchArgument('nav_controller_max_wheel_speed', default_value='1.2',
                              description='导航模式舵轮控制器轮速上限 (m/s)'),
        DeclareLaunchArgument('nav_controller_wheel_accel_limit', default_value='1.0',
                              description='导航模式舵轮控制器轮速变化率上限 (m/s^2)'),
        DeclareLaunchArgument('initial_pose', default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]', description='ICP 初始位姿 [x, y, z, roll, pitch, yaw]'),
        OpaqueFunction(function=_validate_map_yaml),
        robot_state_publisher,
        controller_manager,
        jsb_spawner,
        sdc_after_jsb,
        livox_driver,
        livox_pointcloud_relay,
        pointcloud_to_laserscan,
        local_window_marker,
        OpaqueFunction(function=_setup_localization_nodes),
        map_server_node,
        map_server_lifecycle,
        collision_monitor_node,
        collision_monitor_lifecycle,
        TimerAction(period=3.0, actions=[nav2_navigation_launch]),
        rviz_node,
    ])
