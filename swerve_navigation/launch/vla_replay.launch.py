#!/usr/bin/env python3
"""
VLA Replay Launch File

This launch file starts the minimal components needed for VLA policy replay.
It should be used together with wholebody_vla_record.launch.py which provides:
  - Robot hardware (chassis + arms + lift + head)
  - FAST-LIO2 SLAM for odometry
  - Cameras

This file adds:
  - Future-odom pose tracker for pose-mode VLA policies
  - Optional Nav2 controller_server + path bridge for legacy trajectory replay
  - Replay cmd_vel conditioner + Nav2 velocity_smoother for swerve-friendly output
  - Lifecycle manager for velocity_smoother and optional controller_server

Usage:
  Terminal 1 - Start robot hardware and SLAM:
    ros2 launch <openflex_vla_package> wholebody_vla_record.launch.py enable_vr_teleop:=false

  Terminal 2 - Start VLA replay support (this file):
    ros2 launch swerve_navigation vla_replay.launch.py

  Terminal 3 - Run replay:
    ./scripts/lerobot-replay-openflex.sh \\
      --robot.type=openarmx_follower_ros2 \\
      --robot.skip_send_action=false \\
      --robot.ros2.enable_base=true \\
      --robot.ros2.enable_head=true \\
      --robot.ros2.enable_lift=true \\
      --robot.ros2.odom_topic=/fastlio2/lio_odom \\
      --robot.ros2.base_action_mode=pose \\
      --robot.ros2.base_path_coordinate_mode=episode_local \\
      --robot.ros2.base_path_topic=/vla/base_path \\
      --robot.ros2.base_path_frame_id=odom \\
      --dataset.repo_id=local/openflex_base_vla_test \\
      --dataset.episode=0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_dir = get_package_share_directory('swerve_navigation')

    # Parameters - Use VLA-optimized config
    nav2_params_file = os.path.join(nav_dir, 'config', 'nav2_params_vla_replay.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    controller_frequency_arg = DeclareLaunchArgument(
        'controller_frequency',
        default_value='20.0',
        description='Controller server frequency (Hz)'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/fastlio2/lio_odom',
        description='Odometry topic for VLA path bridge'
    )

    vla_path_topic_arg = DeclareLaunchArgument(
        'vla_path_topic',
        default_value='/vla/base_path',
        description='VLA predicted base path topic'
    )

    base_control_mode_arg = DeclareLaunchArgument(
        'base_control_mode',
        default_value='pose_tracker',
        description="Base replay controller: 'pose_tracker' for pose action policies, 'follow_path' for legacy trajectory policies"
    )

    follow_path_action_arg = DeclareLaunchArgument(
        'follow_path_action',
        default_value='/follow_path',
        description='Nav2 FollowPath action name'
    )

    pose_tracker_target_timeout_arg = DeclareLaunchArgument(
        'pose_tracker_target_timeout',
        default_value='2.0',
        description='How long the pose tracker holds the last VLA target after path messages stop'
    )

    pose_tracker_kx_arg = DeclareLaunchArgument(
        'pose_tracker_kx',
        default_value='0.7',
        description='Pose tracker body-frame X proportional gain'
    )

    pose_tracker_ky_arg = DeclareLaunchArgument(
        'pose_tracker_ky',
        default_value='0.7',
        description='Pose tracker body-frame Y proportional gain'
    )

    pose_tracker_k_yaw_arg = DeclareLaunchArgument(
        'pose_tracker_k_yaw',
        default_value='0.9',
        description='Pose tracker yaw proportional gain'
    )

    pose_tracker_max_wz_arg = DeclareLaunchArgument(
        'pose_tracker_max_wz',
        default_value='0.25',
        description='Pose tracker maximum yaw rate'
    )

    pose_tracker_jump_limit_xy_arg = DeclareLaunchArgument(
        'pose_tracker_jump_limit_xy',
        default_value='0.15',
        description='Maximum accepted target XY jump per VLA update before clipping'
    )

    pose_tracker_jump_limit_yaw_arg = DeclareLaunchArgument(
        'pose_tracker_jump_limit_yaw',
        default_value='0.35',
        description='Maximum accepted target yaw jump per VLA update before clipping'
    )

    pose_tracker_xy_deadband_arg = DeclareLaunchArgument(
        'pose_tracker_xy_deadband',
        default_value='0.008',
        description='Pose tracker XY deadband in meters'
    )

    pose_tracker_yaw_deadband_arg = DeclareLaunchArgument(
        'pose_tracker_yaw_deadband',
        default_value='0.015',
        description='Pose tracker yaw deadband in radians'
    )

    pose_tracker_target_filter_alpha_arg = DeclareLaunchArgument(
        'pose_tracker_target_filter_alpha',
        default_value='0.6',
        description='Pose tracker target low-pass alpha; higher follows VLA targets more directly'
    )

    controller_id_arg = DeclareLaunchArgument(
        'controller_id',
        default_value='FollowPath',
        description='Nav2 controller plugin ID'
    )

    goal_checker_id_arg = DeclareLaunchArgument(
        'goal_checker_id',
        default_value='general_goal_checker',
        description='Nav2 goal checker ID'
    )

    use_pose_tracker = IfCondition(PythonExpression([
        "'", LaunchConfiguration('base_control_mode'), "' == 'pose_tracker'"
    ]))
    use_follow_path = IfCondition(PythonExpression([
        "'", LaunchConfiguration('base_control_mode'), "' == 'follow_path'"
    ]))

    # Rewrite nav2_params to use runtime parameters
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'controller_frequency': LaunchConfiguration('controller_frequency'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'lio_odom_topic': LaunchConfiguration('odom_topic'),
        },
        convert_types=True,
    )

    # Nav2 Controller Server
    # This provides the FollowPath action that VLA bridge needs
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('cmd_vel', '/cmd_vel_raw'),
        ],
        condition=use_follow_path,
    )

    # Replay command conditioner:
    # - Keeps component deadbands and pure-rotation cleanup.
    # - Does not lock "straight" motion, because VLA datasets may contain
    #   deliberate diagonal / lateral swerve motion.
    # - Keeps these replay-only constraints outside the learned dataset.
    replay_cmd_conditioner = Node(
        package='swerve_navigation',
        executable='cmd_vel_replay_conditioner.py',
        name='cmd_vel_replay_conditioner',
        output='screen',
        parameters=[{
            'input_cmd_topic': '/cmd_vel_raw',
            'output_cmd_topic': '/cmd_vel_conditioned',
            'publish_rate': 50.0,
            'cmd_timeout': 0.35,
            'linear_x_deadband': 0.003,
            'linear_y_deadband': 0.003,
            'angular_z_deadband': 0.008,
            'straight_lock_enabled': False,
            'straight_enter_min_abs_vx': 0.08,
            'straight_enter_max_abs_vy': 0.035,
            'straight_enter_max_abs_wz': 0.04,
            'straight_exit_min_abs_vx': 0.05,
            'straight_exit_max_abs_vy': 0.055,
            'straight_exit_max_abs_wz': 0.065,
            'rotate_lock_enabled': True,
            'rotate_enter_min_abs_wz': 0.08,
            'rotate_enter_max_linear_speed': 0.03,
            'rotate_exit_min_abs_wz': 0.055,
            'rotate_exit_max_linear_speed': 0.05,
            'ema_alpha': 1.0,
        }],
    )

    # Smooth the conditioned replay velocity before it reaches the hardware
    # controller. This mirrors the navigation stack's smoother layer while
    # keeping VLA replay-specific deadbands in the conditioner above.
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('cmd_vel', '/cmd_vel_conditioned'),
            ('cmd_vel_smoothed', '/cmd_vel'),
            ('odom', LaunchConfiguration('odom_topic')),
        ],
    )

    # Lifecycle manager for the smoother. It is needed in both pose-tracker and
    # legacy FollowPath modes because velocity_smoother is a lifecycle node.
    smoother_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_vla_smoother',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['velocity_smoother'],
        }],
    )

    # Lifecycle manager for the legacy Nav2 controller_server.
    controller_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_vla_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['controller_server'],
        }],
        condition=use_follow_path,
    )

    # Closed-loop controller for pose-mode policies. It consumes the single
    # future-odom pose published on /vla/base_path and outputs /cmd_vel_raw.
    pose_tracker = Node(
        package='swerve_navigation',
        executable='future_odom_pose_tracker.py',
        name='future_odom_pose_tracker',
        output='screen',
        parameters=[{
            'input_path_topic': LaunchConfiguration('vla_path_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'output_cmd_topic': '/cmd_vel_raw',
            'control_rate': 50.0,
            'target_timeout': LaunchConfiguration('pose_tracker_target_timeout'),
            'odom_timeout': 0.5,
            'strict_frame_check': True,
            'kx': LaunchConfiguration('pose_tracker_kx'),
            'ky': LaunchConfiguration('pose_tracker_ky'),
            'k_yaw': LaunchConfiguration('pose_tracker_k_yaw'),
            'max_vx': 0.22,
            'max_vy': 0.10,
            'max_wz': LaunchConfiguration('pose_tracker_max_wz'),
            'max_ax': 0.45,
            'max_ay': 0.35,
            'max_awz': 0.5,
            'xy_deadband': LaunchConfiguration('pose_tracker_xy_deadband'),
            'yaw_deadband': LaunchConfiguration('pose_tracker_yaw_deadband'),
            'target_filter_alpha': LaunchConfiguration('pose_tracker_target_filter_alpha'),
            'target_jump_limit_xy': LaunchConfiguration('pose_tracker_jump_limit_xy'),
            'target_jump_limit_yaw': LaunchConfiguration('pose_tracker_jump_limit_yaw'),
            'diag_log_period': 1.0,
        }],
        condition=use_pose_tracker,
    )

    # Legacy VLA Path Bridge
    # Converts /vla/base_path to Nav2 FollowPath action goals.
    vla_bridge = Node(
        package='swerve_navigation',
        executable='vla_path_to_nav2_follow_path.py',
        name='vla_path_to_nav2_follow_path',
        output='screen',
        parameters=[{
            'input_path_topic': LaunchConfiguration('vla_path_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'follow_path_action_name': LaunchConfiguration('follow_path_action'),
            'cmd_vel_topic': '/cmd_vel_raw',
            'controller_id': LaunchConfiguration('controller_id'),
            'goal_checker_id': LaunchConfiguration('goal_checker_id'),
            'strict_frame_check': True,
            'path_timeout': 1.0,  # Keep previous replay behavior; tolerate brief path gaps.
            'min_goal_update_period': 0.1,  # Keep short VLA horizons responsive.
            'stop_publish_count': 20,
            'rotation_guard_enabled': True,
            'rotation_xy_span_threshold': 0.06,  # Relaxed for model inference
            'rotation_min_yaw_delta': 0.18,  # Relaxed
            'rotation_release_yaw_error': 0.15,  # Relaxed
            'rotation_release_angular_speed': 0.15,  # Relaxed
            'rotation_release_stable_samples': 2,
            'rotation_hold_path_points': 8,
            'rotation_hold_resend_period': 0.35,
            'rotation_max_hold_duration': 0.8,  # Faster timeout
            'rotation_timeout_release_yaw_error': 0.25,  # More tolerant
            # When VLA predicts a stationary path and the robot is already
            # close enough, cancel FollowPath and hold zero cmd_vel. This
            # prevents centimeter-level odom noise from causing long terminal
            # adjustment motion.
            'static_lock_enabled': False,
            'static_lock_xy_tolerance': 0.08,
            'static_lock_yaw_tolerance': 0.10,
            'static_lock_entry_xy_tolerance': 0.12,
            'static_lock_entry_yaw_tolerance': 0.15,
            'static_lock_release_xy_tolerance': 0.12,
            'static_lock_release_yaw_tolerance': 0.15,
            'static_lock_linear_speed': 0.08,
            'static_lock_angular_speed': 0.10,
            'static_lock_stable_samples': 2,
        }],
        condition=use_follow_path,
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        controller_frequency_arg,
        odom_topic_arg,
        vla_path_topic_arg,
        base_control_mode_arg,
        follow_path_action_arg,
        controller_id_arg,
        goal_checker_id_arg,
        pose_tracker_target_timeout_arg,
        pose_tracker_kx_arg,
        pose_tracker_ky_arg,
        pose_tracker_k_yaw_arg,
        pose_tracker_max_wz_arg,
        pose_tracker_jump_limit_xy_arg,
        pose_tracker_jump_limit_yaw_arg,
        pose_tracker_xy_deadband_arg,
        pose_tracker_yaw_deadband_arg,
        pose_tracker_target_filter_alpha_arg,

        # Nodes
        controller_server,
        replay_cmd_conditioner,
        velocity_smoother,
        smoother_lifecycle_manager,
        controller_lifecycle_manager,
        pose_tracker,
        vla_bridge,
    ])
