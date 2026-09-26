#!/usr/bin/env python3
"""Launch VLA base path bridge into Nav2 FollowPath."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_path_topic_arg = DeclareLaunchArgument(
        "input_path_topic",
        default_value="/vla/base_path",
        description="VLA-predicted base path topic",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/fastlio2/lio_odom",
        description="Odometry topic used to prepend current pose",
    )
    follow_path_action_name_arg = DeclareLaunchArgument(
        "follow_path_action_name",
        default_value="/follow_path",
        description="Nav2 controller_server FollowPath action name",
    )
    controller_id_arg = DeclareLaunchArgument(
        "controller_id",
        default_value="FollowPath",
        description="Nav2 controller plugin id from nav2_params.yaml",
    )
    goal_checker_id_arg = DeclareLaunchArgument(
        "goal_checker_id",
        default_value="general_goal_checker",
        description="Nav2 goal checker id from nav2_params.yaml",
    )
    strict_frame_check_arg = DeclareLaunchArgument(
        "strict_frame_check",
        default_value="true",
        description="Drop VLA paths whose frame does not match the configured odom topic",
    )

    bridge = Node(
        package="swerve_navigation",
        executable="vla_path_to_nav2_follow_path.py",
        name="vla_path_to_nav2_follow_path",
        output="screen",
        parameters=[{
            "input_path_topic": LaunchConfiguration("input_path_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "follow_path_action_name": LaunchConfiguration("follow_path_action_name"),
            "controller_id": LaunchConfiguration("controller_id"),
            "goal_checker_id": LaunchConfiguration("goal_checker_id"),
            "strict_frame_check": LaunchConfiguration("strict_frame_check"),
        }],
    )

    return LaunchDescription([
        input_path_topic_arg,
        odom_topic_arg,
        follow_path_action_name_arg,
        controller_id_arg,
        goal_checker_id_arg,
        strict_frame_check_arg,
        bridge,
    ])
