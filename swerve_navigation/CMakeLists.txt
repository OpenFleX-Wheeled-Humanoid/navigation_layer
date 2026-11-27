cmake_minimum_required(VERSION 3.8)
project(swerve_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz scripts behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/pointcloud_to_livox.py
    scripts/livox_to_pointcloud2.py
    scripts/record_gps_datum.py
    scripts/gps_initial_pose.py
    scripts/plan_only_goal.py
    scripts/pointcloud_qos_relay.py
    scripts/laser_scan_qos_relay.py
    scripts/local_window_marker.py
    scripts/safe_odom_mux.py
    scripts/cmd_vel_safety_gate.py
    scripts/cmd_vel_replay_conditioner.py
    scripts/future_odom_pose_tracker.py
    scripts/vla_path_to_nav2_follow_path.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
