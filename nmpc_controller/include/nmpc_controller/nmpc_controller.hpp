#ifndef NMPC_CONTROLLER__NMPC_CONTROLLER_HPP_
#define NMPC_CONTROLLER__NMPC_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "nmpc_controller/acados_wrapper.hpp"

namespace nmpc_controller
{

class NmpcController : public nav2_core::Controller
{
public:
  NmpcController() = default;
  ~NmpcController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // Prune global plan (in map frame) around robot position and return
  // the robot pose transformed into the map frame.
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    geometry_msgs::msg::PoseStamped & pose_in_map);

  // Sample N+1 reference points along the path
  void sampleReferences(
    const nav_msgs::msg::Path & local_path,
    const geometry_msgs::msg::PoseStamped & pose,
    double desired_vel);
  void sampleVlaReplayReferences(
    const nav_msgs::msg::Path & local_path,
    const geometry_msgs::msg::PoseStamped & pose);

  // Query costmap cost at a position in map frame, normalized to [0,1]
  double getCostmapCost(double map_x, double map_y) const;

  // Seed the solver with a feasible trajectory close to the references.
  void seedSolverWarmStart(const StateVec & x0);

  // Publish predicted trajectory for visualization
  void publishPredictedPath();

  // A* local avoidance: detect obstacles on local_path, plan detour, splice back
  nav_msgs::msg::Path localAstarAvoidance(
    const nav_msgs::msg::Path & local_path,
    const geometry_msgs::msg::PoseStamped & pose_in_map);
  std::vector<std::pair<double, double>> runAstarOnCostmap(
    double start_x, double start_y,
    double goal_x, double goal_y);
  void smoothAstarPath(std::vector<std::pair<double, double>> & path);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("NmpcController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  nav_msgs::msg::Path global_plan_;

  std::unique_ptr<AcadosWrapper> solver_;

  // Predicted trajectory publisher
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;

  // A* detour path publisher for RViz visualization
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr astar_detour_pub_;

  // Reference storage
  std::vector<StateVec> stage_refs_;

  // Parameters
  int horizon_steps_{20};
  double horizon_time_{2.0};

  // Cost weights
  std::array<double, NY> W_diag_{};
  std::array<double, NY_E> W_e_diag_{};

  // Velocity/acceleration limits
  double vx_max_{0.5};
  double vx_min_{-0.3};
  double vy_max_{0.5};
  double omega_max_{1.0};
  double ax_max_{2.0};
  double ay_max_{2.0};
  double alpha_max_{3.0};

  double desired_linear_vel_{0.4};
  double desired_linear_vel_base_{0.4};  // original value from config, used by setSpeedLimit
  double approach_velocity_scaling_dist_{0.5};
  double goal_heading_align_ratio_{0.7};
  double goal_heading_align_dist_{0.8};
  double start_heading_capture_dist_{0.8};
  double start_speed_min_scale_{0.4};
  double prune_search_distance_{3.0};
  double same_goal_replan_attach_dist_{0.18};
  double reference_heading_smoothing_gain_{0.45};
  double reference_omega_smoothing_gain_{0.55};
  double reference_omega_rate_limit_scale_{0.65};
  double curve_speed_reduction_gain_{0.35};
  double curve_speed_min_scale_{0.35};
  double curve_switch_yaw_rate_threshold_{0.35};
  double curve_switch_speed_scale_{0.45};
  double transform_tolerance_{0.1};
  bool allow_lateral_tracking_{true};
  bool swerve_native_mode_{false};
  bool visualize_{true};
  bool vla_replay_mode_{false};
  double vla_replay_point_dt_{0.2};

  // A* local avoidance parameters
  bool local_astar_enabled_{true};
  double local_astar_cost_threshold_{0.40};
  double local_astar_reconnect_dist_{1.5};
  double local_astar_obstacle_lookahead_{2.5};
  double local_astar_inflation_cost_{0.30};
  int local_astar_smooth_iterations_{3};
  double local_astar_smooth_weight_{0.3};

  // In-place rotation before driving
  double rotate_to_heading_threshold_{0.8};            // rad: enter rotate-to-heading mode above this error
  double rotate_to_heading_release_threshold_{0.2};    // rad: stay rotating until the heading error shrinks below this
  double rotate_to_heading_angular_vel_{0.8};          // rad/s: maximum angular velocity for in-place rotation
  double rotate_to_heading_min_angular_vel_{0.2};      // rad/s: minimum angular velocity to avoid stalling near alignment
  double rotate_to_heading_gain_{1.0};                 // proportional gain from heading error to angular velocity
  double rotate_to_heading_goal_reset_dist_{0.25};     // m: new goal distance needed to re-arm initial rotate-to-heading
  double rotate_to_heading_goal_reset_angle_{0.35};    // rad: new goal yaw change needed to re-arm initial rotate-to-heading
  bool rotating_to_heading_{false};
  bool initial_heading_alignment_complete_{false};

  // Final heading alignment at goal: rotate in place after reaching position
  bool final_heading_alignment_active_{false};
  bool final_heading_done_{false};                     // true after final rotation completes — output zero until goal_checker confirms
  double final_rotate_xy_tolerance_{0.30};             // m: position threshold to enter final rotation
  double final_rotate_xy_release_tolerance_{0.36};     // m: once rotating, allow a wider xy band before giving control back to NMPC
  double final_rotate_yaw_threshold_{0.24};            // rad: after completion, yaw drift above this re-enters final in-place rotation
  double final_rotate_yaw_goal_tolerance_{0.15};       // rad: rotate until yaw error drops below this (must match goal_checker yaw_goal_tolerance)
  geometry_msgs::msg::PoseStamped current_goal_pose_;
  bool current_goal_valid_{false};

  // Failure tracking
  int consecutive_failures_{0};
  size_t last_pruned_plan_index_{0};
  rclcpp::Time last_same_goal_replan_accept_stamp_{0, 0, RCL_ROS_TIME};
  bool last_same_goal_replan_accept_valid_{false};
  bool last_same_goal_replan_was_smoothed_obstacle_handoff_{false};

  // Cached robot pose in map frame for warm-starting prune index on replan
  geometry_msgs::msg::Point last_robot_position_map_;
  bool last_robot_position_valid_{false};

  // Startup heading capture in native mode is based on real translation after
  // the initial rotate-to-heading release, not prune-index progress.
  geometry_msgs::msg::Point startup_heading_capture_origin_map_;
  bool startup_heading_capture_origin_valid_{false};
  double startup_heading_capture_progress_{0.0};

  // FAST-LIO2 odometry subscription for velocity consistent with pose source
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub_;
  std::mutex lio_odom_mutex_;
  geometry_msgs::msg::Twist lio_velocity_;
  bool lio_velocity_valid_{false};
  rclcpp::Time lio_velocity_stamp_{0, 0, RCL_ROS_TIME};
  std::string lio_odom_topic_{"/fastlio2/lio_odom"};
  double lio_odom_timeout_{0.5};

  // Cached map→odom transform for costmap queries
  geometry_msgs::msg::TransformStamped map_to_odom_tf_;
  bool map_to_odom_valid_{false};

  static constexpr int MAX_CONSECUTIVE_FAILURES = 5;
};

}  // namespace nmpc_controller

#endif  // NMPC_CONTROLLER__NMPC_CONTROLLER_HPP_
