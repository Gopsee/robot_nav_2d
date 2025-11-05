/**
 * @file trajectory_controller.h
 * 
 * @brief Header file for the TrajectoryControllerNode class.
 * This class implements a ROS2 node that controls a robot to follow a given trajectory
 * using a lookahead-based control strategy.
 */
#ifndef TRAJECTORY_CONTROLLER_H_
#define TRAJECTORY_CONTROLLER_H_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "trajectory_controller/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cmath>

namespace trajectory_controller
{

  class TrajectoryControllerNode: public rclcpp::Node
  {
public:
    // Type aliases for action
    using FollowPath = trajectory_controller::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle < FollowPath >;

    TrajectoryControllerNode();
    ~TrajectoryControllerNode();

protected:
  // ROS2 interfaces - subscriptions, publishers, action server
    rclcpp::Subscription < nav_msgs::msg::Odometry > ::SharedPtr odom_subscription_;
    rclcpp::Publisher < geometry_msgs::msg::Twist > ::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher < visualization_msgs::msg::MarkerArray > ::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr metrics_pub_;

    rclcpp_action::Server < FollowPath > ::SharedPtr action_server_;

  // Internal state
    nav_msgs::msg::Path::SharedPtr trajectory_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    double lookahead_dist_ {1.0};
    double linear_speed_ {0.5};

  // Action callbacks
    rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr < const FollowPath::Goal > goal);

    rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr < GoalHandleFollowPath > goal_handle);

    void handleAccepted(const std::shared_ptr < GoalHandleFollowPath > goal_handle);
    void execute(const std::shared_ptr < GoalHandleFollowPath > goal_handle);

  // Helper functions
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    geometry_msgs::msg::PoseStamped findLookaheadPoint(double x, double y, double Ld);

    void publishDebugMarkers(
      double robot_x, double robot_y, double robot_yaw,
      const geometry_msgs::msg::PoseStamped & lookahead,
      double curvature);

    // Performance statistics  
    struct PerformanceStats {
    double total_cte = 0.0;
    double max_cte = 0.0;
    double total_heading_error = 0.0;
    size_t samples = 0;
    rclcpp::Time start_time;
    };

    PerformanceStats perf_stats_;

  };

}  // namespace trajectory_controller

#endif  // TRAJECTORY_CONTROLLER_H_
