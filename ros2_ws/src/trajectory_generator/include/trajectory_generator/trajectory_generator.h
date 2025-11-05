/**
 * @file trajectory_generator.h
 *
 * @brief Header file for the Trajectory Generator node.
 *
 * This node subscribes to a smoothed path topic, generates a time-parameterized
 * trajectory based on a constant velocity model, and sends the trajectory to
 * a trajectory-following action server.
 *
 */
#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_controller/action/follow_path.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace trajectory_generator {

  class TrajectoryGenerator: public rclcpp::Node {
public:
    using FollowPath = trajectory_controller::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle < FollowPath >;

    TrajectoryGenerator();
    ~TrajectoryGenerator();

protected:
  // ROS2 Interfaces
    rclcpp::Subscription < nav_msgs::msg::Path > ::SharedPtr subscription_;
    rclcpp_action::Client < FollowPath > ::SharedPtr action_client_;

  // Parameters
    double v_const_;
    std::string frame_id_;

  // Internal methods
    void generateTrajectory(const nav_msgs::msg::Path::ConstSharedPtr msg);
    void sendTrajectoryGoal(const nav_msgs::msg::Path & traj);

  // Callbacks for action
    void goalResponseCallback(
      std::shared_future < typename GoalHandleFollowPath::SharedPtr > future);
    void
    feedbackCallback(
      GoalHandleFollowPath::SharedPtr,
      const std::shared_ptr < const FollowPath::Feedback > feedback);
    void resultCallback(const GoalHandleFollowPath::WrappedResult & result);
  };

} // namespace trajectory_generator

#endif // TRAJECTORY_GENERATOR_H_
