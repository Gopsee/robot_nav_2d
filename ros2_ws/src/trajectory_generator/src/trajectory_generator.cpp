/**
 * @file trajectory_generator.cpp
 *
 * @brief Implementation of the Trajectory Generator node.
 *
 * This node listens to a smoothed path topic (`smooth_path`), converts it into
 * a time-parameterized trajectory based on a constant velocity model, and sends
 * the resulting trajectory to a trajectory-following action server (`follow_path`).
 *
 */
#include "trajectory_generator/trajectory_generator.h"

using namespace std::chrono_literals;

namespace trajectory_generator
{

TrajectoryGenerator::TrajectoryGenerator()
: Node("trajectory_generator_node")
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator node has started.");

  // ROS2 Subscription
  subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      "smooth_path", 10,
      std::bind(&TrajectoryGenerator::generateTrajectory, this,
                std::placeholders::_1));

  // Create Action Client
  action_client_ =
    rclcpp_action::create_client<FollowPath>(this, "follow_path");

  // Parameters
  this->declare_parameter<double>("v_const", 0.5);
  this->declare_parameter<std::string>("frame_id", "map");
  v_const_ = this->get_parameter("v_const").as_double();
  frame_id_ = this->get_parameter("frame_id").as_string();

  RCLCPP_INFO(this->get_logger(),
              "TrajectoryGenerator initialized with v_const=%.2f, frame_id=%s",
              v_const_, frame_id_.c_str());
}

TrajectoryGenerator::~TrajectoryGenerator()
{
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator node is shutting down.");
}

/**
 * @brief Callback that generates a time-parameterized trajectory from an input path.
 *
 * The trajectory is generated assuming a constant velocity model.
 * Once generated, the trajectory is sent to the action server.
 *
 * @param msg Shared pointer to the received smoothed path.
 */
void TrajectoryGenerator::generateTrajectory(
  const nav_msgs::msg::Path::ConstSharedPtr msg)
{
  if (msg->poses.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Path too short to generate trajectory.");
    return;
  }

  v_const_ = this->get_parameter("v_const").as_double();

  std::vector<double> x, y, s, t;
  x.reserve(msg->poses.size());
  y.reserve(msg->poses.size());

  double cumulative_dist = 0.0;
  s.push_back(0.0);

  // Extract points
  for (size_t i = 0; i < msg->poses.size(); ++i) {
    x.push_back(msg->poses[i].pose.position.x);
    y.push_back(msg->poses[i].pose.position.y);

    if (i > 0) {
      double dx = x[i] - x[i - 1];
      double dy = y[i] - y[i - 1];
      cumulative_dist += std::sqrt(dx * dx + dy * dy);
      s.push_back(cumulative_dist);
    }
  }

  // Compute timestamps (constant velocity)
  double t_curr = 0.0;
  t.push_back(t_curr);
  for (size_t i = 1; i < s.size(); ++i) {
    double ds = s[i] - s[i - 1];
    double dt = ds / v_const_;
    t_curr += dt;
    t.push_back(t_curr);
  }

  // Build trajectory message
  nav_msgs::msg::Path traj;
  traj.header.frame_id = frame_id_;
  traj.header.stamp = this->now();

  for (size_t i = 0; i < x.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = traj.header;
    pose.pose.position.x = x[i];
    pose.pose.position.y = y[i];
    pose.pose.orientation.w = 1.0;
    traj.poses.push_back(pose);
  }

  RCLCPP_INFO(this->get_logger(),
              "Generated trajectory with %zu points, total length = %.2f m, "
              "velocity = %.2f m/s",
              traj.poses.size(), s.back(), v_const_);

  // Send to action server
  sendTrajectoryGoal(traj);
}

/**
 * @brief Sends the generated trajectory as an action goal to the FollowPath action server.
 *
 * The method waits for the action server to be available before sending the goal.
 * It also sets up callbacks to handle goal response, feedback, and result.
 * @param traj The generated trajectory as a nav_msgs::Path message.
 */
void TrajectoryGenerator::sendTrajectoryGoal(const nav_msgs::msg::Path & traj)
{
  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server 'follow_path' not available after waiting.");
    return;
  }

  auto goal_msg = FollowPath::Goal();
  goal_msg.path = traj;

  RCLCPP_INFO(this->get_logger(),
              "Sending trajectory goal to action server...");

  rclcpp_action::Client<FollowPath>::SendGoalOptions options;
  options.goal_response_callback =
    [this](std::shared_ptr<GoalHandleFollowPath> handle) {
      if (!handle) {
        RCLCPP_ERROR(this->get_logger(),
                       "Goal was rejected by the action server.");
      } else {
        RCLCPP_INFO(this->get_logger(),
                      "Goal accepted by the action server.");
      }
    };

  options.feedback_callback =
    [this](GoalHandleFollowPath::SharedPtr,
    const std::shared_ptr<const FollowPath::Feedback> feedback) {
      RCLCPP_INFO(this->get_logger(), "Feedback: progress = %.2f%%",
                    feedback->progress * 100.0);
    };

  options.result_callback =
    [this](const GoalHandleFollowPath::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Trajectory execution succeeded!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(this->get_logger(), "Trajectory execution aborted.");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "Trajectory execution canceled.");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
          break;
      }
    };

  // Send the goal asynchronously
  action_client_->async_send_goal(goal_msg, options);
}

void TrajectoryGenerator::goalResponseCallback(
  std::shared_future<GoalHandleFollowPath::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal rejected by server.");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, executing trajectory...");
  }
}

void TrajectoryGenerator::feedbackCallback(
  GoalHandleFollowPath::SharedPtr,
  const std::shared_ptr<const FollowPath::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Feedback received: progress = %.2f%%",
              feedback->progress * 100.0);
}

void TrajectoryGenerator::resultCallback(
  const GoalHandleFollowPath::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Trajectory execution succeeded!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(this->get_logger(), "Trajectory execution aborted.");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Trajectory execution canceled.");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
      break;
  }
}

} // namespace trajectory_generator
