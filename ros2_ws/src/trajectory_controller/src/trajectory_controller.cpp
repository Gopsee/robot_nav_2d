/**
 * trajectory_controller.cpp
 *
 * @brief Implementation file for the TrajectoryControllerNode class.
 * This class implements a ROS2 node that controls a robot to follow a given
 * trajectory using a lookahead-based control strategy.
 *
 * The node operates as an **action server** that listens for
 * trajectory-following goals. When a new goal (path) is received, it
 * continuously tracks the robot’s current pose from odometry, computes a
 * lookahead point ahead of the robot on the trajectory, and generates
 * appropriate velocity commands (`cmd_vel`) to steer toward that point.
 *
 * Visualization markers can be published to RViz to help debug or visualize
 * the robot’s current state, target point, and path-following behavior.
 */

#include "trajectory_controller/trajectory_controller.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <thread>

namespace trajectory_controller {

using namespace std::chrono_literals;

TrajectoryControllerNode::TrajectoryControllerNode()
    : Node("trajectory_controller_node") {
  RCLCPP_INFO(this->get_logger(), "Trajectory Controller Node started.");

  // Parameters
  this->declare_parameter<double>("lookahead_dist", 1.0);
  this->declare_parameter<double>("linear_speed", 0.5);

  // Subscribers and publishers
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&TrajectoryControllerNode::odomCallback, this,
                std::placeholders::_1));

  cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "pure_pursuit_debug", 10);
    
  metrics_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "tracking_metrics", 10);


  // Action Server
  action_server_ = rclcpp_action::create_server<FollowPath>(
      this, "follow_path",
      std::bind(&TrajectoryControllerNode::handleGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrajectoryControllerNode::handleCancel, this,
                std::placeholders::_1),
      std::bind(&TrajectoryControllerNode::handleAccepted, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Action server 'follow_path' initialized.");
}

TrajectoryControllerNode::~TrajectoryControllerNode() {
  RCLCPP_INFO(this->get_logger(), "Trajectory Controller Node shutting down.");
}

// Action Server Handlers
rclcpp_action::GoalResponse TrajectoryControllerNode::handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowPath::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received FollowPath goal with %zu points",
              goal->path.poses.size());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryControllerNode::handleCancel(
    const std::shared_ptr<GoalHandleFollowPath>) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryControllerNode::handleAccepted(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
  std::thread{std::bind(&TrajectoryControllerNode::execute, this,
                        std::placeholders::_1),
              goal_handle}
      .detach();
}

/**
 * @brief Executes the path following routine.
 *
 * This method runs in a separate thread after a goal is accepted.
 * It continuously reads the robot's current pose from odometry,
 * computes the control commands using a lookahead point, and publishes them
 * until the goal is completed or canceled.
 *
 * @param goal_handle Shared pointer to the goal handle.
 */
void TrajectoryControllerNode::execute(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<FollowPath::Feedback>();
  auto result = std::make_shared<FollowPath::Result>();

  trajectory_ = std::make_shared<nav_msgs::msg::Path>(goal->path);
  RCLCPP_INFO(this->get_logger(), "Executing path with %zu points",
              trajectory_->poses.size());

  lookahead_dist_ = this->get_parameter("lookahead_dist").as_double();
  linear_speed_ = this->get_parameter("linear_speed").as_double();

  rclcpp::Rate rate(10);
  size_t progress_index = 0;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      geometry_msgs::msg::Twist stop_cmd;
      cmd_vel_publisher_->publish(stop_cmd);
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_WARN(this->get_logger(), "Goal canceled.");
      return;
    }

    if (!current_odom_) {
      rate.sleep();
      continue;
    }

    // Get robot pose
    double x = current_odom_->pose.pose.position.x;
    double y = current_odom_->pose.pose.position.y;

    tf2::Quaternion q(current_odom_->pose.pose.orientation.x,
                      current_odom_->pose.pose.orientation.y,
                      current_odom_->pose.pose.orientation.z,
                      current_odom_->pose.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Compute target
    geometry_msgs::msg::PoseStamped target =
        findLookaheadPoint(x, y, lookahead_dist_);
    double dx = target.pose.position.x - x;
    double dy = target.pose.position.y - y;

    double y_r = -sin(yaw) * dx + cos(yaw) * dy;
    double L = std::hypot(dx, dy);
    if (L < 1e-3) {
      L = 1e-3;
    } // Prevent division by zero

    double curvature = (2.0 * y_r) / (L * L);
    double omega = linear_speed_ * curvature;

    // Check distance to goal
    double goal_dist =
        std::hypot(trajectory_->poses.back().pose.position.x - x,
                   trajectory_->poses.back().pose.position.y - y);

    if (goal_dist < 0.2) {
      geometry_msgs::msg::Twist stop_cmd;
      cmd_vel_publisher_->publish(stop_cmd);
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
      return;
    }

    geometry_msgs::msg::Twist cmd;

    // --- Angular clamp ---
    omega = std::clamp(omega, -1.5, 1.5); // [rad/s]

    // --- Adaptive linear speed ---
    double curvature_abs = std::abs(curvature);
    double speed_scale =
        1.0 / (1.0 + 3.0 * curvature_abs); // reduce on sharp turns
    cmd.linear.x = linear_speed_ * speed_scale;
    cmd.angular.z = omega;

    cmd.linear.x = linear_speed_;
    cmd.angular.z = omega;
    cmd_vel_publisher_->publish(cmd);

    publishDebugMarkers(x, y, yaw, target, curvature);

    // Cross-track error
    double cte = std::hypot(x - dx, y - dy);
    perf_stats_.total_cte += cte;
    perf_stats_.max_cte = std::max(perf_stats_.max_cte, cte);

    // --- Publish CTE ---
    std_msgs::msg::Float64 cte_msg;
    cte_msg.data = cte;
    metrics_pub_->publish(cte_msg);

    // Send feedback
    feedback->progress =
        static_cast<float>(progress_index++) / trajectory_->poses.size();
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }
}

/**
 * @brief Callback function for odometry updates.
 *
 * Stores the latest odometry data, which includes the robot’s current position
 * and orientation.
 *
 * @param msg Shared pointer to the received odometry message.
 */
void TrajectoryControllerNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = msg;
}

/**
 * @brief Finds the lookahead point on the path.
 *
 * Given the robot’s current position `(x, y)` and a desired lookahead distance
 * `Ld`, this function searches the current trajectory to find the next point at
 * approximately that distance ahead of the robot.
 *
 * @param x Current robot x-position.
 * @param y Current robot y-position.
 * @param Ld Lookahead distance (in meters).
 * @return The pose of the selected lookahead point on the trajectory.
 */
geometry_msgs::msg::PoseStamped
TrajectoryControllerNode::findLookaheadPoint(double x, double y, double Ld) {
  geometry_msgs::msg::PoseStamped target = trajectory_->poses.back();
  if (!trajectory_ || trajectory_->poses.empty()) {
    return target;
  }

  // Find nearest point index on the path
  size_t nearest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < trajectory_->poses.size(); ++i) {
    double px = trajectory_->poses[i].pose.position.x;
    double py = trajectory_->poses[i].pose.position.y;
    double dist = std::hypot(px - x, py - y);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  // Now search forward from nearest point to find the lookahead
  for (size_t i = nearest_idx; i + 1 < trajectory_->poses.size(); ++i) {
    const auto &p1 = trajectory_->poses[i].pose.position;
    const auto &p2 = trajectory_->poses[i + 1].pose.position;

    // Vector from p1->p2 and p1->robot
    double path_dx = p2.x - p1.x;
    double path_dy = p2.y - p1.y;
    double robot_dx = x - p1.x;
    double robot_dy = y - p1.y;

    // Dot product to check if robot is ahead of p1 along path direction
    double dot = path_dx * robot_dx + path_dy * robot_dy;
    if (dot > (path_dx * path_dx + path_dy * path_dy)) {
      // Robot is beyond this segment, skip
      continue;
    }

    // Distance from robot to p2
    double dist = std::hypot(p2.x - x, p2.y - y);
    if (dist >= Ld) {
      // Interpolate exact lookahead point between p1 and p2
      double seg_len = std::hypot(path_dx, path_dy);
      double t = (Ld - std::hypot(p1.x - x, p1.y - y)) / seg_len;
      t = std::clamp(t, 0.0, 1.0);

      target.header = trajectory_->header;
      target.pose.position.x = p1.x + t * path_dx;
      target.pose.position.y = p1.y + t * path_dy;
      target.pose.position.z = 0.0;
      target.pose.orientation = trajectory_->poses[i].pose.orientation;
      return target;
    }
  }

  // Fallback: last pose
  return trajectory_->poses.back();
}

/**
 * @brief Publishes RViz markers for debugging and visualization.
 *
 * Displays the robot’s current pose, lookahead point, and curvature direction
 * as colored markers in RViz.
 *
 * @param robot_x Current robot x position.
 * @param robot_y Current robot y position.
 * @param robot_yaw Current robot heading (in radians).
 * @param lookahead The current lookahead point on the path.
 * @param curvature Computed curvature of the trajectory toward the lookahead
 * point.
 */
void TrajectoryControllerNode::publishDebugMarkers(
    double robot_x, double robot_y, double robot_yaw,
    const geometry_msgs::msg::PoseStamped &lookahead, double curvature) {
  visualization_msgs::msg::MarkerArray array;
  rclcpp::Time now = this->get_clock()->now();

  // --- Lookahead point marker ---
  visualization_msgs::msg::Marker lookahead_marker;
  lookahead_marker.header.frame_id = trajectory_->header.frame_id;
  lookahead_marker.header.stamp = now;
  lookahead_marker.ns = "lookahead_point";
  lookahead_marker.id = 0;
  lookahead_marker.type = visualization_msgs::msg::Marker::SPHERE;
  lookahead_marker.action = visualization_msgs::msg::Marker::ADD;
  lookahead_marker.pose = lookahead.pose;
  lookahead_marker.scale.x = 0.2;
  lookahead_marker.scale.y = 0.2;
  lookahead_marker.scale.z = 0.2;
  lookahead_marker.color.r = 0.1f;
  lookahead_marker.color.g = 1.0f;
  lookahead_marker.color.b = 0.1f;
  lookahead_marker.color.a = 1.0f;
  array.markers.push_back(lookahead_marker);

  // --- Line from robot to lookahead ---
  visualization_msgs::msg::Marker line;
  line.header = lookahead_marker.header;
  line.ns = "line_to_lookahead";
  line.id = 1;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.05;
  line.color.r = 0.0;
  line.color.g = 0.5;
  line.color.b = 1.0;
  line.color.a = 1.0;
  geometry_msgs::msg::Point pr, pl;
  pr.x = robot_x;
  pr.y = robot_y;
  pl = lookahead.pose.position;
  line.points.push_back(pr);
  line.points.push_back(pl);
  array.markers.push_back(line);

  // --- Curvature circle marker ---
  if (std::abs(curvature) > 1e-5) {
    double radius = 1.0 / curvature;
    visualization_msgs::msg::Marker circle;
    circle.header = lookahead_marker.header;
    circle.ns = "curvature_circle";
    circle.id = 2;
    circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
    circle.action = visualization_msgs::msg::Marker::ADD;
    circle.scale.x = 0.03;
    circle.color.r = 1.0;
    circle.color.g = 0.0;
    circle.color.b = 0.0;
    circle.color.a = 0.8;

    // center of curvature
    geometry_msgs::msg::Point center;
    center.x = robot_x - radius * sin(robot_yaw);
    center.y = robot_y + radius * cos(robot_yaw);

    const int N = 40;
    for (int i = 0; i <= N; ++i) {
      double theta = (2 * M_PI * i) / N;
      geometry_msgs::msg::Point p;
      p.x = center.x + radius * cos(theta);
      p.y = center.y + radius * sin(theta);
      p.z = 0.05;
      circle.points.push_back(p);
    }
    array.markers.push_back(circle);
  }

  marker_pub_->publish(array);
}

} // namespace trajectory_controller
