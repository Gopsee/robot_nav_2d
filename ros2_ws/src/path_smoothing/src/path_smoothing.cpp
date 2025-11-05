/**
 * @file path_smoothing.cpp
 * @brief Implementation of the PathSmoothing node for smoothing discrete
 * waypoints into a smooth, continuous path.
 *
 * @author: Gowtham Pagadala
 *
 * This node subscribes to an input ROS2 topic (`input_path`) of type
 * `nav_msgs::msg::Path`, applies cubic B-spline smoothing to reduce sharp turns
 * and discontinuities, and publishes the resulting smooth path on the topic
 * `smooth_path`.
 *
 */
#include "path_smoothing/path_smoothing.h"

namespace path_smoothing
{
PathSmoothing::PathSmoothing()
: Node("path_smoothing_node")
{

  // Publisher and Subscriber Initialization
  pub_smooth_path_ =
    this->create_publisher<nav_msgs::msg::Path>("smooth_path", 10);

  subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      "input_path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received path with %zu poses.",
                    msg->poses.size());
      std::vector<geometry_msgs::msg::Pose> path;
      for (const auto & pose_stamped : msg->poses) {
        path.push_back(pose_stamped.pose);
      }
      smoothPath(path);
      });
  RCLCPP_INFO(this->get_logger(), "PathSmoothing node has been started.");

}

PathSmoothing::~PathSmoothing()
{
  RCLCPP_INFO(this->get_logger(), "PathSmoothing node is shutting down.");
}

/**
 * @brief Smooths an input path using cubic B-spline interpolation.
 *
 * Converts the input set of poses (x, y coordinates) into a continuous,
 * smooth curve using cubic B-spline interpolation, then publishes it as
 * a `nav_msgs::msg::Path` message.
 *
 * @param path The vector of geometry_msgs::msg::Pose points representing the coarse input path.
 */
void PathSmoothing::smoothPath(std::vector<geometry_msgs::msg::Pose> & path)
{
  // Check for sufficient points to perform smoothing
  if (path.size() < 4) {
    RCLCPP_WARN(this->get_logger(), "Path too short to smooth.");
    return;
  }
  std::vector<geometry_msgs::msg::Pose> smoothed_path;

  std::vector<double> x, y;
  for (auto & pose : path) {
    x.push_back(pose.position.x);
    y.push_back(pose.position.y);
  }

  auto [smooth_x, smooth_y] = cubicBSplineSmooth(x, y, spline_resolution_);

  nav_msgs::msg::Path smooth_path;
  smooth_path.header.stamp = this->now();
  smooth_path.header.frame_id = frame_id_;

  for (size_t i = 0; i < smooth_x.size(); ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header = smooth_path.header;
    p.pose.position.x = smooth_x[i];
    p.pose.position.y = smooth_y[i];
    p.pose.orientation.w = 1.0;
    smooth_path.poses.push_back(p);
  }

  // Publish the smoothed path
  pub_smooth_path_->publish(smooth_path);
  RCLCPP_INFO(this->get_logger(), "Published smoothed path with %zu points",
              smooth_x.size());
}

/**
 * @brief Cubic B-spline smoothing algorithm.
 *
 * This function applies cubic B-spline interpolation to generate a smooth curve
 * between a set of discrete (x, y) points.
 *
 * The spline basis functions (b0, b1, b2, b3) define how each set of 4
 * consecutive points contribute to the curve.
 *
 * @param x The vector of x coordinates of the input path.
 * @param y The vector of y coordinates of the input path.
 * @param resolution Number of interpolated points between each original segment.
 * @return A pair of vectors (smooth_x, smooth_y) representing the smoothed curve.
 */
std::pair<std::vector<double>, std::vector<double>>
PathSmoothing::cubicBSplineSmooth(
  const std::vector<double> & x,
  const std::vector<double> & y,
  int resolution)
{
  int n = x.size();
  std::vector<double> smooth_x, smooth_y;

  // Add the first point to ensure the curve starts correctly
  smooth_x.push_back(x[0]);
  smooth_y.push_back(y[0]);

  // B-spline coefficients for cubic curve segments
  for (int i = 0; i < n - 3; i++) {
    for (int j = 0; j <= resolution; j++) {
      double t = static_cast<double>(j) / resolution;

      // Cubic B-spline basis functions
      double b0 = (-t * t * t + 3 * t * t - 3 * t + 1) / 6.0;
      double b1 = (3 * t * t * t - 6 * t * t + 4) / 6.0;
      double b2 = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6.0;
      double b3 = (t * t * t) / 6.0;

      // Compute the interpolated point
      double sx = b0 * x[i] + b1 * x[i + 1] + b2 * x[i + 2] + b3 * x[i + 3];
      double sy = b0 * y[i] + b1 * y[i + 1] + b2 * y[i + 2] + b3 * y[i + 3];

      smooth_x.push_back(sx);
      smooth_y.push_back(sy);
    }
  }

  // Add the last point to ensure the curve ends correctly
  smooth_x.push_back(x[n - 1]);
  smooth_y.push_back(y[n - 1]);

  return {smooth_x, smooth_y};
}

} // namespace path_smoothing
