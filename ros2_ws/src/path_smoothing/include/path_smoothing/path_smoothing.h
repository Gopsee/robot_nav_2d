/**
 * path_smoothing.h
 * @author Gowtham Pagadala
 *
 * @brief Header file for the PathSmoothing class that implements path smoothing
 * using cubic B-splines in a ROS2 node.
 */
#ifndef PATH_SMOOTHING_H
#define PATH_SMOOTHING_H

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace path_smoothing {
  class PathSmoothing: public rclcpp::Node {
private:
  // Path Smoothing Function
    void smoothPath(std::vector < geometry_msgs::msg::Pose > &path);

  // Cubic B-Spline Smoothing Helper Function
    std::pair < std::vector < double >, std::vector < double >>
    cubicBSplineSmooth(const std::vector < double > &x, const std::vector < double > &y,
                     int resolution);

    rclcpp::Subscription < nav_msgs::msg::Path > ::SharedPtr subscription_;
    rclcpp::Publisher < nav_msgs::msg::Path > ::SharedPtr pub_smooth_path_;

    int spline_resolution_ = 10;
    std::string frame_id_ = "map";

public:
    PathSmoothing();
    ~PathSmoothing();
  };
} // namespace path_smoothing

#endif // PATH_SMOOTHING_H
