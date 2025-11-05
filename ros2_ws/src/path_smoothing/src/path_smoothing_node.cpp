/**
 * path_smoothing_node.cpp
 *
 * @brief Main entry point for the PathSmoothing ROS2 node.
 * This node initializes the PathSmoothing class and starts the ROS2 spin loop.
 */
#include "path_smoothing/path_smoothing.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<path_smoothing::PathSmoothing>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
