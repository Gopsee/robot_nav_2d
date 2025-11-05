/**
 * trajectory_controller_node.cpp
 * 
 * @brief Main file for the TrajectoryControllerNode ROS2 node.
 * This node controls a robot to follow a given trajectory using a lookahead-based control strategy.
 */
#include "trajectory_controller/trajectory_controller.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<trajectory_controller::TrajectoryControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
