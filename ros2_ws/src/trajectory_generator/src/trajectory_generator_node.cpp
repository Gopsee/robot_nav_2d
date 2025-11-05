/**
 * @file trajectory_generator_node.cpp
 *
 * @brief Main node file for the Trajectory Generator ROS2 node.
 * This node subscribes to path messages and generates trajectories
 * to be sent to a trajectory controller via action interface.
 */
#include "trajectory_generator/trajectory_generator.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<trajectory_generator::TrajectoryGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
