#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include "path_smoothing/path_smoothing.h"

using namespace std::chrono_literals;

class PathSmoothingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<path_smoothing::PathSmoothing>();

    got_msg_ = false;
    subscription_ = node_->create_subscription<nav_msgs::msg::Path>(
        "smooth_path", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        got_msg_ = true;
        last_smoothed_path_ = *msg;
        });

    publisher_ = node_->create_publisher<nav_msgs::msg::Path>("input_path", 10);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
  bool got_msg_;
  nav_msgs::msg::Path last_smoothed_path_;
};

TEST_F(PathSmoothingTest, publishesSmoothedPath)
{
  // Prepare input path
  nav_msgs::msg::Path input_path;
  for (int i = 0; i < 5; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    pose.pose.position.y = std::sin(i * 0.5);
    input_path.poses.push_back(pose);
  }

  // Publish input path
  publisher_->publish(input_path);

  // Wait up to 3 seconds for output
  auto start = std::chrono::steady_clock::now();
  while (!got_msg_ &&
    (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)))
  {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(got_msg_) << "No smoothed path received from node";
  EXPECT_GT(last_smoothed_path_.poses.size(), 2u)
      << "Smoothed path is unexpectedly short";
}
