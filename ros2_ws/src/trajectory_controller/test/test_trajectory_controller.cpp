#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "trajectory_controller/trajectory_controller.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace trajectory_controller;
using FollowPath = trajectory_controller::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

class TrajectoryControllerNodeTestable : public TrajectoryControllerNode {
public:
  using TrajectoryControllerNode::findLookaheadPoint;
  using TrajectoryControllerNode::odomCallback;
  using TrajectoryControllerNode::handleGoal;
  using TrajectoryControllerNode::handleCancel;
  using TrajectoryControllerNode::trajectory_;
  using TrajectoryControllerNode::current_odom_;
};

class TrajectoryControllerTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<TrajectoryControllerNodeTestable>();
  }

  std::shared_ptr<TrajectoryControllerNodeTestable> node_;
};

// ✅ TEST 1: handleGoal should accept valid goal
TEST_F(TrajectoryControllerTest, HandleGoalAcceptsValidGoal) {
  nav_msgs::msg::Path path;
  path.poses.resize(5);
  auto goal = std::make_shared<trajectory_controller::action::FollowPath::Goal>();
  goal->path = path;

  rclcpp_action::GoalUUID uuid;
  auto response = node_->handleGoal(uuid, goal);
  EXPECT_EQ(response, rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
}

// ✅ TEST 2: handleCancel should accept cancellation
TEST_F(TrajectoryControllerTest, HandleCancelAcceptsCancel) {
  std::shared_ptr<GoalHandleFollowPath> handle;
  auto response = node_->handleCancel(handle);
  EXPECT_EQ(response, rclcpp_action::CancelResponse::ACCEPT);
}

// ✅ TEST 3: Odom callback updates internal state
TEST_F(TrajectoryControllerTest, OdomCallbackStoresMessage) {
  nav_msgs::msg::Odometry msg;
  msg.pose.pose.position.x = 1.0;
  msg.pose.pose.position.y = 2.0;

  node_->odomCallback(std::make_shared<nav_msgs::msg::Odometry>(msg));

  ASSERT_TRUE(node_->current_odom_ != nullptr);
  EXPECT_DOUBLE_EQ(node_->current_odom_->pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(node_->current_odom_->pose.pose.position.y, 2.0);
}

// ✅ TEST 4: findLookaheadPoint returns correct lookahead point
TEST_F(TrajectoryControllerTest, FindLookaheadPointBasicTest) {
  nav_msgs::msg::Path path;
  for (int i = 0; i < 5; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 1.0;
    pose.pose.position.y = 0.0;
    path.poses.push_back(pose);
  }

  node_->trajectory_ = std::make_shared<nav_msgs::msg::Path>(path);

  // Robot at x=0, y=0; lookahead distance = 2.0
  auto lookahead = node_->findLookaheadPoint(0.0, 0.0, 2.0);
  EXPECT_NEAR(lookahead.pose.position.x, 2.0, 0.1);
  EXPECT_NEAR(lookahead.pose.position.y, 0.0, 0.1);
}

// ✅ TEST 5: findLookaheadPoint should return last pose if no valid segment
TEST_F(TrajectoryControllerTest, FindLookaheadPointFallback) {
  node_->trajectory_ = std::make_shared<nav_msgs::msg::Path>();
  geometry_msgs::msg::PoseStamped last_pose;
  last_pose.pose.position.x = 5.0;
  last_pose.pose.position.y = 5.0;
  node_->trajectory_->poses.push_back(last_pose);

  auto target = node_->findLookaheadPoint(10.0, 10.0, 2.0);
  EXPECT_DOUBLE_EQ(target.pose.position.x, 5.0);
  EXPECT_DOUBLE_EQ(target.pose.position.y, 5.0);
}
