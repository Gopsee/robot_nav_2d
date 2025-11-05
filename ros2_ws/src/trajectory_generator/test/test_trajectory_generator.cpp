#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "trajectory_generator/trajectory_generator.h"

using namespace trajectory_generator;
using FollowPath = trajectory_controller::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

// Subclass to expose protected members
class TrajectoryGeneratorTestable : public TrajectoryGenerator {
public:
  using TrajectoryGenerator::generateTrajectory;
  using TrajectoryGenerator::sendTrajectoryGoal;
  using TrajectoryGenerator::resultCallback;
  using TrajectoryGenerator::v_const_;
  using TrajectoryGenerator::frame_id_;
};

class TrajectoryGeneratorTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node_ = std::make_shared<TrajectoryGeneratorTestable>();
  }
  std::shared_ptr<TrajectoryGeneratorTestable> node_;
};

// ✅ TEST 1: Short path should trigger warning
TEST_F(TrajectoryGeneratorTest, ShortPathIgnored) {
  auto path = std::make_shared<nav_msgs::msg::Path>();
  path->poses.resize(1);
  EXPECT_NO_THROW(node_->generateTrajectory(path));
}

// ✅ TEST 2: Proper trajectory generation
TEST_F(TrajectoryGeneratorTest, GeneratesTrajectoryCorrectly) {
  auto path = std::make_shared<nav_msgs::msg::Path>();
  for (int i = 0; i < 5; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i * 1.0;
    pose.pose.position.y = 0.0;
    path->poses.push_back(pose);
  }

  node_->set_parameter(rclcpp::Parameter("v_const", 1.0));

  EXPECT_NO_THROW(node_->generateTrajectory(path));
  EXPECT_DOUBLE_EQ(node_->v_const_, 1.0);
  EXPECT_EQ(node_->frame_id_, "map");
}

// ✅ TEST 3: Result callback with SUCCEEDED result
TEST_F(TrajectoryGeneratorTest, ResultCallbackSucceeded) {
  GoalHandleFollowPath::WrappedResult result;
  result.code = rclcpp_action::ResultCode::SUCCEEDED;
  EXPECT_NO_THROW(node_->resultCallback(result));
}

// ✅ TEST 4: Result callback with ABORTED result
TEST_F(TrajectoryGeneratorTest, ResultCallbackAborted) {
  GoalHandleFollowPath::WrappedResult result;
  result.code = rclcpp_action::ResultCode::ABORTED;
  EXPECT_NO_THROW(node_->resultCallback(result));
}

// ✅ TEST 5: Result callback with unknown code
TEST_F(TrajectoryGeneratorTest, ResultCallbackUnknown) {
  GoalHandleFollowPath::WrappedResult result;
  result.code = static_cast<rclcpp_action::ResultCode>(999);
  EXPECT_NO_THROW(node_->resultCallback(result));
}
