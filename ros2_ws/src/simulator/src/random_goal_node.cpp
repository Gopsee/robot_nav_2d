#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cmath>
#include <random>

static constexpr double GOAL_TOLERANCE  = 2.0;   // metres — triggers next goal
static constexpr double GOAL_BOUNDS     = 30.0;  // ± metres from origin
static constexpr double MIN_ORIGIN_DIST = 5.0;   // keep goals away from start

class RandomGoalNode : public rclcpp::Node
{
public:
  RandomGoalNode()
  : Node("random_goal_node"),
    rng_(std::random_device{}()),
    pos_dist_(-GOAL_BOUNDS, GOAL_BOUNDS)
  {
    robot_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/robot_1/goal_pose", 10);
    drone_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/drone_1/goal_pose", 10);

    robot_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robot_1/estimated_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        robot_x_ = msg->pose.position.x;
        robot_y_ = msg->pose.position.y;
        robot_ready_ = true;
      });

    drone_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone_1/estimated_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        drone_x_ = msg->pose.position.x;
        drone_y_ = msg->pose.position.y;
        drone_ready_ = true;
      });

    // Check at 2 Hz whether each agent has reached its current goal
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RandomGoalNode::tick, this));

    RCLCPP_INFO(get_logger(), "Random goal node started — waiting for first poses...");
  }

private:
  void tick()
  {
    if (robot_ready_) {
      if (!robot_has_goal_ ||
          std::hypot(robot_x_ - robot_gx_, robot_y_ - robot_gy_) < GOAL_TOLERANCE) {
        sendGoal(robot_goal_pub_, robot_gx_, robot_gy_, "robot_1");
        robot_has_goal_ = true;
      }
    }

    if (drone_ready_) {
      if (!drone_has_goal_ ||
          std::hypot(drone_x_ - drone_gx_, drone_y_ - drone_gy_) < GOAL_TOLERANCE) {
        sendGoal(drone_goal_pub_, drone_gx_, drone_gy_, "drone_1");
        drone_has_goal_ = true;
      }
    }
  }

  void sendGoal(
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub,
    double & gx, double & gy,
    const std::string & name)
  {
    do {
      gx = pos_dist_(rng_);
      gy = pos_dist_(rng_);
    } while (gx * gx + gy * gy < MIN_ORIGIN_DIST * MIN_ORIGIN_DIST);

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp    = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = gx;
    msg.pose.position.y = gy;
    msg.pose.orientation.w = 1.0;
    pub->publish(msg);

    RCLCPP_INFO(get_logger(), "[%s] new goal → (%.1f, %.1f)", name.c_str(), gx, gy);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr drone_goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mt19937 rng_;
  std::uniform_real_distribution<double> pos_dist_;

  double robot_x_{0}, robot_y_{0};
  double drone_x_{0}, drone_y_{0};
  double robot_gx_{0}, robot_gy_{0};
  double drone_gx_{0}, drone_gy_{0};

  bool robot_ready_{false}, robot_has_goal_{false};
  bool drone_ready_{false}, drone_has_goal_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomGoalNode>());
  rclcpp::shutdown();
  return 0;
}
