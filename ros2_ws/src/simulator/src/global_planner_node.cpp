#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planner_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "Global Planner Node started.");

    // Parameters
    this->declare_parameter<std::string>("global_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<int>("num_waypoints", 20);

    global_frame_ = this->get_parameter("global_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    num_waypoints_ = this->get_parameter("num_waypoints").as_int();

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("input_path", 10);

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));


    RCLCPP_INFO(this->get_logger(), "Waiting for goal from RViz...");
  }

private:
  struct Point { double x, y; };

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Get robot current pose from TF
    Point start;
    if (!getCurrentRobotPose(start)) {
      RCLCPP_WARN(this->get_logger(),
        "Failed to get start pose from TF. Skipping path generation.");
      return;
    }

    Point goal;
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;
    global_frame_ = msg->header.frame_id;
    RCLCPP_INFO(this->get_logger(),
                "Received new goal from RViz: (%.2f, %.2f)", goal.x, goal.y);

    // Generate new path
    path_ = generatePath(start, goal);
    path_.header.frame_id = global_frame_;
    path_.header.stamp = this->get_clock()->now();

    // Publish once immediately
    path_pub_->publish(path_);
    RCLCPP_INFO(this->get_logger(), "Published new global path with %zu waypoints.",
                path_.poses.size());
  }

  bool getCurrentRobotPose(Point & start)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
      start.x = tf.transform.translation.x;
      start.y = tf.transform.translation.y;
      return true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      return false;
    }
  }

  nav_msgs::msg::Path generatePath(const Point & start, const Point & goal)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;

    for (int i = 0; i <= num_waypoints_; ++i) {
      double t = static_cast<double>(i) / num_waypoints_;
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = start.x + t * (goal.x - start.x);
      pose.pose.position.y = start.y + t * (goal.y - start.y);
      pose.pose.orientation.w = 1.0; // facing forward
      path.poses.push_back(pose);
    }

    return path;
  }


  // -----------------------------
  // Members
  // -----------------------------
  std::string global_frame_;
  std::string base_frame_;
  int num_waypoints_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  nav_msgs::msg::Path path_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
