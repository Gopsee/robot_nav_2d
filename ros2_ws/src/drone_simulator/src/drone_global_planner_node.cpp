#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

class DroneGlobalPlannerNode : public rclcpp::Node
{
public:
  DroneGlobalPlannerNode()
  : Node("drone_global_planner_node"),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::string>("global_frame",    "map");
    this->declare_parameter<std::string>("base_frame",      "drone_base_link");
    this->declare_parameter<int>        ("num_waypoints",   20);
    this->declare_parameter<double>     ("cruise_altitude", 5.0);

    global_frame_    = this->get_parameter("global_frame").as_string();
    base_frame_      = this->get_parameter("base_frame").as_string();
    num_waypoints_   = this->get_parameter("num_waypoints").as_int();
    cruise_altitude_ = this->get_parameter("cruise_altitude").as_double();

    path_pub_ = create_publisher<nav_msgs::msg::Path>("input_path", 10);
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      std::bind(&DroneGlobalPlannerNode::goalCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Drone Global Planner ready (cruise alt=%.1f m, %d waypoints)",
      cruise_altitude_, num_waypoints_);
  }

private:
  struct Point3D { double x, y, z; };

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    Point3D start;
    if (!getDronePose(start)) {
      RCLCPP_WARN(get_logger(), "Cannot get drone pose from TF — skipping.");
      return;
    }

    // Goal is always at cruise altitude
    Point3D goal{msg->pose.position.x, msg->pose.position.y, cruise_altitude_};
    global_frame_ = msg->header.frame_id;

    RCLCPP_INFO(get_logger(),
      "Goal (%.2f, %.2f) → planning at z=%.1f m", goal.x, goal.y, cruise_altitude_);

    auto path = generatePath(start, goal);
    path.header.frame_id = global_frame_;
    path.header.stamp    = get_clock()->now();
    path_pub_->publish(path);

    RCLCPP_INFO(get_logger(), "Published 3D path with %zu waypoints.", path.poses.size());
  }

  bool getDronePose(Point3D & out)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
      out.x = tf.transform.translation.x;
      out.y = tf.transform.translation.y;
      out.z = tf.transform.translation.z;
      return true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
      return false;
    }
  }

  nav_msgs::msg::Path generatePath(const Point3D & start, const Point3D & goal)
  {
    nav_msgs::msg::Path path;
    for (int i = 0; i <= num_waypoints_; ++i) {
      double t = static_cast<double>(i) / num_waypoints_;
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id     = global_frame_;
      pose.pose.position.x     = start.x + t * (goal.x - start.x);
      pose.pose.position.y     = start.y + t * (goal.y - start.y);
      pose.pose.position.z     = cruise_altitude_;
      pose.pose.orientation.w  = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  std::string global_frame_, base_frame_;
  int         num_waypoints_;
  double      cruise_altitude_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  tf2_ros::Buffer          tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneGlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
