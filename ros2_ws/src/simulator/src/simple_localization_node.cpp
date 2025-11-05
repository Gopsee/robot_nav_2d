#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

class SimpleLocalizationNode : public rclcpp::Node
{
public:
  SimpleLocalizationNode()
  : Node("simple_localization_node"), x_(0.0), y_(0.0), theta_(0.0),
    v_(0.0), w_(0.0)
  {
        // Subscribe to velocity commands
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SimpleLocalizationNode::cmdCallback, this, std::placeholders::_1));

        // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer for continuous updates (10 Hz)
    double publish_rate = 10.0;      // Hz
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&SimpleLocalizationNode::update, this));

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Simple Localization Node Started (publishing odom @ %.1f Hz)",
      publish_rate);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  double x_, y_, theta_;     // Pose
  double v_, w_;             // Linear and angular velocity
  rclcpp::Time last_time_;

    // Called whenever /cmd_vel is updated
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    v_ = msg->linear.x;
    w_ = msg->angular.z;
  }

    // Called at a fixed rate (e.g., 10 Hz)
  void update()
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

        // Integrate motion model
    if (fabs(w_) < 1e-6) {
      x_ += v_ * dt * cos(theta_);
      y_ += v_ * dt * sin(theta_);
    } else {
      x_ += (v_ / w_) * (sin(theta_ + w_ * dt) - sin(theta_));
      y_ += (v_ / w_) * (-cos(theta_ + w_ * dt) + cos(theta_));
      theta_ += w_ * dt;
    }

        // Normalize angle
    if (theta_ > M_PI) {theta_ -= 2.0 * M_PI;} else if (theta_ < -M_PI) {theta_ += 2.0 * M_PI;}

        // Quaternion for orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

        // -----------------
        // Publish Odometry
        // -----------------
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = v_;
    odom.twist.twist.angular.z = w_;

    odom_pub_->publish(odom);

        // -----------------
        // Publish TF
        // -----------------
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);

        // -----------------
        // Publish PoseStamped (optional for RViz)
        // -----------------
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = current_time;
    pose_msg.header.frame_id = "odom";
    pose_msg.pose = odom.pose.pose;
    pose_pub_->publish(pose_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
