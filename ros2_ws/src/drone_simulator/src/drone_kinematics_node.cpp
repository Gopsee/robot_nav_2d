#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <algorithm>
#include <cmath>

class DroneKinematicsNode : public rclcpp::Node
{
public:
  DroneKinematicsNode()
  : Node("drone_kinematics_node"),
    x_(0.0), y_(0.0), z_(0.0), yaw_(0.0),
    vx_(0.0), omega_(0.0), z_setpoint_(0.0)
  {
    this->declare_parameter<double>("cruise_altitude", 5.0);
    this->declare_parameter<double>("altitude_kp",    2.0);
    this->declare_parameter<double>("max_vz",         2.0);
    this->declare_parameter<double>("update_rate",   50.0);

    cruise_altitude_ = this->get_parameter("cruise_altitude").as_double();
    altitude_kp_     = this->get_parameter("altitude_kp").as_double();
    max_vz_          = this->get_parameter("max_vz").as_double();
    double rate_hz   = this->get_parameter("update_rate").as_double();

    // All topic names are relative — namespace is applied by the launch file
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&DroneKinematicsNode::cmdCallback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      std::bind(&DroneKinematicsNode::goalCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz)),
      std::bind(&DroneKinematicsNode::update, this));

    last_time_ = this->now();
    RCLCPP_INFO(get_logger(),
      "Drone Kinematics started (%.0f Hz, cruise alt=%.1f m, Kp_z=%.1f)",
      rate_hz, cruise_altitude_, altitude_kp_);
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    vx_    = msg->linear.x;
    omega_ = msg->angular.z;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr)
  {
    z_setpoint_ = cruise_altitude_;
    RCLCPP_INFO(get_logger(), "Goal received — climbing to %.1f m", cruise_altitude_);
  }

  void update()
  {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // XY integration — same differential-drive model as the ground robot
    if (std::fabs(omega_) < 1e-6) {
      x_ += vx_ * dt * std::cos(yaw_);
      y_ += vx_ * dt * std::sin(yaw_);
    } else {
      x_ += (vx_ / omega_) * (std::sin(yaw_ + omega_ * dt) - std::sin(yaw_));
      y_ += (vx_ / omega_) * (-std::cos(yaw_ + omega_ * dt) + std::cos(yaw_));
      yaw_ += omega_ * dt;
    }
    if (yaw_ >  M_PI) { yaw_ -= 2.0 * M_PI; }
    if (yaw_ < -M_PI) { yaw_ += 2.0 * M_PI; }

    // Altitude P-controller
    double vz = altitude_kp_ * (z_setpoint_ - z_);
    vz = std::clamp(vz, -max_vz_, max_vz_);
    z_ += vz * dt;
    z_ = std::max(z_, 0.0);   // cannot go underground

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    // Publish odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp         = now;
    odom.header.frame_id      = "odom";
    odom.child_frame_id       = "drone_base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x  = vx_;
    odom.twist.twist.linear.z  = vz;
    odom.twist.twist.angular.z = omega_;
    odom_pub_->publish(odom);

    // Broadcast TF: odom → drone_base_link
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp            = now;
    tf_msg.header.frame_id         = "odom";
    tf_msg.child_frame_id          = "drone_base_link";
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = z_;
    tf_msg.transform.rotation.x    = q.x();
    tf_msg.transform.rotation.y    = q.y();
    tf_msg.transform.rotation.z    = q.z();
    tf_msg.transform.rotation.w    = q.w();
    tf_broadcaster_->sendTransform(tf_msg);

    // Publish PoseStamped for lidar node
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp    = now;
    pose_msg.header.frame_id = "odom";
    pose_msg.pose            = odom.pose.pose;
    pose_pub_->publish(pose_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                  tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr                                    timer_;

  double x_, y_, z_, yaw_;
  double vx_, omega_;
  double z_setpoint_;
  double cruise_altitude_, altitude_kp_, max_vz_;
  rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
