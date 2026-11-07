#pragma once

#include <cstddef>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <payload_interfaces/msg/drive_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

class PathFollower : public rclcpp::Node
{
public:
  PathFollower();

private:
  /// Gets and stores current path to follow
  void path_callback(nav_msgs::msg::Path::SharedPtr msg);
  void control_loop();
  bool get_robot_pose(double& x, double& y, double& yaw);
  void publish_drive(double linear, double angular);

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
  rclcpp::Publisher<payload_interfaces::msg::DriveCommand>::SharedPtr drive_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::Path current_path_;
  std::size_t waypoint_index_ = 0;
  bool following_path_ = false;

  /// Controller settings
  double linear_speed_ = 0.3;        /// m/s
  double angular_gain_ = 1.5;        /// idk
  double max_angular_speed_ = 0.8;   /// rad/s
  double waypoint_tolerance_ = 0.2;  /// m
  double heading_tolerance_ = 0.15;  /// rad
};
