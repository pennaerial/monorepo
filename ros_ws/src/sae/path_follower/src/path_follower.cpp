#include "path_follower/path_follower.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <utility>

using namespace std::chrono_literals;

namespace
{
/// Both in meters
constexpr double WHEEL_SEPARATION = 0.112;
constexpr double WHEEL_RADIUS = 0.017;

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

PathFollower::PathFollower() : rclcpp::Node("path_follower")
{
  /// TODO: remove hardcoded payload_0
  drive_publisher_ = this->create_publisher<payload_interfaces::msg::DriveCommand>("/payload_0/cmd_drive", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS path_qos(10);
  path_qos.reliable().transient_local();

  path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", path_qos, std::bind(&PathFollower::path_callback, this, std::placeholders::_1)
  );

  control_timer_ = this->create_wall_timer(100ms, std::bind(&PathFollower::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "Path follower initialized for payload_0");
}

/// Just stores current path being published
void PathFollower::path_callback(nav_msgs::msg::Path::SharedPtr msg)
{
  /// Stops payload
  publish_drive(0.0, 0.0);

  current_path_ = *msg;
  waypoint_index_ = 0;
  following_path_ = !current_path_.poses.empty();

  if (current_path_.header.frame_id.empty()) {
    following_path_ = false;
    RCLCPP_WARN(this->get_logger(), "Received path without a frame_id");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", current_path_.poses.size());
}

std::optional<RobotPose> PathFollower::get_robot_pose()
{
  if (current_path_.header.frame_id.empty()) {
    return std::nullopt;
  }

  try {
    const auto transform = tf_buffer_->lookupTransform(current_path_.header.frame_id, "base_link", tf2::TimePointZero);

    const auto& translation = transform.transform.translation;
    const auto& rotation = transform.transform.rotation;

    tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    return RobotPose{translation.x, translation.y, yaw};

  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return std::nullopt;
  }
}

void PathFollower::publish_drive(double linear, double angular)
{
  /// Kinematic equation for tank drive velocities from linear/angular velocities
  /// Also divided by wheel radius for speed in rad/s instead of m/s
  double left_velocity = (linear - angular * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS;
  double right_velocity = (linear + angular * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS;

  /// Custom DriveCommand message I changed to be left/right instead of linear/angular
  payload_interfaces::msg::DriveCommand command;
  command.left_velocity = left_velocity;
  command.right_velocity = right_velocity;

  drive_publisher_->publish(command);
}

/// Runs 10 times per second, basic control loop calculating speeds
void PathFollower::control_loop()
{
  if (!following_path_) {
    return;
  }

  auto pose = get_robot_pose();

  if (!pose) {
    publish_drive(0.0, 0.0);
    return;
  }

  const double x = pose->x;
  const double y = pose->y;
  const double yaw = pose->yaw;

  /// Perform this as long as we haven't completed the path
  while (waypoint_index_ < current_path_.poses.size()) {
    const auto& waypoint = current_path_.poses[waypoint_index_].pose.position;

    const double distance = std::hypot(waypoint.x - x, waypoint.y - y);

    /// Perform rest of the loop if the payload hasn't gotten to the current waypoint
    if (distance >= waypoint_tolerance_) {
      break;
    }

    /// Move to next waypoint if the payload has gotten to the current waypoint
    ++waypoint_index_;
  }

  /// Stop when the path is over (duh)
  if (waypoint_index_ >= current_path_.poses.size()) {
    following_path_ = false;
    publish_drive(0.0, 0.0);

    RCLCPP_INFO(this->get_logger(), "Path completed");
    return;
  }

  const auto& target = current_path_.poses[waypoint_index_].pose.position;

  const double dx = target.x - x;
  const double dy = target.y - y;

  const double target_yaw = std::atan2(dy, dx);

  /// Normalize_angle calculates ideal angle to turn instead of having it
  /// Make like a 359 degree turn when it could make a -1 degree turn
  const double heading_error = normalize_angle(target_yaw - yaw);

  const double angular_velocity = std::clamp(angular_gain_ * heading_error, -max_angular_speed_, max_angular_speed_);

  /// Only drive once the payload is within a certain heading of its desired heading.
  /// Turns in place until it reaches roughly desired heading
  if (std::abs(heading_error) > heading_tolerance_) {
    publish_drive(0.0, angular_velocity);
    return;
  }

  publish_drive(linear_speed_, angular_velocity);
}

/// This is how you run this file (because i forget)
/* ros2 run payload_controller path_follower_node \ --ros-args -p use_sim_time:=true */