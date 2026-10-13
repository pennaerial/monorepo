#pragma once

enum class PayloadCorner {
  front_left,
  front_right,
  back_right,
  back_left,
};

constexpr bool payload_corner_is_left(const PayloadCorner corner)
{
  return corner == PayloadCorner::front_left || corner == PayloadCorner::back_left;
}

constexpr const char* payload_corner_name(const PayloadCorner corner)
{
  switch (corner) {
    case PayloadCorner::front_left: return "front_left";
    case PayloadCorner::front_right: return "front_right";
    case PayloadCorner::back_right: return "back_right";
    case PayloadCorner::back_left: return "back_left";
  }

  return "unknown";
}

constexpr const char* payload_corner_wheel_joint_name(const PayloadCorner corner)
{
  switch (corner) {
    case PayloadCorner::front_left: return "front_left_wheel_joint";
    case PayloadCorner::front_right: return "front_right_wheel_joint";
    case PayloadCorner::back_right: return "rear_right_wheel_joint";
    case PayloadCorner::back_left: return "rear_left_wheel_joint";
  }

  return "";
}

constexpr const char* payload_corner_motor_command_topic(const PayloadCorner corner)
{
  switch (corner) {
    case PayloadCorner::front_left: return "command/motor_speed_left";
    case PayloadCorner::front_right: return "command/motor_speed_right";
    case PayloadCorner::back_right: return "command/motor_speed_back_right";
    case PayloadCorner::back_left: return "command/motor_speed_back_left";
  }

  return "";
}
