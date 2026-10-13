#include "encoder_sitl.hpp"

#include <cmath>
#include <cstdio>

#include "convert.hpp"
#include "esp_log.h"
// [-]#include "sensor_msgs/msg/encoder.h"
#include "std_msgs/msg/Header.h"

static const char* TAG = "Encoder_SITL";

namespace
{

// Matches the current payload-controller output encoder configuration.
constexpr double ENCODER_TICKS_PER_REVOLUTION = 617.0;
constexpr double RADIANS_PER_REVOLUTION = 2 * std::numbers::pi;

int64_t radians_to_encoder_ticks(const double radians)
{
  return static_cast<int64_t>(std::llround(radians * ENCODER_TICKS_PER_REVOLUTION / RADIANS_PER_REVOLUTION));
}

}  // namespace

namespace drivers
{


Encoder_SITL::Encoder_SITL() : sitl_config_(sitl::get_config()) {}

void Encoder_SITL::start()
{
  char topic[128];
  make_encoder_topic(topic, sizeof(topic));

  char poster_right[128];
  make_motor_advertiser_right(poster_right, sizeof(poster_right));

  char poster_left[128];
  make_motor_advertiser_left(poster_left, sizeof(poster_left));

  if (!gz_node_.Subscribe(topic, &Encoder_SITL::on_encoder_msg, this)) {
    ESP_LOGE(TAG, "Failed to subscribe to encoder topic %s", topic);
  } else {
    ESP_LOGI(TAG, "Subscribed to encoder topic %s", topic);
  }

  ESP_LOGI(TAG, "Enabling posting to %s", poster_right);
  right_motor_publisher = motor_node_.Advertise<gz::msgs::Actuators>(poster_right);
  ESP_LOGI(TAG, "Right actuator publisher valid: %d", right_motor_publisher.Valid());

  ESP_LOGI(TAG, "Enabling posting to %s", poster_left);
  left_motor_publisher = motor_node_.Advertise<gz::msgs::Actuators>(poster_left);
  ESP_LOGI(TAG, "Left actuator publisher valid: %d", left_motor_publisher.Valid());
}

void Encoder_SITL::make_motor_advertiser_right(char* buf, std::size_t size)
{
  std::snprintf(buf, size, "/model/%s/command/motor_speed_right", sitl_config_.gz_model);
}

void Encoder_SITL::make_motor_advertiser_left(char* buf, std::size_t size)
{
  std::snprintf(buf, size, "/model/%s/command/motor_speed_left", sitl_config_.gz_model);
}

void Encoder_SITL::publish_motor_right(double rad_s)
{
  gz::msgs::Actuators msg;

  // TODO convert rad/s to PWM
  msg.add_velocity(rad_s);
  if (!right_motor_publisher.Valid()) {
    return;
  }

  right_motor_publisher.Publish(msg);
}

void Encoder_SITL::publish_motor_left(double rad_s)
{
  gz::msgs::Actuators msg;

  // TODO convert rad/s to PWM
  msg.add_velocity(rad_s);
  if (!left_motor_publisher.Valid()) {
    return;
  }

  left_motor_publisher.Publish(msg);
}

void Encoder_SITL::make_encoder_topic(char* buf, std::size_t size)
{
  std::snprintf(buf, size, "/world/%s/model/%s/joint_state", sitl_config_.gz_world, sitl_config_.gz_model);
}

void Encoder_SITL::on_encoder_msg(const gz::msgs::Model& gz_msg)
{
  payload_interfaces_msg_MotorState msg = get_latest();
  bool found_left_wheel = false;
  bool found_right_wheel = false;

  for (int i = 0; i < gz_msg.joint_size(); ++i) {
    const gz::msgs::Joint& joint = gz_msg.joint(i);
    if (!joint.has_axis1()) {
      continue;
    }

    const int64_t ticks = radians_to_encoder_ticks(joint.axis1().position());
    if (joint.name() == "front_left_wheel_joint") {
      msg.left_encoder_count = ticks;
      found_left_wheel = true;
    } else if (joint.name() == "front_right_wheel_joint") {
      msg.right_encoder_count = ticks;
      found_right_wheel = true;
    }
  }

  if (!found_left_wheel && !found_right_wheel) {
    static uint32_t missing_wheel_joint_count = 0;
    ++missing_wheel_joint_count;
    if (missing_wheel_joint_count <= 3 || (missing_wheel_joint_count % 50) == 0) {
      ESP_LOGW(TAG, "Joint-state message did not include either front wheel joint");
    }
    return;
  }

  write_latest(msg);
}

Encoder* Encoder::instance()
{
  static Encoder_SITL instance;  // only instantiated once because static
  return &instance;
}

}  // namespace drivers
