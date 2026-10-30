#include "encoder_sitl.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>

#include "esp_log.h"

static const char* TAG = "Encoder_SITL";

namespace
{

const char* joint_name_for(Side side)
{
  return side == Side::LEFT ? "front_left_wheel_joint" : "front_right_wheel_joint";
}

// Matches the current payload-controller output encoder configuration.
constexpr double RADIANS_PER_REVOLUTION = 2 * std::numbers::pi;

int64_t radians_to_encoder_ticks(const double radians)
{
  return static_cast<int64_t>(std::llround(radians * drivers::Encoder::TICKS_PER_REVOLUTION / RADIANS_PER_REVOLUTION));
}

}  // namespace

namespace drivers
{


Encoder_SITL::Encoder_SITL(Side side) : sitl_config_(sitl::get_config()), joint_name_(joint_name_for(side)) {}

void Encoder_SITL::start()
{
  char topic[128];
  make_encoder_topic(topic, sizeof(topic));

  if (!gz_node_.Subscribe(topic, &Encoder_SITL::on_encoder_msg, this)) {
    ESP_LOGE(TAG, "Failed to subscribe to encoder topic %s", topic);
  } else {
    ESP_LOGI(TAG, "Subscribed %s encoder to topic %s", joint_name_, topic);
  }
}

void Encoder_SITL::make_encoder_topic(char* buf, std::size_t size)
{
  std::snprintf(buf, size, "/world/%s/model/%s/joint_state", sitl_config_.gz_world, sitl_config_.gz_model);
}

void Encoder_SITL::on_encoder_msg(const gz::msgs::Model& gz_msg)
{
  for (int i = 0; i < gz_msg.joint_size(); ++i) {
    const gz::msgs::Joint& joint = gz_msg.joint(i);
    if (!joint.has_axis1() || joint.name() != joint_name_) {
      continue;
    }

    const int64_t ticks = radians_to_encoder_ticks(joint.axis1().position());
    const auto now = std::chrono::steady_clock::now();
    const double delta_s = std::chrono::duration<double>(now - previous_sample_time_).count();
    if (has_previous_sample_ && delta_s > 0.0) {
      const double angular_velocity_rad_s =
          (static_cast<double>(ticks) - previous_ticks_) * RADIANS_PER_REVOLUTION / (TICKS_PER_REVOLUTION * delta_s);
      write_latest(ticks, angular_velocity_rad_s);
    } else {
      // The first count seeds the difference; it is not a velocity measurement.
      write_latest(ticks);
    }
    previous_ticks_ = ticks;
    previous_sample_time_ = now;
    has_previous_sample_ = true;
    return;
  }

  ++missing_wheel_joint_count_;
  if (missing_wheel_joint_count_ <= 3 || (missing_wheel_joint_count_ % 50) == 0) {
    ESP_LOGW(TAG, "Joint-state message did not include %s", joint_name_);
  }
}

Encoder& Encoder::instance(Side side)
{
  static Encoder_SITL left(Side::LEFT);
  static Encoder_SITL right(Side::RIGHT);
  return side == Side::LEFT ? left : right;
}

}  // namespace drivers
