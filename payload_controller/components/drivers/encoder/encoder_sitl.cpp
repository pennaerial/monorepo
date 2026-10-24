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


Encoder_SITL::Encoder_SITL(PayloadCorner corner) : Encoder(corner), sitl_config_(sitl::get_config()) {}

void Encoder_SITL::start()
{
  char topic[128];
  make_encoder_topic(topic, sizeof(topic));

  if (!gz_node_.Subscribe(topic, &Encoder_SITL::on_encoder_msg, this)) {
    ESP_LOGE(TAG, "Failed to subscribe to encoder topic %s", topic);
  } else {
    ESP_LOGI(TAG, "Subscribed %s encoder to topic %s", payload_corner_name(corner()), topic);
  }
}

void Encoder_SITL::make_encoder_topic(char* buf, std::size_t size)
{
  std::snprintf(buf, size, "/world/%s/model/%s/joint_state", sitl_config_.gz_world, sitl_config_.gz_model);
}

void Encoder_SITL::on_encoder_msg(const gz::msgs::Model& gz_msg)
{
  payload_interfaces_msg_MotorState msg = get_latest();
  bool found_wheel = false;
  const char* joint_name = payload_corner_wheel_joint_name(corner());

  for (int i = 0; i < gz_msg.joint_size(); ++i) {
    const gz::msgs::Joint& joint = gz_msg.joint(i);
    if (!joint.has_axis1()) {
      continue;
    }

    const int64_t ticks = radians_to_encoder_ticks(joint.axis1().position());
    if (joint.name() == joint_name) {
      if (payload_corner_is_left(corner())) {
        msg.left_encoder_count = ticks;
      } else {
        msg.right_encoder_count = ticks;
      }
      found_wheel = true;
      break;
    }
  }

  if (!found_wheel) {
    ++missing_wheel_joint_count_;
    if (missing_wheel_joint_count_ <= 3 || (missing_wheel_joint_count_ % 50) == 0) {
      ESP_LOGW(TAG, "Joint-state message did not include %s", joint_name);
    }
    return;
  }

  write_latest(msg);
}

std::unique_ptr<Encoder> Encoder::create(PayloadCorner corner)
{
  return std::make_unique<Encoder_SITL>(corner);
}

}  // namespace drivers
