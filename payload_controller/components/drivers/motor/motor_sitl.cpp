#include "motor_sitl.hpp"

#include <cstdio>

#include "esp_log.h"

static const char* TAG = "Motor_SITL";

namespace drivers
{

Motor_SITL::Motor_SITL(PayloadCorner corner) : Motor(corner), sitl_config_(sitl::get_config()) {}

void Motor_SITL::start()
{
  char topic[128];
  make_motor_advertiser(topic, sizeof(topic));

  ESP_LOGI(TAG, "Enabling %s motor posting to %s", payload_corner_name(corner()), topic);
  motor_publisher_ = gz_node_.Advertise<gz::msgs::Actuators>(topic);
  ESP_LOGI(TAG, "%s actuator publisher valid: %d", payload_corner_name(corner()), motor_publisher_.Valid());
}

void Motor_SITL::make_motor_advertiser(char* buf, std::size_t size)
{
  std::snprintf(buf, size, "/model/%s/%s", sitl_config_.gz_model, payload_corner_motor_command_topic(corner()));
}

void Motor_SITL::publish(double rad_s)
{
  if (!motor_publisher_.Valid()) {
    return;
  }

  gz::msgs::Actuators msg;
  msg.add_velocity(rad_s);
  motor_publisher_.Publish(msg);
}

std::unique_ptr<Motor> Motor::create(PayloadCorner corner)
{
  return std::make_unique<Motor_SITL>(corner);
}

}  // namespace drivers
