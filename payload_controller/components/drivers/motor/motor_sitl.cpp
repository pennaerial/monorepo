#include "motor_sitl.hpp"

#include <cstdio>

#include "esp_log.h"

static const char* TAG = "Motor_SITL";

namespace drivers
{

Motor_SITL::Motor_SITL(const MotorConfig& config)
    : sitl_config_(sitl::get_config()), command_topic_(config.sitl_command_topic)
{
}

void Motor_SITL::start()
{
  char topic[128];
  make_motor_advertiser(topic, sizeof(topic));

  ESP_LOGI(TAG, "Enabling motor posting to %s", topic);
  motor_publisher_ = gz_node_.Advertise<gz::msgs::Actuators>(topic);
  ESP_LOGI(TAG, "Actuator publisher for %s valid: %d", topic, motor_publisher_.Valid());
}

void Motor_SITL::make_motor_advertiser(char* buf, std::size_t size)
{
  std::snprintf(buf, size, "/model/%s/%s", sitl_config_.gz_model, command_topic_);
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

}  // namespace drivers
