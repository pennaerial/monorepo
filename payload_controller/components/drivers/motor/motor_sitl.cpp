#include "motor_sitl.hpp"

#include <cstdio>

#include "esp_log.h"

static const char* TAG = "Motor_SITL";

namespace drivers
{

namespace
{
const char* command_topic_for(Side side)
{
  return side == Side::LEFT ? "command/motor_speed_left" : "command/motor_speed_right";
}
}  // namespace

Motor_SITL::Motor_SITL(Side side) : sitl_config_(sitl::get_config()), command_topic_(command_topic_for(side)) {}

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

Motor& get_motor(Side side)
{
  static Motor_SITL left(Side::LEFT);
  static Motor_SITL right(Side::RIGHT);
  return side == Side::LEFT ? left : right;
}

}  // namespace drivers
