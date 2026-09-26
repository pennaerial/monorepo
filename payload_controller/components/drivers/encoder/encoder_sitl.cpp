#include "encoder_sitl.hpp"

#include <cstdio>

#include "convert.hpp"
#include "esp_log.h"
// [-]#include "sensor_msgs/msg/encoder.h"
#include "std_msgs/msg/Header.h"

static const char* TAG = "Encoder_SITL";

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

  ESP_LOGI(TAG, "Subscribing to %s", topic);
  gz_node_.Subscribe(topic, &Encoder_SITL::on_encoder_msg, this);

  ESP_LOGI(TAG, "Enabling posting to %s", poster_right);
  right_motor_publisher = gz_node_.Advertise<gz::msgs::Actuators>(poster_right);

  ESP_LOGI(TAG, "Enabling posting to %s", poster_left);
  left_motor_publisher = gz_node_.Advertise<gz::msgs::Actuators>(poster_left);
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
  ESP_LOGI(TAG, "Changing right motor velocity");
  gz::msgs::Actuators msg;

  // TODO convert rad/s to PWM
  msg.add_velocity(rad_s);
  right_motor_publisher.Publish(msg);
}

void Encoder_SITL::publish_motor_left(double rad_s)
{
  ESP_LOGI(TAG, "Changing left motor velocity");

  gz::msgs::Actuators msg;

  // TODO convert rad/s to PWM
  msg.add_velocity(rad_s);
  left_motor_publisher.Publish(msg);
}

void Encoder_SITL::make_encoder_topic(char* buf, std::size_t size)
{
  std::snprintf(buf, size, "/world/%s/model/%s/joint_state", sitl_config_.gz_world, sitl_config_.gz_model);
}

void Encoder_SITL::on_encoder_msg(const gz::msgs::Model& gz_msg)
{
  // ESP_LOGI(TAG, "on_encoder_msg");

  // // For each joint just print the position (which should be its rotation)
  // for (int i = 0; i < gz_msg.joint_size(); ++i) {
  //   const gz::msgs::Joint& jointMsg = gz_msg.joint(i);
  //   std::string name = jointMsg.name();
  //   ESP_LOGI(TAG, "Joint Name: %s", name.c_str());
  //   if (jointMsg.has_axis1()) {
  //     double position = jointMsg.axis1().position();
  //     ESP_LOGI(TAG, "Joint Position: %f", position);
  //   }
  // }

  //   [-]sensor_msgs_msg_encoder msg = gz_to_dds(gz_msg);
  //   [-]write_latest(msg);  // update our latest encoder value
}

Encoder* Encoder::instance()
{
  static Encoder_SITL instance;  // only instantiated once because static
  return &instance;
}

}  // namespace drivers