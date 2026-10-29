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
  return side == Side::LEFT ? "joint/front_left_wheel_joint/cmd_force" : "joint/front_right_wheel_joint/cmd_force";
}
}  // namespace

Motor_SITL::Motor_SITL(Side side)
    : sitl_config_(sitl::get_config()),
      command_topic_(command_topic_for(side)),
      joint_name_(side == Side::LEFT ? "front_left_wheel_joint" : "front_right_wheel_joint")
{
}

void Motor_SITL::start()
{
  char topic[128];
  make_motor_advertiser(topic, sizeof(topic));

  ESP_LOGI(TAG, "Enabling motor posting to %s", topic);
  motor_publisher_ = gz_node_.Advertise<gz::msgs::Double>(topic);
  ESP_LOGI(TAG, "Torque publisher for %s valid: %d", topic, motor_publisher_.Valid());

  std::snprintf(topic, sizeof(topic), "/world/%s/model/%s/joint_state", sitl_config_.gz_world, sitl_config_.gz_model);
  if (!gz_node_.Subscribe(topic, &Motor_SITL::on_joint_state_msg, this)) {
    ESP_LOGE(TAG, "Failed to subscribe to motor velocity topic %s", topic);
  }
}

void Motor_SITL::on_joint_state_msg(const gz::msgs::Model& msg)
{
  for (const auto& joint : msg.joint()) {
    if (joint.name() == joint_name_ && joint.has_axis1()) {
      wheel_angular_velocity_rad_s_.store(joint.axis1().velocity(), std::memory_order_relaxed);
      return;
    }
  }
}

void Motor_SITL::make_motor_advertiser(char* buf, std::size_t size)
{
  std::snprintf(buf, size, "/model/%s/%s", sitl_config_.gz_model, command_topic_);
}

void Motor_SITL::publish(double pwm)
{
  if (!motor_publisher_.Valid()) {
    return;
  }

  gz::msgs::Double msg;
  msg.set_data(generate_motor_torque(pwm));
  motor_publisher_.Publish(msg);
}

float Motor_SITL::generate_motor_torque(float pwm)
{
  constexpr float MOTOR_TORQUE_CONSTANT = 0.002324f;  // N*m/A
  constexpr float BACK_EMF_CONSTANT = 0.002324f;      // V*s/rad
  constexpr float MOTOR_WINDING_RESISTANCE = 8.955f;  // Ohms
  constexpr float BATTERY_VOLTAGE = 6.0f;             // Volts

  // Additional gearbox constants
  constexpr float GEAR_RATIO = 100.37f;
  constexpr float GEARBOX_EFFICIENCY = 0.66f;  // Estimated

  float motorVelocity = wheel_angular_velocity_rad_s_.load(std::memory_order_relaxed) * GEAR_RATIO;

  float voltage = pwm * BATTERY_VOLTAGE;

  float current = (voltage - BACK_EMF_CONSTANT * motorVelocity) / MOTOR_WINDING_RESISTANCE;

  float motorTorque = MOTOR_TORQUE_CONSTANT * current;

  float outputTorque = motorTorque * GEAR_RATIO * GEARBOX_EFFICIENCY;

  return outputTorque;
}

Motor& Motor::instance(Side side)
{
  static Motor_SITL left(Side::LEFT);
  static Motor_SITL right(Side::RIGHT);
  return side == Side::LEFT ? left : right;
}

}  // namespace drivers
