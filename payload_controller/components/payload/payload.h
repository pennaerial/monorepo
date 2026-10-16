#pragma once

#include <array>
#include <memory>

#include "dds_client.hpp"
#include "encoder.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.hpp"
#include "motor.hpp"
#include "payload_corner.hpp"
#include "payload_interfaces/msg/MotorState.h"
#include "topics.h"

class Payload
{
public:
  void init();
  void update();
  void publish_sensor_debug();
  void motor_updates();

  static Payload* instance();

private:
  Payload();
  ~Payload() = default;

  Payload(const Payload&) = delete;
  Payload& operator=(const Payload&) = delete;
  Payload(Payload&&) = delete;
  Payload& operator=(Payload&&) = delete;

  static constexpr std::array<PayloadCorner, 2> PAYLOAD_CORNERS = {
      PayloadCorner::front_left, PayloadCorner::front_right
  };

  drivers::IMU* imu;
  std::array<std::unique_ptr<drivers::Encoder>, PAYLOAD_CORNERS.size()> encoders;
  std::array<std::unique_ptr<drivers::Motor>, PAYLOAD_CORNERS.size()> motors;
  uint32_t last_published_imu_update_ = 0;
  std::array<uint32_t, PAYLOAD_CORNERS.size()> last_published_encoder_updates{};
  payload_interfaces_msg_MotorState latest_motor_state{};
};
