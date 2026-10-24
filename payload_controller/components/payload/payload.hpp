#pragma once

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

/// Pairs a low-level encoder with the payload side it measures.
struct EncoderWrapper {
  EncoderWrapper(Side side, const char* debug_name)
      : side(side), debug_name(debug_name), encoder(drivers::Encoder::instance(side))
  {
  }

  const Side side;
  const char* const debug_name;
  drivers::Encoder& encoder;
  /// Encoder update count at the last motor-state publish.
  uint32_t last_published_update = 0;
};

/// Pairs a low-level motor with the payload side it drives.
struct MotorWrapper {
  MotorWrapper(Side side, const char* debug_name)
      : side(side), debug_name(debug_name), motor(drivers::Motor::instance(side))
  {
  }

  const Side side;
  const char* const debug_name;
  drivers::Motor& motor;
};

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

  /// Copies a new encoder reading into latest_motor_state_. Returns true if it was new.
  bool read_encoder(EncoderWrapper& wrapper);

  drivers::IMU* imu_;
  EncoderWrapper left_encoder_;
  EncoderWrapper right_encoder_;
  MotorWrapper left_motor_;
  MotorWrapper right_motor_;
  uint32_t last_published_imu_update_ = 0;
  payload_interfaces_msg_MotorState latest_motor_state_{};
};
