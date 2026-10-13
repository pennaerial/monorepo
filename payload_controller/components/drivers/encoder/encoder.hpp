#pragma once

#include <cstdint>

#include "payload_interfaces/msg/MotorState.h"
#include "static_mutex.hpp"

namespace drivers
{

/**
 * @class Encoder
 * @brief Abstract Interface for Encoder sensor class. The point of this class is to give a clean
 * interface so the Payload Controller doesn't need to know where it gets sensor data from. This
 * header is the only interface the rest of the application needs, hence why there is no
 * Encoder_sitl.hpp
 *
 */
class Encoder
{
public:
  virtual ~Encoder() = default;

  /// Starts the encoder. It should immediately start writing encoder messages to internal buffer
  virtual void start() = 0;
  /// Writes a new incoming encoder reading to the local cache.
  void write_latest(const payload_interfaces_msg_MotorState& msg);
  /// Returns the latest encoder reading.
  payload_interfaces_msg_MotorState get_latest();
  /// Returns true and copies the latest reading if it is newer than last_update_count.
  bool get_latest_if_updated(uint32_t& last_update_count, payload_interfaces_msg_MotorState& msg);

  /// Gets singleton instance of encoder implementation (sitl, hardware). implemented in backend .cpp
  /// files, not encoder.cpp
  static Encoder* instance();

  /**
   * @brief Publishes the desired right motor velocity
   *
   * @param rad_s Desired velocity
   */
  virtual void publish_motor_right(double rad_s) = 0;

  /**
   * @brief Publishes the desired left motor velocity
   *
   * @param rad_s Desired velocity
   */
  virtual void publish_motor_left(double rad_s) = 0;

protected:
  Encoder() = default;  // prevent public instantiation

private:
  payload_interfaces_msg_MotorState reading_{};
  uint32_t update_count_ = 0;
  util::StaticMutex mtx_;
};

}  // namespace drivers
