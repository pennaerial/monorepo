#pragma once

// [-]#include "sensor_msgs/msg/encoder.h"
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
  /// Writes a new incoming encoder reading to
  //   [-]void write_latest(const sensor_msgs_msg_encoder& msg);
  /// returns the latest Encoder reading

  //   [-]sensor_msgs_msg_encoder get_latest();

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
  //   [-]sensor_msgs_msg_encoder reading_;
  util::StaticMutex mtx_;


private:
  /// Implemented once per backend (encoder_sitl.cpp / encoder_hw.cpp).
  /// Constructs the concrete backend instance.
  static Encoder& create();
};
}  // namespace drivers
