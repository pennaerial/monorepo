#pragma once

#include <cstdint>
#include <memory>

#include "payload_corner.hpp"
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

  /// Creates an encoder implementation (sitl, hardware). Implemented in backend .cpp files.
  static std::unique_ptr<Encoder> create(PayloadCorner corner);

  PayloadCorner corner() const;

protected:
  explicit Encoder(PayloadCorner corner);

private:
  PayloadCorner corner_;
  payload_interfaces_msg_MotorState reading_{};
  uint32_t update_count_ = 0;
  util::StaticMutex mtx_;
};

}  // namespace drivers
