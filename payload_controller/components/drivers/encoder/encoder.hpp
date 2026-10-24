#pragma once

#include <cstdint>

#include "payload_corner.hpp"
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
  /// Writes a new incoming encoder tick count to the local cache.
  void write_latest(int64_t ticks);
  /// Returns the latest encoder tick count.
  int64_t get_latest();
  /// Returns true and copies the latest tick count if it is newer than last_update_count.
  bool get_latest_if_updated(uint32_t& last_update_count, int64_t& ticks);
  /// Gets the encoder instance for the given side (sitl, hardware). implemented in backend .cpp
  /// files, not encoder.cpp
  static Encoder& instance(Side side);

protected:
  Encoder() = default;  // only backends construct the base

private:
  int64_t ticks_ = 0;
  uint32_t update_count_ = 0;
  util::StaticMutex mtx_;
};

}  // namespace drivers
