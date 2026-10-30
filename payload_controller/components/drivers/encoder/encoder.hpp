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
  /// Encoder counts per output-shaft revolution (including quadrature decoding).
  static constexpr double TICKS_PER_REVOLUTION = 617.0;

  virtual ~Encoder() = default;

  /// Starts the encoder. It should immediately start writing encoder messages to internal buffer
  virtual void start() = 0;
  /// Caches the latest encoder tick count.
  void write_latest(int64_t ticks);
  /// Atomically caches ticks and output-shaft angular velocity in rad/s.
  void write_latest(int64_t ticks, double angular_velocity_rad_s);
  /// Returns the latest encoder tick count.
  int64_t get_latest();
  /// Returns true and copies the latest tick count if it is newer than last_update_count.
  bool get_latest_if_updated(uint32_t& last_update_count, int64_t& ticks);
  /// Returns the cached output-shaft angular velocity in rad/s (initially zero).
  double get_latest_angular_velocity();
  /// Reads a new velocity sample using a cursor independent of tick-count reads.
  bool get_latest_angular_velocity_if_updated(uint32_t& last_update_count, double& angular_velocity_rad_s);
  /// Gets the encoder instance for the given side (sitl, hardware). implemented in backend .cpp
  /// files, not encoder.cpp
  static Encoder& instance(Side side);

protected:
  Encoder() = default;  // only backends construct the base

private:
  int64_t ticks_ = 0;
  double angular_velocity_rad_s_ = 0.0;
  uint32_t update_count_ = 0;
  uint32_t velocity_update_count_ = 0;
  util::StaticMutex mtx_;
};

}  // namespace drivers
