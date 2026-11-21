#pragma once

#include <cstdint>

#include "sample_fifo.hpp"
#include "sensor_msgs/msg/Imu.h"
#include "task_mutex.hpp"

class DDSClient;

namespace drivers
{

/**
 * @class IMU
 * @brief Abstract Interface for IMU sensor class. The point of this class is to give a clean
 * interface so the Payload Controller doesn't need to know where it gets sensor data from. This
 * header is the only interface the rest of the application needs, hence why there is no
 * imu_sitl.hpp
 *
 */
class IMU
{
public:
  virtual ~IMU() = default;

  /// Compact timestamped input for the estimator, without telemetry fields.
  struct Sample {
    int64_t timestamp_ns;
    double angular_velocity[3];
    double linear_acceleration[3];
  };
  static constexpr std::size_t SAMPLE_BUFFER_CAPACITY = 64;

  /// Drains up to capacity samples in arrival order; only the estimator consumes this FIFO.
  /// Reports overruns since the previous drain. Latest-value telemetry reads are independent.
  std::size_t read_samples(Sample* samples, std::size_t capacity, std::size_t& dropped);

  /// Starts the IMU. It should immediately start writing imu messages to internal buffer
  virtual void start() = 0;
  /// Writes a new incoming imu reading to the local cache
  void write_latest(const sensor_msgs_msg_Imu& msg);
  /// returns the latest imu reading
  sensor_msgs_msg_Imu get_latest();
  /// Returns true and copies the latest imu reading if it is newer than last_update_count.
  bool get_latest_if_updated(uint32_t& last_update_count, sensor_msgs_msg_Imu& msg);
  /// Gets singleton instance of imu implementation (sitl, hardware). implemented in backend .cpp
  /// files, not imu.cpp
  static IMU* instance();
  /// Polls the sensor; backends receiving samples asynchronously need no work here.
  virtual void update() {}

protected:
  IMU() = default;  // prevent public instantiation

private:
  sensor_msgs_msg_Imu reading_{};
  uint32_t update_count_ = 0;
  util::SampleFifo<Sample, SAMPLE_BUFFER_CAPACITY> estimator_samples_;
  util::TaskMutex mtx_;
};

}  // namespace drivers
