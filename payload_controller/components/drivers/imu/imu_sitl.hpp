#pragma once

#include <gz/msgs/imu.pb.h>

#include <cstddef>
#include <gz/transport/Node.hh>

#include "imu.hpp"
#include "sitl_runtime.hpp"
#include "static_mutex.hpp"

namespace drivers
{

class IMU_SITL : public IMU
{
public:
  IMU_SITL();

  void start() override;
  void update() override;

private:
  /**
   * @brief Writes the IMU topic to a C-style char array.
   *
   * @param[out] buf Buffer that the topic string is written to.
   * @param size Size of the buffer. Use sizeof().
   */
  void make_imu_topic(char* buf, std::size_t size);

  /**
   * @brief Callback function for the IMU topic.
   * Runs in a gz background thread not managed by FreeRTOS/Linux/POSIX
   * simulator.
   *
   * @param gz_msg Gazebo IMU message.
   */
  void on_imu_msg(const gz::msgs::IMU& gz_msg);

  util::StaticMutex pending_mutex_;
  util::SampleFifo<sensor_msgs_msg_Imu, SAMPLE_BUFFER_CAPACITY> pending_;

  /// Contains Simulation Configuration (e.g. entity and world name)
  const sitl::SimConfig sitl_config_;
  /// Declared last so callbacks stop before pending sample storage is destroyed.
  gz::transport::Node gz_node_;
};

}  // namespace drivers
