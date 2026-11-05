#pragma once

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/model.pb.h>

#include <cstddef>
#include <gz/transport/Node.hh>

#include "encoder.hpp"
#include "sitl_runtime.hpp"

namespace drivers
{

class Encoder_SITL : public Encoder
{
public:
  Encoder_SITL(Side side);

  void start() override;

private:
  /**
   * @brief Writes the Encoder topic to a C-style char array.
   *
   * @param[out] buf Buffer that the topic string is written to.
   * @param size Size of the buffer. Use sizeof().
   */
  void make_encoder_topic(char* buf, std::size_t size);

  /**
   * @brief Writes the Encoder poster to a C-style char array.
   *
   * @param[out] buf Buffer that the poster string is written to.
   * @param size Size of the buffer. Use sizeof().
   */
  void make_encoder_poster(char* buf, std::size_t size);

  /**
   * @brief Callback function for the Encoder topic.
   * Runs in a gz background thread not managed by FreeRTOS/Linux/POSIX
   * sencoderlator.
   *
   * @param gz_msg Gazebo Encoder message.
   */
  void on_encoder_msg(const gz::msgs::Model& gz_msg);

  /// Contains Simulation Configuration (e.g. entity and world name)
  const sitl::SimConfig sitl_config_;
  const char* joint_name_;
  uint32_t missing_wheel_joint_count_ = 0;
  bool has_previous_sample_ = false;
  int64_t previous_ticks_ = 0;
  /// Gazebo simulation time at the start of the velocity measurement window.
  int64_t previous_sample_time_ns_ = 0;
  /// Declared last so callbacks stop before the state above is destroyed.
  gz::transport::Node gz_node_;
};

}  // namespace drivers
