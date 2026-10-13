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
  Encoder_SITL();

  void start() override;

private:
  /**
   * @brief Writes the right motor advertiser to a C-style char array.
   *
   * @param[out] buf Buffer that the topic string is written to.
   * @param size Size of the buffer. Use sizeof().
   */
  void make_motor_advertiser_right(char* buf, std::size_t size);
  /**
   * @brief Writes the left motor advertiser to a C-style char array.
   *
   * @param[out] buf Buffer that the topic string is written to.
   * @param size Size of the buffer. Use sizeof().
   */
  void make_motor_advertiser_left(char* buf, std::size_t size);

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

  void publish_motor_left(double rad_s) override;
  void publish_motor_right(double rad_s) override;

  /// gz node instance for receiving encoder feedback from Gazebo.
  gz::transport::Node gz_node_;
  /// Separate node for motor commands so subscriber callbacks cannot contend
  /// with the publisher path on the same transport node.
  gz::transport::Node motor_node_;
  // Right Encoder Publisher
  gz::transport::Node::Publisher right_motor_publisher;
  // Left Encoder Publisher
  gz::transport::Node::Publisher left_motor_publisher;
  /// Contains Sencoderlation Configuration (e.g. entity and world name)
  const sitl::SimConfig sitl_config_;
};

}  // namespace drivers
