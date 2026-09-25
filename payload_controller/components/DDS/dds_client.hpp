#pragma once

#include <uxr/client/client.h>

#include <cstddef>
#include <cstdint>

#include "sdkconfig.h"
#include "sensor_msgs/msg/Imu.h"

#include "topics.h"

constexpr uint32_t STREAM_HISTORY = 8;

#if defined(CONFIG_IDF_TARGET_LINUX)
constexpr uint32_t TRANSPORT_MTU = UXR_CONFIG_UDP_TRANSPORT_MTU;
#else
constexpr uint32_t TRANSPORT_MTU = UXR_CONFIG_CUSTOM_TRANSPORT_MTU;
#endif

constexpr uint32_t BUFFER_SIZE = TRANSPORT_MTU * STREAM_HISTORY;


class DDSClient
{
public:
  DDSClient(const char* ip, const char* port);

  /// Set ourselves up as a Micro XRCE-DDS Client and register to the agent as a session
  /// Also sets us up as participant and generates the publisher and subscriber
  void init();

  /// TODO needs some sort of msg and enum id
  void publish();

  // TODO should get data from some sort of buffer with a specified enum
  void getData();

  /// updates all internal msgs
  void update(const sensor_msgs_msg_Imu& msg);

private:
  void generate_topics(uint16_t requests[], std::size_t& request_count);
  void generate_writers(uint16_t requests[], std::size_t& request_count);
  void generate_readers(uint16_t requests[], std::size_t& request_count);

  uxrObjectId datawriter_id_;
  uxrObjectId datareader_id_;

  /// IMU msg sent to DDS agent
  sensor_msgs_msg_Imu imu_msg{};

  /// callback function for receiving a topic. Recreates the DDSClient instance with void* args and calls handle_topic
  static void on_topic_callback(
      uxrSession* session,
      uxrObjectId object_id,
      uint16_t request_id,
      uxrStreamId stream_id,
      ucdrBuffer* ub,
      uint16_t length,
      void* args
  );

  /// Finishes handling the topic
  void handle_topic(
      uxrSession* session,
      uxrObjectId object_id,
      uint16_t request_id,
      uxrStreamId stream_id,
      ucdrBuffer* ub,
      uint16_t length
  );


  /// TODO: Compute a 32 bit unique key
  // uint32_t unique_key();

  /// ip for UDP transport
  const char* ip_;
  /// port for UDP transport
  const char* port_;
  /// participant ID registered with agent
  uxrObjectId participant_id_;
  /// default publisher used by the generated DataWriters
  uxrObjectId publisher_id_;
  /// default subscriber used by the generated DataReaders
  uxrObjectId subscriber_id_;

#if defined(CONFIG_IDF_TARGET_LINUX)
  uxrUDPTransport transport_;
#else
  uxrCustomTransport transport_;
#endif

  /// the uxr session object. Interacts directly with DDS Agent
  uxrSession session_;

  /// DDS stream buffer for reliable output
  uint8_t output_reliable_stream_buffer_[BUFFER_SIZE];
  /// uxrStreamId associated with output reliable buffer
  uxrStreamId reliable_out_;
  /// DDS stream buffer for reliable input
  uint8_t input_reliable_stream_buffer_[BUFFER_SIZE];
  /// uxrStreamId associated with input reliable buffer
  uxrStreamId reliable_in_;

};
