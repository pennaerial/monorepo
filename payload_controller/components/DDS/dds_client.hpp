#pragma once

#include <uxr/client/client.h>

#include <cstdint>

#include "sensor_msgs/msg/Imu.h"

namespace dds_config
{

// Keep eight reliable frames so short transport stalls do not immediately drop IMU samples.
inline constexpr uint32_t STREAM_HISTORY = 8;

#if defined(UCLIENT_PROFILE_UDP)
inline constexpr uint32_t TRANSPORT_MTU = UXR_CONFIG_UDP_TRANSPORT_MTU;
#elif defined(UCLIENT_PROFILE_CUSTOM_TRANSPORT)
inline constexpr uint32_t TRANSPORT_MTU = UXR_CONFIG_CUSTOM_TRANSPORT_MTU;
#else
#error "No supported Micro-XRCE-DDS transport enabled"
#endif

// Micro-XRCE-DDS requires one MTU-sized slot for every reliable-history entry.
inline constexpr uint32_t BUFFER_SIZE = TRANSPORT_MTU * STREAM_HISTORY;

}  // namespace dds_config


class DDSClient
{
public:
  DDSClient(const char* ip, const char* port);

  /// Run the DDSClient
  void run();

  /// Publishes the latest IMU message when the DDS session is connected.
  void publish_imu(const sensor_msgs_msg_Imu& imu_msg);

private:
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

#if defined(UCLIENT_PROFILE_UDP)
  uxrUDPTransport transport_;
#elif defined(UCLIENT_PROFILE_CUSTOM_TRANSPORT)
  uxrCustomTransport transport_;
#endif
  /// the uxr session object. Interacts directly with DDS Agent
  uxrSession session_;

  /// DDS stream buffer for reliable output
  uint8_t output_reliable_stream_buffer_[dds_config::BUFFER_SIZE];
  /// uxrStreamId associated with output reliable buffer
  uxrStreamId reliable_out_;
  /// DDS stream buffer for reliable input
  uint8_t input_reliable_stream_buffer_[dds_config::BUFFER_SIZE];
  /// uxrStreamId associated with input reliable buffer
  uxrStreamId reliable_in_;

  uxrObjectId datawriter_id_;
  bool connected_{false};
};
