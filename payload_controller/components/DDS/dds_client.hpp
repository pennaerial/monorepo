#pragma once

#include <uxr/client/client.h>

#include <cstdint>

#include "sdkconfig.h"
#include "sensor_msgs/msg/Imu.h"

// Keep eight reliable frames so short transport stalls do not immediately drop IMU samples.
constexpr uint32_t STREAM_HISTORY = 8;

#if defined(CONFIG_IDF_TARGET_LINUX)
constexpr uint32_t TRANSPORT_MTU = UXR_CONFIG_UDP_TRANSPORT_MTU;
#else
constexpr uint32_t TRANSPORT_MTU = UXR_CONFIG_CUSTOM_TRANSPORT_MTU;
#endif

// Micro-XRCE-DDS requires one MTU-sized slot for every reliable-history entry.
constexpr uint32_t BUFFER_SIZE = TRANSPORT_MTU * STREAM_HISTORY;


class DDSClient
{
public:
  DDSClient(const char* ip, const char* port);

  /// Run the DDSClient
  void run();
  /// updates all internal msgs
  void update(const sensor_msgs_msg_Imu& msg);

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

  uxrObjectId datareader_id_;
  uxrObjectId datawriter_id_;
  bool connected_{false};

  /// IMU msg sent to DDS agent
  sensor_msgs_msg_Imu imu_msg{};
};
