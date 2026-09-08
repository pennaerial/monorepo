#pragma once

#include <uxr/client/client.h>
#include <uxr/client/config.h>
#include <cstdint>

#define STREAM_HISTORY  8
#define BUFFER_SIZE     UXR_CONFIG_UDP_TRANSPORT_MTU* STREAM_HISTORY

enum class TransportType {
  SERIAL,
  UDP,
};

class DDSClient
{
public:
  DDSClient(TransportType transport, const char* ip, const char* port);

  /// Run the DDSClient
  void run();

private:
  /// callback function for receiving a topic. Recreates the DDSClient instance with void* args and calls handle_topic
  static void on_topic_callback(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, ucdrBuffer* ub, uint16_t length, void* args);

  /// Finishes handling the topic
  void handle_topic(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, ucdrBuffer* ub, uint16_t length);

  /// TODO: Compute a 32 bit unique key
  // uint32_t unique_key();

  /// transport type of client, UDP or SERIAL
  TransportType transport_;
  /// ip for UDP transport
  const char* ip_;
  /// port for UDP transport
  const char* port_;

  uxrObjectId participant_id_;

  /// UDP transport object
  uxrUDPTransport transport_udp_{};
  /// the uxr session object. Interacts directly with DDS Agent
  uxrSession session_;
  /// Stream buffer for 
  uint8_t output_reliable_stream_buffer_[BUFFER_SIZE];
  ///
  uint8_t input_reliable_stream_buffer_[BUFFER_SIZE];



};
