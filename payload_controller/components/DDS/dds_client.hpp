#pragma once

#include <uxr/client/client.h>

#include <cstddef>
#include <cstdint>

#include "sdkconfig.h"
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
  using ReaderCallback = void (*)(const Topic& topic, const void* msg, uint16_t length, void* args);

  DDSClient(const char* ip, const char* port);

  /// Set ourselves up as a Micro XRCE-DDS Client and register to the agent as a session.
  void init();

  /// Queues a topic sample for writing. The data must remain valid until update() runs.
  bool publish(const char* topic_name, const void* msg);

  /// Writes queued publishes and services incoming data.
  void update();

  /// Forwards received and deserialized reader samples to application code.
  void set_reader_callback(ReaderCallback callback, void* args);

private:
  void generate_topics(uint16_t requests[], std::size_t& request_count, uxrObjectId participant_id);
  void generate_writers(uint16_t requests[], std::size_t& request_count, uxrObjectId publisher_id);
  void generate_readers(uint16_t requests[], std::size_t& request_count, uxrObjectId subscriber_id);

  bool topic_matches(const Topic& topic, const char* topic_name) const;
  const Topic* find_topic(const char* topic_name, std::size_t& topic_index) const;
  bool send_publish(std::size_t topic_index, const void* msg);

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

  const void* pending_publishes_[topic_count]{};
  ReaderCallback reader_callback_ = nullptr;
  void* reader_callback_args_ = nullptr;
};
