#pragma once

#include <uxr/client/client.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "sdkconfig.h"
#include "static_mutex.hpp"
#include "topics.h"

constexpr uint32_t STREAM_HISTORY = 8;

#if defined(CONFIG_IDF_TARGET_LINUX)
constexpr uint32_t TRANSPORT_MTU = UXR_CONFIG_UDP_TRANSPORT_MTU;
#else
constexpr uint32_t TRANSPORT_MTU = UXR_CONFIG_CUSTOM_TRANSPORT_MTU;
#endif

constexpr uint32_t BUFFER_SIZE = TRANSPORT_MTU * STREAM_HISTORY;
constexpr uint32_t RELIABLE_STREAM_BLOCK_SIZE = BUFFER_SIZE / STREAM_HISTORY;
constexpr uint32_t ESTIMATED_XRCE_WRITE_OVERHEAD = 32;


class DDSClient
{
public:
  // Called from update()/XRCE session servicing when a configured READER topic
  // receives data. `msg` points to the deserialized generated message type
  // associated with `topic`, and is only valid for the duration of the call.
  using ReaderCallback = void (*)(const Topic& topic, const void* msg, uint16_t length, void* args);

  DDSClient(const char* ip, const char* port);

  /// Set ourselves up as a Micro XRCE-DDS Client and register to the agent as a session.
  void init();

  /// Queues a topic sample for writing. Data is copied into an internal queue,
  /// so callers may pass stack/local messages safely.
  bool publish(const char* topic_name, const void* msg);

  /// Writes all queued publishes, clears the queue, and services incoming data.
  void update();

  /// Binds a callback for one configured READER topic.
  bool set_reader_callback(const char* topic_name, ReaderCallback callback, void* args);

private:
  // These queue XRCE CREATE/request messages; init() later flushes and waits for status.
  void generate_topics(uint16_t requests[], std::size_t& request_count, uxrObjectId participant_id);
  void generate_writers(uint16_t requests[], std::size_t& request_count, uxrObjectId publisher_id);
  void generate_readers(uint16_t requests[], std::size_t& request_count, uxrObjectId subscriber_id);

  // Topic lookup accepts either the friendly name, e.g. "IMU", or DDS name, e.g. "rt/imu".
  bool topic_matches(const Topic& topic, const char* topic_name) const;
  const Topic* find_topic(const char* topic_name, std::size_t& topic_index) const;
  bool send_publish(std::size_t topic_index, const void* msg);
  bool confirm_delivery();
  static bool flush_output_stream(uxrSession* session, void* args);

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

  // Stores copied message bytes so multiple publish() calls, including multiple
  // messages for the same topic, can be drained in order by update().
  struct PendingPublish {
    std::size_t topic_index;
    std::vector<uint8_t> data;
  };

  std::vector<PendingPublish> pending_publishes_;
  util::StaticMutex pending_publishes_mtx_;
  std::array<ReaderCallback, topic_count> reader_callbacks_{};
  std::array<void*, topic_count> reader_callback_args_{};
};
