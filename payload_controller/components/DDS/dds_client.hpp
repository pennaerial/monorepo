#pragma once

#include <uxr/client/client.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "sdkconfig.h"
#include "topics.h"

/// Number of reliable XRCE stream blocks kept in the output/input history.
constexpr uint32_t STREAM_HISTORY = 8;

#if defined(CONFIG_IDF_TARGET_LINUX)
/// MTU used by the selected XRCE transport backend.
constexpr uint32_t TRANSPORT_MTU = UXR_CONFIG_UDP_TRANSPORT_MTU;
#else
/// MTU used by the selected XRCE transport backend.
constexpr uint32_t TRANSPORT_MTU = UXR_CONFIG_CUSTOM_TRANSPORT_MTU;
#endif

/// Total bytes reserved for each reliable XRCE stream buffer.
constexpr uint32_t BUFFER_SIZE = TRANSPORT_MTU * STREAM_HISTORY;
/// Bytes available in one reliable stream history block.
constexpr uint32_t RELIABLE_STREAM_BLOCK_SIZE = BUFFER_SIZE / STREAM_HISTORY;
/// Conservative allowance for XRCE write framing when checking one-block writes.
constexpr uint32_t ESTIMATED_XRCE_WRITE_OVERHEAD = 32;


class DDSClient
{
public:
  // Called from update()/XRCE session servicing when a configured READER topic
  // receives data. `msg` points to the deserialized generated message type
  // associated with `topic`, and is only valid for the duration of the call.
  using ReaderCallback = void (*)(const Topic& topic, const void* msg, uint16_t length, void* args);

  static DDSClient& instance();

  /// Opens transport/session streams and creates the configured XRCE-DDS entities.
  void init();

  /// Writes a sample for a writer topic matched by TopicId.
  bool publish(TopicId topic_id, const void* msg);

  /// Services incoming data and reliable stream bookkeeping.
  void update();

  /// Binds a callback for a reader topic matched by name or TopicId.
  bool set_reader_callback(TopicId topic_id, ReaderCallback callback, void* args);

private:
  /// Stores the Agent address used when init() opens the XRCE transport.
  DDSClient(const char* ip, const char* port);
  DDSClient(const DDSClient&) = delete;
  DDSClient& operator=(const DDSClient&) = delete;

  /// Queues XRCE Topic create requests for every configured topic.
  void generate_topics(uint16_t requests[], std::size_t& request_count, uxrObjectId participant_id);
  /// Queues XRCE DataWriter create requests for configured writer topics.
  void generate_writers(uint16_t requests[], std::size_t& request_count, uxrObjectId publisher_id);
  /// Queues XRCE DataReader create and request-data requests for configured reader topics.
  void generate_readers(uint16_t requests[], std::size_t& request_count, uxrObjectId subscriber_id);

  /// Runs the XRCE session until reliable output delivery is confirmed or times out.
  bool confirm_delivery();

  /// Static XRCE topic callback that routes incoming samples back to this DDSClient.
  static void on_topic_callback(
      uxrSession* session,
      uxrObjectId object_id,
      uint16_t request_id,
      uxrStreamId stream_id,
      ucdrBuffer* ub,
      uint16_t length,
      void* args
  );

  /// Deserializes an incoming reader sample and forwards it to the bound callback.
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

  bool delivery_pending_ = false;
  std::array<ReaderCallback, TOPIC_COUNT> reader_callbacks_{};
  std::array<void*, TOPIC_COUNT> reader_callback_args_{};
};
