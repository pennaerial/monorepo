#pragma once

#include "esp_log.h"
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
  template<TopicId Id>
  bool publish(const typename TopicMsg<Id>::Message& msg);

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

template<TopicId Id>
bool DDSClient::publish(const typename TopicMsg<Id>::Message& msg)
{
  constexpr const char* TAG = "DDSClient";
  constexpr std::size_t topic_index = TopicMsg<Id>::index;
  const Topic& topic = TOPICS[topic_index];

  if (topic.dir != Topic::Direction::WRITER) {
    ESP_LOGE(TAG, "DDS topic %s is not configured as a writer", topic.name);
    return false;
  }

  if (topic.size_of_topic == nullptr || topic.serialize_topic == nullptr) {
    ESP_LOGE(TAG, "DDS topic %s is missing serializer hooks", topic.name);
    return false;
  }

  ucdrBuffer ub;
  const uint32_t topic_size = topic.size_of_topic(&msg, 0);
  if (topic_size + ESTIMATED_XRCE_WRITE_OVERHEAD > RELIABLE_STREAM_BLOCK_SIZE) {
    ESP_LOGE(TAG, "DDS topic %s is too large for one XRCE reliable stream block", topic.name);
    return false;
  }

  uint16_t request_id =
      uxr_prepare_output_stream(&session_, reliable_out_, datawriter_id(topic_index), &ub, topic_size);
  if (request_id == UXR_INVALID_REQUEST_ID) {
    ESP_LOGW(TAG, "XRCE reliable output stream is full; dropping DDS topic %s", topic.name);
    return false;
  }

  if (!topic.serialize_topic(&ub, &msg)) {
    ESP_LOGE(TAG, "Failed to serialize DDS topic %s", topic.name);
    return false;
  }

  delivery_pending_ = true;
  return true;
}
