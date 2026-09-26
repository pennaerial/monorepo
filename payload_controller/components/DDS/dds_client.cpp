#include "dds_client.hpp"

#include "esp_log.h"

#include <cstring>

#ifndef CONFIG_IDF_TARGET_LINUX
#include "uart_transport.hpp"
#endif


static const char* TAG = "DDSClient";
// TODO: This shouldn't be hardcoded in, should be derived by some vehicle-specific parameter.
// Need to set up a parameter system first
static constexpr uint32_t SESSION_KEY = 0xABCDABCD;

namespace
{
// XRCE object IDs are a numeric key plus an object type. Both must match.
bool same_object_id(const uxrObjectId lhs, const uxrObjectId rhs)
{
  return lhs.id == rhs.id && lhs.type == rhs.type;
}
}

DDSClient::DDSClient(const char* ip, const char* port) : ip_(ip), port_(port) {}

void DDSClient::init()
{
  // Open the UDP link to the Micro-XRCE-DDS Agent. This is the transport
  // underneath the XRCE session; no DDS entities exist yet.

#if defined(CONFIG_IDF_TARGET_LINUX)
  if (!uxr_init_udp_transport(&transport_, UXR_IPv4, ip_, port_)) {
    ESP_LOGE(TAG, "UXR UDP transport failed to init!");
    return;
  }
  ESP_LOGI(TAG, "UXR UDP transport init success!");
#else  // init custom transport
  uxr_set_custom_transport_callbacks(&transport_, true, uart_open, uart_close, uart_write, uart_read);
  UartTransportConfig uart_config{};
  if (!uxr_init_custom_transport(&transport_, &uart_config)) {
    ESP_LOGE(TAG, "Custom UART transport failed to initialize!");
    return;
  }
  ESP_LOGE(TAG, "Custom UART transport successfully initialized");
#endif

  // Bind the transport to an XRCE session. The session key identifies this
  // client to the Agent, and the topic callback receives subscribed samples.
  uxr_init_session(&session_, &transport_.comm, SESSION_KEY);
  uxr_set_topic_callback(&session_, on_topic_callback, this);

  // Handshake with the Agent so later create/read/write requests have a live
  // XRCE session to run on.
  if (!uxr_create_session(&session_)) {
    ESP_LOGI(TAG, "Error creating session");
    return;
  }
  ESP_LOGI(TAG, "UXR Session created");

  // Reliable streams are XRCE queues. The output stream carries entity-create
  // requests and writes; the input stream carries data requested from readers.
  reliable_out_ =
      uxr_create_output_reliable_stream(&session_, output_reliable_stream_buffer_, BUFFER_SIZE, STREAM_HISTORY);

  reliable_in_ =
      uxr_create_input_reliable_stream(&session_, input_reliable_stream_buffer_, BUFFER_SIZE, STREAM_HISTORY);

  const uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
  const uxrObjectId publisher_id = uxr_object_id(DEFAULT_PUBLISHER_KEY, UXR_PUBLISHER_ID);
  const uxrObjectId subscriber_id = uxr_object_id(DEFAULT_SUBSCRIBER_KEY, UXR_SUBSCRIBER_ID);

  // Create one XRCE Publisher and one XRCE Subscriber. Topic entries decide
  // whether they need a DataWriter or DataReader.
  uint16_t participant_req = uxr_buffer_create_participant_bin(
      &session_, reliable_out_, participant_id,
      0,  // DDS domain ID
      "default_xrce_participant", UXR_REPLACE
  );

  uint16_t publisher_req = uxr_buffer_create_publisher_bin(
      &session_, reliable_out_, publisher_id, participant_id, UXR_REPLACE
  );

  uint16_t subscriber_req = uxr_buffer_create_subscriber_bin(
      &session_, reliable_out_, subscriber_id, participant_id, UXR_REPLACE
  );

  // Participant, one Publisher, one Subscriber, every Topic, every DataWriter,
  // and each DataReader plus its request_data stream request.
  constexpr std::size_t CREATE_REQUEST_COUNT = 3 + topic_count + datawriter_count() + (datareader_count() * 2);
  uint16_t requests[CREATE_REQUEST_COUNT]{};
  uint8_t status[CREATE_REQUEST_COUNT]{};
  std::size_t request_count = 0;

  requests[request_count++] = participant_req;
  requests[request_count++] = publisher_req;
  requests[request_count++] = subscriber_req;

  generate_topics(requests, request_count, participant_id);
  generate_writers(requests, request_count, publisher_id);
  generate_readers(requests, request_count, subscriber_id);

  // Actually send the queued requests and wait for status responses from the
  // Agent. Buffer-create calls only enqueue work; this drives the session.
  if (!uxr_run_session_until_all_status(&session_, 1000, requests, status, request_count)) {
    ESP_LOGE(TAG, "Error creating DDS entities");
    return;
  }

  ESP_LOGI(TAG, "Entities creation success");
}

void DDSClient::generate_topics(uint16_t requests[], std::size_t& request_count, const uxrObjectId participant_id)
{
  for (std::size_t topic_index = 0; topic_index < topic_count; ++topic_index) {
    const Topic& topic = topics[topic_index];
    // TODO we might not want to create topics for the datareaders as ros2 might create those. IDK
    // Create the XRCE topic once; readers and writers both reference this object.
    ESP_LOGI(TAG, "Creating DDS topic %s (%s)", topic.topic_name, topic.type_name);
    requests[request_count++] = uxr_buffer_create_topic_bin(
        &session_, reliable_out_, topic_id(topic_index), participant_id, topic.topic_name, topic.type_name, UXR_REPLACE
    );
  }
}

// Creates DataWriters only for topics marked as WRITER in topics.h.
void DDSClient::generate_writers(uint16_t requests[], std::size_t& request_count, const uxrObjectId publisher_id)
{
  for (std::size_t topic_index = 0; topic_index < topic_count; ++topic_index) {
    const Topic& topic = topics[topic_index];
    if (topic.dir != Topic::Direction::WRITER) {
      continue;
    }

    ESP_LOGI(TAG, "Creating DDS datawriter %s", topic.name);
    requests[request_count++] = uxr_buffer_create_datawriter_bin(
        &session_, reliable_out_, datawriter_id(topic_index), publisher_id, topic_id(topic_index), topic.qos, UXR_REPLACE
    );
  }
}

// Creates DataReaders only for topics marked as READER in topics.h.
void DDSClient::generate_readers(uint16_t requests[], std::size_t& request_count, const uxrObjectId subscriber_id)
{
  for (std::size_t topic_index = 0; topic_index < topic_count; ++topic_index) {
    const Topic& topic = topics[topic_index];
    if (topic.dir != Topic::Direction::READER) {
      continue;
    }

    const uxrObjectId reader_id = datareader_id(topic_index);
    ESP_LOGI(TAG, "Creating DDS datareader %s", topic.name);
    requests[request_count++] = uxr_buffer_create_datareader_bin(
        &session_, reliable_out_, reader_id, subscriber_id, topic_id(topic_index), topic.qos, UXR_REPLACE
    );

    // A DataReader exists after create_datareader_bin(), but data will not be
    // delivered to on_topic_callback until we request it from the Agent.
    uxrDeliveryControl delivery_control{};
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    requests[request_count++] = uxr_buffer_request_data(&session_, reliable_out_, reader_id, reliable_in_, &delivery_control);
  }
}

bool DDSClient::topic_matches(const Topic& topic, const char* topic_name) const
{
  return topic_name != nullptr &&
      (std::strcmp(topic.name, topic_name) == 0 || std::strcmp(topic.topic_name, topic_name) == 0);
}

const Topic* DDSClient::find_topic(const char* topic_name, std::size_t& topic_index) const
{
  for (std::size_t i = 0; i < topic_count; ++i) {
    if (topic_matches(topics[i], topic_name)) {
      topic_index = i;
      return &topics[i];
    }
  }

  return nullptr;
}

// Public publish API: validate the named topic and copy the typed message into
// a byte queue. Serialization intentionally happens later in update().
bool DDSClient::publish(const char* topic_name, const void* msg)
{
  std::size_t topic_index = 0;
  const Topic* topic = find_topic(topic_name, topic_index);
  if (topic == nullptr) {
    ESP_LOGE(TAG, "Unknown DDS topic %s", topic_name == nullptr ? "<null>" : topic_name);
    return false;
  }

  if (topic->dir != Topic::Direction::WRITER) {
    ESP_LOGE(TAG, "DDS topic %s is not configured as a writer", topic->name);
    return false;
  }

  if (msg == nullptr) {
    ESP_LOGE(TAG, "DDS publish called with null data for %s", topic->name);
    return false;
  }

  // Copy the typed message now so callers can publish stack/local data safely.
  PendingPublish& pending = pending_publishes_.emplace_back();
  pending.topic_index = topic_index;
  pending.data.resize(topic->message_size);
  std::memcpy(pending.data.data(), msg, topic->message_size);
  return true;
}

void DDSClient::update()
{
  bool wrote_data = false;
  std::size_t writes_since_confirm = 0;
  ESP_LOGI(TAG, "Pending messages: %zu", pending_publishes_.size());

  // Drain every queued message, preserving publish() call order across topics.
  // A reliable stream with history N has N blocks, so confirm delivery before
  // queuing more than STREAM_HISTORY normal writes into the stream.
  for (const PendingPublish& pending : pending_publishes_) {
    if (writes_since_confirm >= STREAM_HISTORY) {
      confirm_delivery();
      writes_since_confirm = 0;
    }

    if (send_publish(pending.topic_index, pending.data.data())) {
      wrote_data = true;
      ++writes_since_confirm;
    }
  }
  pending_publishes_.clear();

  if (wrote_data) {
    confirm_delivery();
  }

  uxr_run_session_time(&session_, 10);
}

// Serializes one queued message into the XRCE reliable output stream.
bool DDSClient::send_publish(const std::size_t topic_index, const void* msg)
{
  const Topic& topic = topics[topic_index];
  if (topic.size_of_topic == nullptr || topic.serialize_topic == nullptr) {
    ESP_LOGE(TAG, "DDS topic %s is missing serializer hooks", topic.name);
    return false;
  }

  ucdrBuffer ub;
  const uint32_t topic_size = topic.size_of_topic(msg, 0);
  const bool use_fragmented_stream =
      topic_size + ESTIMATED_XRCE_WRITE_OVERHEAD > RELIABLE_STREAM_BLOCK_SIZE;

  uint16_t request_id = UXR_INVALID_REQUEST_ID;
  if (use_fragmented_stream) {
    request_id = uxr_prepare_output_stream_fragmented(
        &session_, reliable_out_, datawriter_id(topic_index), &ub, topic_size, flush_output_stream, this
    );
  } else {
    request_id = uxr_prepare_output_stream(&session_, reliable_out_, datawriter_id(topic_index), &ub, topic_size);

    // If the stream is full earlier than our batch estimate, flush and retry.
    if (request_id == UXR_INVALID_REQUEST_ID) {
      confirm_delivery();
      request_id = uxr_prepare_output_stream(&session_, reliable_out_, datawriter_id(topic_index), &ub, topic_size);
    }

    // If the message still cannot fit in one reliable block, fall back to the
    // fragmented path so large topic samples can span multiple blocks.
    if (request_id == UXR_INVALID_REQUEST_ID) {
      request_id = uxr_prepare_output_stream_fragmented(
          &session_, reliable_out_, datawriter_id(topic_index), &ub, topic_size, flush_output_stream, this
      );
    }
  }

  if (request_id == UXR_INVALID_REQUEST_ID) {
    ESP_LOGE(TAG, "Failed to reserve XRCE output stream space for %s", topic.name);
    return false;
  }

  if (!topic.serialize_topic(&ub, msg)) {
    ESP_LOGE(TAG, "Failed to serialize DDS topic %s", topic.name);
    return false;
  }

  return true;
}

bool DDSClient::confirm_delivery()
{
  const bool delivered = uxr_run_session_until_confirm_delivery(&session_, 1000);
  if (!delivered) {
    ESP_LOGW(TAG, "Timed out waiting for XRCE reliable output delivery");
  }
  return delivered;
}

bool DDSClient::flush_output_stream(uxrSession* session, void* args)
{
  DDSClient* client = static_cast<DDSClient*>(args);
  if (client == nullptr) {
    return false;
  }
  return uxr_run_session_until_confirm_delivery(session, 1000);
}

// Optional application hook for one received READER topic.
bool DDSClient::set_reader_callback(const char* topic_name, ReaderCallback callback, void* args)
{
  for (std::size_t topic_index = 0; topic_index < topic_count; ++topic_index) {
    const Topic& topic = topics[topic_index];
    if (topic.dir != Topic::Direction::READER || !topic_matches(topic, topic_name)) {
      continue;
    }

    reader_callbacks_[topic_index] = callback;
    reader_callback_args_[topic_index] = args;
    return true;
  }

  ESP_LOGE(TAG, "Unknown DDS reader topic %s", topic_name == nullptr ? "<null>" : topic_name);
  return false;
}

// Static C callback required by Micro XRCE-DDS; routes back to this instance.
void DDSClient::on_topic_callback(
    uxrSession* session,
    uxrObjectId object_id,
    uint16_t request_id,
    uxrStreamId stream_id,
    ucdrBuffer* ub,
    uint16_t length,
    void* args
)
{
  DDSClient* client = static_cast<DDSClient*>(args);
  if (!client) {
    ESP_LOGE(TAG, "on_topic_callback: static_cast into DDSClient failed");
    return;
  }
  client->handle_topic(session, object_id, request_id, stream_id, ub, length);
}

void DDSClient::handle_topic(
    uxrSession* session,
    uxrObjectId object_id,
    uint16_t request_id,
    uxrStreamId stream_id,
    ucdrBuffer* ub,
    uint16_t length
)
{
  (void)session;
  (void)request_id;
  (void)stream_id;

  // Match the incoming XRCE DataReader ID back to the configured Topic.
  for (std::size_t topic_index = 0; topic_index < topic_count; ++topic_index) {
    const Topic& topic = topics[topic_index];
    if (topic.dir != Topic::Direction::READER || !same_object_id(object_id, datareader_id(topic_index))) {
      continue;
    }

    if (topic.deserialize_topic == nullptr) {
      ESP_LOGE(TAG, "DDS topic %s is missing deserializer hook", topic.name);
      return;
    }

    // The deserialized message has the generated C type associated with topic.
    // It is stack-owned and only valid until reader_callback_ returns.
    alignas(std::max_align_t) uint8_t msg_buffer[max_topic_message_size()];
    if (!topic.deserialize_topic(ub, msg_buffer)) {
      ESP_LOGE(TAG, "Failed to deserialize DDS topic %s", topic.name);
      return;
    }

    ReaderCallback callback = reader_callbacks_[topic_index];
    if (callback != nullptr) {
      callback(topic, msg_buffer, length, reader_callback_args_[topic_index]);
    }
    return;
  }

  ESP_LOGW(TAG, "Received data for unknown DDS DataReader id=%u type=%u", object_id.id, object_id.type);
}
