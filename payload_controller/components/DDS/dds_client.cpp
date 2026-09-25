#include "dds_client.hpp"

#include "esp_log.h"

#ifndef CONFIG_IDF_TARGET_LINUX
#include "uart_transport.hpp"
#endif


static const char* TAG = "DDSClient";
// TODO: This shouldn't be hardcoded in, should be derived by some vehicle-specific parameter.
// Need to set up a parameter system first
static constexpr uint32_t SESSION_KEY = 0xABCDABCD;

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

  participant_id_ = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
  publisher_id_ = uxr_object_id(DEFAULT_PUBLISHER_KEY, UXR_PUBLISHER_ID);
  subscriber_id_ = uxr_object_id(DEFAULT_SUBSCRIBER_KEY, UXR_SUBSCRIBER_ID);

  // Create one XRCE Publisher and one XRCE Subscriber. Individual topic entries
  // decide whether they need a DataWriter, a DataReader, or both.
  uint16_t participant_req = uxr_buffer_create_participant_bin(
      &session_, reliable_out_, participant_id_,
      0,  // DDS domain ID
      "default_xrce_participant", UXR_REPLACE
  );

  uint16_t publisher_req = uxr_buffer_create_publisher_bin(
      &session_, reliable_out_, publisher_id_, participant_id_, UXR_REPLACE
  );

  uint16_t subscriber_req = uxr_buffer_create_subscriber_bin(
      &session_, reliable_out_, subscriber_id_, participant_id_, UXR_REPLACE
  );

  /// INIT DONE

  constexpr std::size_t CREATE_REQUEST_COUNT = 3 + topic_count + datawriter_count() + (datareader_count() * 2);
  uint16_t requests[CREATE_REQUEST_COUNT]{};
  uint8_t status[CREATE_REQUEST_COUNT]{};
  std::size_t request_count = 0;

  requests[request_count++] = participant_req;
  requests[request_count++] = publisher_req;
  requests[request_count++] = subscriber_req;

  generate_topics(requests, request_count);
  generate_writers(requests, request_count);
  generate_readers(requests, request_count);

  datawriter_id_ = datawriter_id(IMU_TOPIC_INDEX); // REMOVE

  // Actually send the queued requests and wait for status responses from the
  // Agent. Buffer-create calls only enqueue work; this drives the session.
  if (!uxr_run_session_until_all_status(&session_, 1000, requests, status, request_count)) {
    ESP_LOGE(TAG, "Error creating DDS entities");
    return;
  }

  ESP_LOGI(TAG, "Entities creation success");
}

void DDSClient::generate_topics(uint16_t requests[], std::size_t& request_count)
{
  for (std::size_t topic_index = 0; topic_index < topic_count; ++topic_index) {
    const Topic& topic = topics[topic_index];
    /// TODO right now we create topics for both readers & writers, however it is likely that ROS will create the
    /// topics for our readers so we might want to only run the code below for writers
    ESP_LOGI(TAG, "Creating DDS topic %s (%s)", topic.topic_name, topic.type_name);
    requests[request_count++] = uxr_buffer_create_topic_bin(
        &session_, reliable_out_, topic_id(topic_index), participant_id_, topic.topic_name, topic.type_name, UXR_REPLACE
    );
  }
}

void DDSClient::generate_writers(uint16_t requests[], std::size_t& request_count)
{
  for (std::size_t topic_index = 0; topic_index < topic_count; ++topic_index) {
    const Topic& topic = topics[topic_index];
    if (topic.dir != Topic::Direction::WRITER) {
      continue;
    }

    ESP_LOGI(TAG, "Creating DDS datawriter %s", topic.name);
    requests[request_count++] = uxr_buffer_create_datawriter_bin(
        &session_, reliable_out_, datawriter_id(topic_index), publisher_id_, topic_id(topic_index), topic.qos, UXR_REPLACE
    );
  }
}

void DDSClient::generate_readers(uint16_t requests[], std::size_t& request_count)
{
  for (std::size_t topic_index = 0; topic_index < topic_count; ++topic_index) {
    const Topic& topic = topics[topic_index];
    if (topic.dir != Topic::Direction::READER) {
      continue;
    }

    const uxrObjectId reader_id = datareader_id(topic_index);
    ESP_LOGI(TAG, "Creating DDS datareader %s", topic.name);
    requests[request_count++] = uxr_buffer_create_datareader_bin(
        &session_, reliable_out_, reader_id, subscriber_id_, topic_id(topic_index), topic.qos, UXR_REPLACE
    );

    // Creates a request to forward data to us
    uxrDeliveryControl delivery_control{};
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    requests[request_count++] = uxr_buffer_request_data(&session_, reliable_out_, reader_id, reliable_in_, &delivery_control);
  }
}


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
  sensor_msgs_msg_Imu msg;
  sensor_msgs_msg_Imu_deserialize_topic(ub, &msg);

  ESP_LOGI(TAG, "orientation x: %f", msg.orientation.x);
  ESP_LOGI(TAG, "orientation y: %f", msg.orientation.y);
  ESP_LOGI(TAG, "orientation z: %f", msg.orientation.z);
  ESP_LOGI(TAG, "orientation w: %f", msg.orientation.w);
  ESP_LOGI(TAG, "handling topic..");
}


void DDSClient::update(const sensor_msgs_msg_Imu& msg)
{
  ESP_LOGI(TAG, "Updating");
  imu_msg = msg;
  ucdrBuffer ub;
  uint32_t topic_size = sensor_msgs_msg_Imu_size_of_topic(&imu_msg, 0);

  // Reserve space in the reliable output stream for one DataWriter sample and
  // attach a CDR buffer that the generated serializer can write into.
  uxr_prepare_output_stream(&session_, reliable_out_, datawriter_id_, &ub, topic_size);
  sensor_msgs_msg_Imu_serialize_topic(&ub, &imu_msg);

  // Flush the reliable output stream and wait for the Agent to acknowledge the
  // write, then briefly spin the session so input traffic/callbacks are served.
  uxr_run_session_until_confirm_delivery(&session_, 1000);
  uxr_run_session_time(&session_, 10);
}
