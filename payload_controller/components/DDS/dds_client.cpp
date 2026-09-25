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

void DDSClient::run()
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

  uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
  const char* participant_xml =
      "<dds>"
      "<participant>"
      "<rtps>"
      "<name>default_xrce_participant</name>"
      "</rtps>"
      "</participant>"
      "</dds>";
  // Queue creation of the DDS Participant. The returned request id is checked
  // later when the session is run and the Agent replies with creation status.
  uint16_t participant_req = uxr_buffer_create_participant_xml(
      &session_, reliable_out_, participant_id,
      0,  // DDS domain ID
      participant_xml, UXR_REPLACE
  );

  uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
  const char* topic_xml =
      "<dds>"
      "<topic>"
      // "<name>HelloWorldTopic</name>"
      // "<dataType>HelloWorld</dataType>"
      "<name>rt/imu</name>"  // ROS naming conventions in DDS namespace
      "<dataType>sensor_msgs::msg::dds_::Imu_</dataType>"
      "</topic>"
      "</dds>";
  // Queue creation of the DDS Topic that both the reader and writer will use.
  uint16_t topic_req =
      uxr_buffer_create_topic_xml(&session_, reliable_out_, topic_id, participant_id, topic_xml, UXR_REPLACE);


  uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
  const char* publisher_xml = "";
  // Queue creation of a Publisher entity under the participant. An empty XML
  // string asks the Agent to use defaults.
  uint16_t publisher_req = uxr_buffer_create_publisher_xml(
      &session_, reliable_out_, publisher_id, participant_id, publisher_xml, UXR_REPLACE
  );

  uxrObjectId subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
  const char* subscriber_xml = "";
  // Queue creation of a Subscriber entity under the participant.
  uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(
      &session_, reliable_out_, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE
  );

  datawriter_id_ = uxr_object_id(0x01, UXR_DATAWRITER_ID);
  const char* datawriter_xml =
      "<dds>"
      "<data_writer>"
      "<topic>"
      "<kind>NO_KEY</kind>"
      "<name>rt/imu</name>"
      "<dataType>sensor_msgs::msg::dds_::Imu_</dataType>"
      "</topic>"
      "</data_writer>"
      "</dds>";
  // Queue creation of the DataWriter used by update() to publish IMU samples.
  uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(
      &session_, reliable_out_, datawriter_id_, publisher_id, datawriter_xml, UXR_REPLACE
  );

  datareader_id_ = uxr_object_id(0x01, UXR_DATAREADER_ID);
  const char* datareader_xml =
      "<dds>"
      "<data_reader>"
      "<topic>"
      "<kind>NO_KEY</kind>"
      "<name>rt/imu</name>"
      "<dataType>sensor_msgs::msg::dds_::Imu_</dataType>"
      "</topic>"
      "</data_reader>"
      "</dds>";
  // Queue creation of the DataReader used to receive samples through the topic
  // callback registered above.
  uint16_t datareader_req = uxr_buffer_create_datareader_xml(
      &session_, reliable_out_, datareader_id_, subscriber_id, datareader_xml, UXR_REPLACE
  );

  uxrDeliveryControl delivery_control{};
  delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
  // Queue a read request on the DataReader. Incoming samples for this request
  // are delivered on reliable_in_ and then dispatched to on_topic_callback.
  uint16_t read_data_req =
      uxr_buffer_request_data(&session_, reliable_out_, datareader_id_, reliable_in_, &delivery_control);

  // Create entities
  uint8_t status[7];
  uint16_t requests[7] = {participant_req, topic_req,      publisher_req, subscriber_req,
                          datawriter_req,  datareader_req, read_data_req};

  // Actually send the queued requests and wait for status responses from the
  // Agent. Buffer-create calls only enqueue work; this drives the session.
  if (!uxr_run_session_until_all_status(&session_, 1000, requests, status, 6)) {
    ESP_LOGE(TAG, "Error at creating 6 entities");
    return;
  }

  ESP_LOGI(TAG, "Entities creation success");
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
