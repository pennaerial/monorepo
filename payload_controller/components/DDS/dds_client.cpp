#include "dds_client.hpp"

#include "esp_log.h"

#if defined(UCLIENT_PROFILE_CUSTOM_TRANSPORT)
#include "dds_uart_transport.hpp"
#endif


namespace
{

const char* TAG = "DDSClient";
// Placeholder XRCE client key until the vehicle parameter system can provide a unique 32-bit value.
constexpr uint32_t SESSION_KEY = 0xABCDABCD;
// All DDS objects belong to the first object instance in this client session.
constexpr uint8_t OBJECT_INSTANCE_ID = 0x01;
// Use the default DDS domain until domain selection becomes a vehicle parameter.
constexpr uint16_t DDS_DOMAIN_ID = 0;
// Bound agent handshakes and reliable delivery so a disconnected agent cannot block forever.
constexpr int SESSION_TIMEOUT_MS = 1000;
// Participant, topic, publisher, subscriber, writer, and reader are created together.
constexpr uint16_t ENTITY_COUNT = 6;

}  // namespace

DDSClient::DDSClient(const char* ip, const char* port) : ip_(ip), port_(port) {}

void DDSClient::run()
{
  // Host simulation talks to a local UDP agent; ESP32-S3 uses the board's dedicated UART link.
#if defined(UCLIENT_PROFILE_UDP)
  if (!uxr_init_udp_transport(&transport_, UXR_IPv4, ip_, port_)) {
    ESP_LOGE(TAG, "UXR UDP transport failed to init!");
    return;
  }
  ESP_LOGI(TAG, "UXR UDP transport init success!");
#elif defined(UCLIENT_PROFILE_CUSTOM_TRANSPORT)
  if (!dds_uart_transport::initialize(transport_)) {
    ESP_LOGE(TAG, "UXR UART transport failed to init!");
    return;
  }
  ESP_LOGI(
      TAG, "UXR UART transport init success on TX=%d RX=%d", dds_uart_transport::TX_GPIO, dds_uart_transport::RX_GPIO
  );
#endif

  uxr_init_session(&session_, &transport_.comm, SESSION_KEY);
  uxr_set_topic_callback(&session_, on_topic_callback, this);
  if (!uxr_create_session(&session_)) {
    ESP_LOGE(TAG, "Error creating session");
#if defined(UCLIENT_PROFILE_CUSTOM_TRANSPORT)
    dds_uart_transport::close(transport_);
#endif
    return;
  }
  ESP_LOGI(TAG, "UXR Session created");


  reliable_out_ = uxr_create_output_reliable_stream(
      &session_, output_reliable_stream_buffer_, dds_config::BUFFER_SIZE, dds_config::STREAM_HISTORY
  );

  reliable_in_ = uxr_create_input_reliable_stream(
      &session_, input_reliable_stream_buffer_, dds_config::BUFFER_SIZE, dds_config::STREAM_HISTORY
  );

  uxrObjectId participant_id = uxr_object_id(OBJECT_INSTANCE_ID, UXR_PARTICIPANT_ID);
  const char* participant_xml =
      "<dds>"
      "<participant>"
      "<rtps>"
      "<name>default_xrce_participant</name>"
      "</rtps>"
      "</participant>"
      "</dds>";
  uint16_t participant_req = uxr_buffer_create_participant_xml(
      &session_, reliable_out_, participant_id, DDS_DOMAIN_ID, participant_xml, UXR_REPLACE
  );

  uxrObjectId topic_id = uxr_object_id(OBJECT_INSTANCE_ID, UXR_TOPIC_ID);
  const char* topic_xml =
      "<dds>"
      "<topic>"
      // "<name>HelloWorldTopic</name>"
      // "<dataType>HelloWorld</dataType>"
      "<name>rt/imu</name>"  // ROS naming conventions in DDS namespace
      "<dataType>sensor_msgs::msg::dds_::Imu_</dataType>"
      "</topic>"
      "</dds>";
  uint16_t topic_req =
      uxr_buffer_create_topic_xml(&session_, reliable_out_, topic_id, participant_id, topic_xml, UXR_REPLACE);


  uxrObjectId publisher_id = uxr_object_id(OBJECT_INSTANCE_ID, UXR_PUBLISHER_ID);
  const char* publisher_xml = "";
  uint16_t publisher_req = uxr_buffer_create_publisher_xml(
      &session_, reliable_out_, publisher_id, participant_id, publisher_xml, UXR_REPLACE
  );

  uxrObjectId subscriber_id = uxr_object_id(OBJECT_INSTANCE_ID, UXR_SUBSCRIBER_ID);
  const char* subscriber_xml = "";
  uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(
      &session_, reliable_out_, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE
  );

  datawriter_id_ = uxr_object_id(OBJECT_INSTANCE_ID, UXR_DATAWRITER_ID);
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
  uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(
      &session_, reliable_out_, datawriter_id_, publisher_id, datawriter_xml, UXR_REPLACE
  );

  uxrObjectId datareader_id = uxr_object_id(OBJECT_INSTANCE_ID, UXR_DATAREADER_ID);
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
  uint16_t datareader_req = uxr_buffer_create_datareader_xml(
      &session_, reliable_out_, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE
  );

  // create requester and replier...

  // Create entities
  uint8_t status[ENTITY_COUNT];
  uint16_t requests[ENTITY_COUNT] = {participant_req, topic_req,      publisher_req,
                                     subscriber_req,  datawriter_req, datareader_req};

  if (!uxr_run_session_until_all_status(&session_, SESSION_TIMEOUT_MS, requests, status, ENTITY_COUNT)) {
    ESP_LOGE(TAG, "Error creating DDS entities");
    return;
  }

  // Publishing is enabled only after the agent confirms every required DDS entity.
  connected_ = true;
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
  ESP_LOGI(TAG, "handling topic..");
}


void DDSClient::update(const sensor_msgs_msg_Imu& imu_msg)
{
  if (!connected_) {
    return;
  }

  ucdrBuffer ub;
  uint32_t topic_size = sensor_msgs_msg_Imu_size_of_topic(&imu_msg, 0);
  uxr_prepare_output_stream(&session_, reliable_out_, datawriter_id_, &ub, topic_size);
  sensor_msgs_msg_Imu_serialize_topic(&ub, &imu_msg);

  uxr_run_session_until_confirm_delivery(&session_, SESSION_TIMEOUT_MS);
}
