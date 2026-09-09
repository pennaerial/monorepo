#include "dds_client.hpp"
#include "esp_log.h"

// #include "esp_mac.h"


static const char* TAG = "DDSClient";

DDSClient::DDSClient(TransportType transport, const char* ip, const char* port)
    : transport_(transport), ip_(ip), port_(port)
{
}

void DDSClient::run()
{
  if (!uxr_init_udp_transport(&transport_udp_, UXR_IPv4, ip_, port_)) {
    ESP_LOGE(TAG, "UXR UDP transport failed to init!");
    return;
  }
  ESP_LOGI(TAG, "UXR UDP transport init success!");

  uxr_init_session(&session_, &transport_udp_.comm, 0xABCDABCD);
  uxr_set_topic_callback(&session_, on_topic_callback, this);
  if(!uxr_create_session(&session_)) {
    ESP_LOGI(TAG, "Error creating session");
    return;
  }
  ESP_LOGI(TAG, "UXR Session created");



  reliable_out_ = uxr_create_output_reliable_stream(&session_, output_reliable_stream_buffer_, BUFFER_SIZE, STREAM_HISTORY);

  reliable_in_ = uxr_create_input_reliable_stream(&session_, input_reliable_stream_buffer_, BUFFER_SIZE, STREAM_HISTORY);

  uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
  const char* participant_xml = "<dds>"
                                  "<participant>"
                                      "<rtps>"
                                          "<name>default_xrce_participant</name>"
                                      "</rtps>"
                                  "</participant>"
                              "</dds>";
  uint16_t participant_req = uxr_buffer_create_participant_xml(
    &session_,
    reliable_out_,
    participant_id,
    0,                  // DDS domain ID
    participant_xml,
    UXR_REPLACE
  );

  uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
  const char* topic_xml = "<dds>"
                              "<topic>"
                                  // "<name>HelloWorldTopic</name>"
                                  // "<dataType>HelloWorld</dataType>"
                                  "<name>rt/imu</name>" // ROS naming conventions in DDS namespace
                                  "<dataType>sensor_msgs::msg::dds_::Imu_</dataType>"
                              "</topic>"
                          "</dds>";
  uint16_t topic_req = uxr_buffer_create_topic_xml(&session_, reliable_out_, topic_id, participant_id, topic_xml, UXR_REPLACE);



  uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
  const char* publisher_xml = "";
  uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session_, reliable_out_, publisher_id, participant_id, publisher_xml, UXR_REPLACE);

  uxrObjectId subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
  const char* subscriber_xml = "";
  uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(&session_, reliable_out_, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);

  datawriter_id_ = uxr_object_id(0x01, UXR_DATAWRITER_ID);
  const char* datawriter_xml = "<dds>"
                                 "<data_writer>"
                                     "<topic>"
                                         "<kind>NO_KEY</kind>"
                                         "<name>rt/imu</name>"
                                         "<dataType>sensor_msgs::msg::dds_::Imu_</dataType>"
                                     "</topic>"
                                 "</data_writer>"
                             "</dds>";
  uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(&session_, reliable_out_, datawriter_id_, publisher_id, datawriter_xml, UXR_REPLACE);

  uxrObjectId datareader_id = uxr_object_id(0x01, UXR_DATAREADER_ID);
  const char* datareader_xml = "<dds>"
                                 "<data_reader>"
                                     "<topic>"
                                         "<kind>NO_KEY</kind>"
                                         "<name>rt/imu</name>"
                                         "<dataType>sensor_msgs::msg::dds_::Imu_</dataType>"
                                     "</topic>"
                                 "</data_reader>"
                               "</dds>";
  uint16_t datareader_req = uxr_buffer_create_datareader_xml(&session_, reliable_out_, datareader_id, subscriber_id, datareader_xml, UXR_REPLACE);

  // create requester and replier...

  // Create entities
  uint8_t status[6];
  uint16_t requests[6] = { participant_req, topic_req, publisher_req, subscriber_req, datawriter_req, datareader_req };

  if (!uxr_run_session_until_all_status(&session_, 1000, requests, status, 6)) {
    ESP_LOGE(TAG, "Error at creating 6 entities");
    return;
  }

  ESP_LOGI(TAG, "Entities creation success");

}

void DDSClient::on_topic_callback(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, ucdrBuffer* ub, uint16_t length, void* args)
{
  DDSClient* client = static_cast<DDSClient*>(args);
  if (!client) {
    ESP_LOGE(TAG, "on_topic_callback: static_cast into DDSClient failed");
    return;
  }
  client->handle_topic(session, object_id, request_id, stream_id, ub, length);
}

void DDSClient::handle_topic(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, ucdrBuffer* ub, uint16_t length)
{
  ESP_LOGI(TAG, "handling topic..");
}


void DDSClient::update()
{
  ucdrBuffer ub;
  uint32_t topic_size = sensor_msgs_msg_Imu_size_of_topic(&imu_msg, 0);
  uxr_prepare_output_stream(&session_, reliable_out_, datawriter_id_, &ub, topic_size);
  sensor_msgs_msg_Imu_serialize_topic(&ub, &imu_msg);

  uxr_run_session_until_confirm_delivery(&session_, 1000);
}
