#include "dds_client.hpp"
#include "esp_log.h"
#include "esp_mac.h"

#define STREAM_HISTORY  8
#define BUFFER_SIZE     UXR_CONFIG_UDP_TRANSPORT_MTU* STREAM_HISTORY

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



  uxrSession session = session_;
  uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
  uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

  uint8_t input_reliable_stream_buffer[BUFFER_SIZE];
  uxrStreamId reliable_in = uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

  uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
  const char* participant_xml = "<dds>"
                                  "<participant>"
                                      "<rtps>"
                                          "<name>default_xrce_participant</name>"
                                      "</rtps>"
                                  "</participant>"
                              "</dds>";
  uint16_t participant_req = uxr_buffer_create_participant_ref(
    &session,
    reliable_out,
    participant_id,
    0,                  // DDS domain ID
    participant_xml,
    UXR_REPLACE
  );

  uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
  const char* topic_xml = "<dds>"
                              "<topic>"
                                  "<name>HelloWorldTopic</name>"
                                  "<dataType>HelloWorld</dataType>"
                              "</topic>"
                          "</dds>";
  uint16_t topic_req = uxr_buffer_create_topic_xml(&session, reliable_out, topic_id, participant_id, topic_xml, UXR_REPLACE);



  uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
  const char* publisher_xml = "";
  uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session, reliable_out, publisher_id, participant_id, publisher_xml, UXR_REPLACE);

  uxrObjectId subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
  const char* subscriber_xml = "";
  uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(&session, reliable_out, subscriber_id, participant_id, subscriber_xml, UXR_REPLACE);
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
