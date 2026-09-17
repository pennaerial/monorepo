#include "dds_client.hpp"

#include "esp_log.h"

#if defined(UCLIENT_PROFILE_CUSTOM_TRANSPORT)
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#endif


static const char* TAG = "DDSClient";
// TODO: This shouldn't be hardcoded in, should be derived by some vehicle-specific parameter.
// Need to set up a parameter system first
static constexpr uint32_t SESSION_KEY = 0xABCDABCD;

#if defined(UCLIENT_PROFILE_CUSTOM_TRANSPORT)
namespace
{
constexpr uart_port_t DDS_UART = UART_NUM_1;
constexpr int DDS_UART_TX = 39;
constexpr int DDS_UART_RX = 38;
constexpr int DDS_UART_BAUD = 115200;

bool uart_open(uxrCustomTransport* transport)
{
  (void)transport;
  const uart_config_t config{
      .baud_rate = DDS_UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_DEFAULT,
      .flags = {},
  };

  if (uart_param_config(DDS_UART, &config) != ESP_OK ||
      uart_set_pin(DDS_UART, DDS_UART_TX, DDS_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK ||
      uart_driver_install(DDS_UART, 2 * UXR_CONFIG_CUSTOM_TRANSPORT_MTU, 0, 0, nullptr, 0) != ESP_OK) {
    return false;
  }
  uart_flush_input(DDS_UART);
  return true;
}

bool uart_close(uxrCustomTransport* transport)
{
  (void)transport;
  return uart_driver_delete(DDS_UART) == ESP_OK;
}

size_t uart_write(uxrCustomTransport* transport, const uint8_t* buffer, size_t length, uint8_t* error_code)
{
  (void)transport;
  const int written = uart_write_bytes(DDS_UART, buffer, length);
  *error_code = written < 0 ? 1 : 0;
  return written < 0 ? 0 : static_cast<size_t>(written);
}

size_t uart_read(uxrCustomTransport* transport, uint8_t* buffer, size_t length, int timeout, uint8_t* error_code)
{
  (void)transport;
  const int read = uart_read_bytes(DDS_UART, buffer, length, pdMS_TO_TICKS(timeout));
  *error_code = read < 0 ? 1 : 0;
  return read < 0 ? 0 : static_cast<size_t>(read);
}
}  // namespace
#endif

DDSClient::DDSClient(const char* ip, const char* port) : ip_(ip), port_(port) {}

void DDSClient::run()
{
#if defined(UCLIENT_PROFILE_UDP)
  if (!uxr_init_udp_transport(&transport_, UXR_IPv4, ip_, port_)) {
    ESP_LOGE(TAG, "UXR UDP transport failed to init!");
    return;
  }
  ESP_LOGI(TAG, "UXR UDP transport init success!");
#elif defined(UCLIENT_PROFILE_CUSTOM_TRANSPORT)
  uxr_set_custom_transport_callbacks(&transport_, true, uart_open, uart_close, uart_write, uart_read);
  if (!uxr_init_custom_transport(&transport_, nullptr)) {
    ESP_LOGE(TAG, "UXR UART transport failed to init!");
    return;
  }
  ESP_LOGI(TAG, "UXR UART transport init success on TX=%d RX=%d", DDS_UART_TX, DDS_UART_RX);
#endif

  uxr_init_session(&session_, &transport_.comm, SESSION_KEY);
  uxr_set_topic_callback(&session_, on_topic_callback, this);
  if (!uxr_create_session(&session_)) {
    ESP_LOGE(TAG, "Error creating session");
#if defined(UCLIENT_PROFILE_CUSTOM_TRANSPORT)
    uxr_close_custom_transport(&transport_);
#endif
    return;
  }
  ESP_LOGI(TAG, "UXR Session created");


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
  uint16_t topic_req =
      uxr_buffer_create_topic_xml(&session_, reliable_out_, topic_id, participant_id, topic_xml, UXR_REPLACE);


  uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
  const char* publisher_xml = "";
  uint16_t publisher_req = uxr_buffer_create_publisher_xml(
      &session_, reliable_out_, publisher_id, participant_id, publisher_xml, UXR_REPLACE
  );

  uxrObjectId subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
  const char* subscriber_xml = "";
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
  uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(
      &session_, reliable_out_, datawriter_id_, publisher_id, datawriter_xml, UXR_REPLACE
  );

  uxrObjectId datareader_id = uxr_object_id(0x01, UXR_DATAREADER_ID);
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
  uint8_t status[6];
  uint16_t requests[6] = {participant_req, topic_req, publisher_req, subscriber_req, datawriter_req, datareader_req};

  if (!uxr_run_session_until_all_status(&session_, 1000, requests, status, 6)) {
    ESP_LOGE(TAG, "Error at creating 6 entities");
    return;
  }

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

  uxr_run_session_until_confirm_delivery(&session_, 1000);
}
