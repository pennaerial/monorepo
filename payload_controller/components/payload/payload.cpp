#include "payload.h"

#include <cstring>

static const char* TAG = "Payload";

Payload::Payload() = default;

Payload* Payload::instance()
{
  static Payload instance;  // only instantiated once because static
  return &instance;
}

void testCallback(const Topic& topic, const void* msg, uint16_t length, void* args)
{
  (void)length;
  (void)args;

  if (std::strcmp(topic.type_name, "sensor_msgs::msg::dds_::Imu_") != 0) {
    ESP_LOGW(TAG, "Unexpected DDS callback type: %s", topic.type_name);
    return;
  }

  static uint32_t callback_count = 0;
  ++callback_count;

  const sensor_msgs_msg_Imu* imu_msg = static_cast<const sensor_msgs_msg_Imu*>(msg);
  if ((callback_count % 10) == 0) {
    ESP_LOGI(
        TAG, "DDS IMU callback #%u orientation: [%f, %f, %f, %f]", callback_count, imu_msg->orientation.x,
        imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w
    );
  }
}

void Payload::init()
{
  imu = drivers::IMU::instance();
  imu->start();

  encoders = drivers::Encoder::instance();
  encoders->start();

  if (!DDSClient::instance().set_reader_callback(TopicId::IMU_READER, &testCallback, nullptr)) {
    ESP_LOGW(TAG, "Failed to bind IMU DDS reader callback");
  }
}

void Payload::update()
{
  publish_sensor_debug();

  motor_updates();
}

void Payload::motor_updates()
{
  encoders->publish_motor_left(10);
  encoders->publish_motor_right(5);
}

void Payload::publish_sensor_debug()
{
  const sensor_msgs_msg_Imu latest = imu->get_latest();
  if (!DDSClient::instance().publish(TopicId::IMU_WRITER, &latest)) {
    ESP_LOGW(TAG, "Failed to queue IMU DDS publish");
  }
}
