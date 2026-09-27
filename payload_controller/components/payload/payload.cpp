#include "payload.h"

#include <cstring>

static const char* TAG = "Payload";

#if !defined(CONFIG_IDF_TARGET_LINUX)
namespace
{
constexpr char DDS_CLIENT_TASK_NAME[] = "dds_client";
constexpr uint32_t DDS_CLIENT_TASK_STACK_SIZE_BYTES = 12288;
constexpr TickType_t DDS_CLIENT_UPDATE_DELAY = pdMS_TO_TICKS(1);
}  // namespace
#endif

Payload::Payload() : dds_client("127.0.0.1", "7777") {}

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

  dds_client.init();

  if (!dds_client.set_reader_callback("rt/imu", &testCallback, nullptr)) {
    ESP_LOGW(TAG, "Failed to bind IMU DDS reader callback");
  }

#if !defined(CONFIG_IDF_TARGET_LINUX)
  start_dds_task();
#endif
}

void Payload::update()
{
  publish_sensor_debug();

#if defined(CONFIG_IDF_TARGET_LINUX)
  dds_client.update();
#endif
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
  if (!dds_client.publish(TopicId::IMU_WRITER, &latest)) {
    ESP_LOGW(TAG, "Failed to queue IMU DDS publish");
  }
}

void Payload::start_dds_task()
{
#if !defined(CONFIG_IDF_TARGET_LINUX)
  const UBaseType_t current_priority = uxTaskPriorityGet(nullptr);
  const UBaseType_t dds_priority = current_priority > 0 ? current_priority - 1 : 0;
  if (xTaskCreate(
          [](void* args) {
            Payload* payload = static_cast<Payload*>(args);
            while (true) {
              payload->dds_client.update();
              vTaskDelay(DDS_CLIENT_UPDATE_DELAY);
            }
          },
          DDS_CLIENT_TASK_NAME, DDS_CLIENT_TASK_STACK_SIZE_BYTES, this, dds_priority, nullptr
      ) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create DDS client task");
  }
#endif
}
