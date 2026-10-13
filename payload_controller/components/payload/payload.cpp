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
  if (callback_count <= 3 || (callback_count % 10) == 0) {
    ESP_LOGI(
        TAG, "DDS IMU callback #%u orientation: [%f, %f, %f, %f]", callback_count, imu_msg->orientation.x,
        imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w
    );
  }
}

void encoderCallback(const Topic& topic, const void* msg, uint16_t length, void* args)
{
  (void)length;
  (void)args;

  if (std::strcmp(topic.type_name, "payload_interfaces::msg::dds_::MotorState_") != 0) {
    ESP_LOGW(TAG, "Unexpected DDS callback type: %s", topic.type_name);
    return;
  }

  static uint32_t callback_count = 0;
  ++callback_count;

  const payload_interfaces_msg_MotorState* motor_state = static_cast<const payload_interfaces_msg_MotorState*>(msg);
  if ((callback_count % 10) == 0) {
    ESP_LOGI(
        TAG, "DDS encoder callback #%u ticks: left=%lld right=%lld", callback_count,
        static_cast<long long>(motor_state->left_encoder_count),
        static_cast<long long>(motor_state->right_encoder_count)
    );
  }
}

void Payload::init()
{
  imu = drivers::IMU::instance();
  imu->start();

  for (std::size_t i = 0; i < PAYLOAD_CORNERS.size(); ++i) {
    encoders[i] = drivers::Encoder::create(PAYLOAD_CORNERS[i]);
    encoders[i]->start();

    motors[i] = drivers::Motor::create(PAYLOAD_CORNERS[i]);
    motors[i]->start();
  }

  if (!DDSClient::instance().set_reader_callback(TopicId::IMU_READER, &testCallback, nullptr)) {
    ESP_LOGW(TAG, "Failed to bind IMU DDS reader callback");
  }
  if (!DDSClient::instance().set_reader_callback(TopicId::MOTOR_STATE_READER, &encoderCallback, nullptr)) {
    ESP_LOGW(TAG, "Failed to bind motor-state DDS reader callback");
  }
}

void Payload::update()
{
  publish_sensor_debug();

  motor_updates();
}

void Payload::motor_updates()
{
  for (auto& motor : motors) {
    if (motor == nullptr) {
      continue;
    }

    // temp for testing
    motor->publish(payload_corner_is_left(motor->corner()) ? 10 : 5);
  }
}

void Payload::publish_sensor_debug()
{
  sensor_msgs_msg_Imu latest_imu{};
  if (imu->get_latest_if_updated(last_published_imu_update_, latest_imu)) {
    if (!DDSClient::instance().publish<TopicId::IMU_WRITER>(latest_imu)) {
      ESP_LOGW(TAG, "Failed to publish IMU DDS sample");
    }
  }

  bool encoder_updated = false;
  for (std::size_t i = 0; i < encoders.size(); ++i) {
    if (encoders[i] == nullptr) {
      continue;
    }

    payload_interfaces_msg_MotorState encoder_msg{};
    if (!encoders[i]->get_latest_if_updated(last_published_encoder_updates[i], encoder_msg)) {
      continue;
    }

    if (payload_corner_is_left(encoders[i]->corner())) {
      latest_motor_state.left_encoder_count = encoder_msg.left_encoder_count;
    } else {
      latest_motor_state.right_encoder_count = encoder_msg.right_encoder_count;
    }
    encoder_updated = true;
  }

  if (!encoder_updated) {
    return;
  }

  if (!DDSClient::instance().publish<TopicId::MOTOR_STATE_WRITER>(latest_motor_state)) {
    ESP_LOGW(TAG, "Failed to publish motor-state DDS sample");
  }
}
