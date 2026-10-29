#include "payload.hpp"

#include <cstring>
#include <initializer_list>

static const char* TAG = "Payload";

Payload::Payload()
    : left_encoder_(Side::LEFT, "left"),
      right_encoder_(Side::RIGHT, "right"),
      left_motor_(Side::LEFT, "left"),
      right_motor_(Side::RIGHT, "right")
{
}

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
  imu_ = drivers::IMU::instance();
  imu_->start();

  for (EncoderWrapper* wrapper : {&left_encoder_, &right_encoder_}) {
    ESP_LOGI(TAG, "Starting %s encoder", wrapper->debug_name);
    wrapper->encoder.start();
  }

  for (MotorWrapper* wrapper : {&left_motor_, &right_motor_}) {
    ESP_LOGI(TAG, "Starting %s motor", wrapper->debug_name);
    wrapper->motor.start();
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
  // Temporary test torques in N*m; these are not wheel-speed setpoints.
  left_motor_.motor.publish(10);
  right_motor_.motor.publish(5);
}

void Payload::publish_sensor_debug()
{
  sensor_msgs_msg_Imu latest_imu{};
  if (imu_->get_latest_if_updated(last_published_imu_update_, latest_imu)) {
    if (!DDSClient::instance().publish<TopicId::IMU_WRITER>(latest_imu)) {
      ESP_LOGW(TAG, "Failed to publish IMU DDS sample");
    }
  }

  // Read both sides every time; `||` would skip the right encoder when the left one updated.
  const bool left_updated = read_encoder(left_encoder_);
  const bool right_updated = read_encoder(right_encoder_);
  const bool encoder_updated = left_updated || right_updated;

  if (!encoder_updated) {
    return;
  }

  if (!DDSClient::instance().publish<TopicId::MOTOR_STATE_WRITER>(latest_motor_state_)) {
    ESP_LOGW(TAG, "Failed to publish motor-state DDS sample");
  }
}

bool Payload::read_encoder(EncoderWrapper& wrapper)
{
  int64_t ticks = 0;
  if (!wrapper.encoder.get_latest_if_updated(wrapper.last_published_update, ticks)) {
    return false;
  }

  if (wrapper.side == Side::LEFT) {
    latest_motor_state_.left_encoder_count = ticks;
  } else {
    latest_motor_state_.right_encoder_count = ticks;
  }
  return true;
}
