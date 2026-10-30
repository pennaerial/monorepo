#include "payload.hpp"

#include <chrono>
#include <cstring>
#include <initializer_list>

static const char* TAG = "Payload";

namespace
{
// Output-shaft/wheel speed, not the motor rotor speed before the gearbox.
constexpr float LEFT_SETPOINT_RAD_S = 7.0f;
constexpr float RIGHT_SETPOINT_RAD_S = 7.0f;
constexpr float SPEED_KP = 0.08f;  // PWM duty per rad/s of error.
constexpr float SPEED_KI = 0.15f;  // PWM duty per radian of accumulated error.
constexpr float MAX_INTEGRAL_DUTY = 0.3f;
constexpr float FRICTION_DUTY = 0.05f;
// Feedforward calibration: measured speed at full duty under the test load.
// This scales the duty estimate; it does not configure the simulated motor.
constexpr float FULL_DUTY_SPEED_RAD_S = 7.434f;
// Friction plus velocity feedforward reaches duty 1 at the calibrated speed:
// duty = sign(speed) * FRICTION_DUTY + VELOCITY_DUTY * speed / FULL_DUTY_SPEED_RAD_S.
constexpr float VELOCITY_DUTY = 1.0f - FRICTION_DUTY;
}  // namespace

Payload::Payload()
    : left_motor_controller(
          SPEED_KP,
          SPEED_KI,
          LEFT_SETPOINT_RAD_S,
          MAX_INTEGRAL_DUTY,
          FRICTION_DUTY,
          VELOCITY_DUTY,
          FULL_DUTY_SPEED_RAD_S
      ),
      right_motor_controller(
          SPEED_KP,
          SPEED_KI,
          RIGHT_SETPOINT_RAD_S,
          MAX_INTEGRAL_DUTY,
          FRICTION_DUTY,
          VELOCITY_DUTY,
          FULL_DUTY_SPEED_RAD_S
      ),
      left_encoder_(Side::LEFT, "left"),
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
  ESP_LOGI(
      TAG, "Speed config: left=%.3f right=%.3f full_duty=%.3f rad/s kp=%.3f ki=%.3f",
      LEFT_SETPOINT_RAD_S, RIGHT_SETPOINT_RAD_S, FULL_DUTY_SPEED_RAD_S, SPEED_KP, SPEED_KI
  );
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
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
  update_motor(left_motor_, left_encoder_, left_motor_controller, LEFT_SETPOINT_RAD_S, now_us);
  update_motor(right_motor_, right_encoder_, right_motor_controller, RIGHT_SETPOINT_RAD_S, now_us);
}

void Payload::update_motor(
    MotorWrapper& motor,
    EncoderWrapper& encoder,
    PWMController& controller,
    float setpoint_rad_s,
    int64_t now_us
)
{
  double angular_velocity_rad_s;
  if (encoder.encoder.get_latest_angular_velocity_if_updated(encoder.last_control_update, angular_velocity_rad_s)) {
    if (motor.last_control_timestamp_us != 0 && now_us > motor.last_control_timestamp_us) {
      const float delta_s = (now_us - motor.last_control_timestamp_us) * 1e-6;
      motor.pwm = controller.command(delta_s, angular_velocity_rad_s);

      // Every 100 ms; the controller still runs on each new sample.
      if (now_us - motor.last_debug_timestamp_us >= 100000) {
        ESP_LOGI(
            TAG, "%s motor: setpoint=%.3f rad/s measured=%.3f rad/s error=%.3f rad/s pwm=%.3f", motor.debug_name,
            setpoint_rad_s, angular_velocity_rad_s, setpoint_rad_s - angular_velocity_rad_s, motor.pwm
        );
        motor.last_debug_timestamp_us = now_us;
      }
    } else {
      // Seed the controller clock before its first update.
      motor.pwm = 0.0f;
      controller.change_set_point(0.0f);
      controller.change_set_point(setpoint_rad_s);
      motor.last_debug_timestamp_us = now_us;
    }
    motor.last_control_timestamp_us = now_us;
  }

  // Keep publishing the last duty between encoder samples; do not integrate stale feedback.
  motor.motor.publish(motor.pwm);
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
