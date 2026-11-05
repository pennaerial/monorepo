#include "payload.hpp"

#include <chrono>
#include <cmath>
#include <initializer_list>

#include "ekf.hpp"

static const char* TAG = "Payload";

namespace
{
// Output-shaft/wheel speed, not the motor rotor speed before the gearbox.
constexpr float LEFT_SETPOINT_RAD_S = 7.0f;
constexpr float RIGHT_SETPOINT_RAD_S = 3.0f;
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

void Payload::init(const Eigen::Vector3f& start_position)
{
  EKF::instance().init(start_position);
  drivers::Camera::instance().start();

  ESP_LOGI(
      TAG, "Speed config: left=%.3f right=%.3f full_duty=%.3f rad/s kp=%.3f ki=%.3f", LEFT_SETPOINT_RAD_S,
      RIGHT_SETPOINT_RAD_S, FULL_DUTY_SPEED_RAD_S, SPEED_KP, SPEED_KI
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
}

void Payload::update()
{
  update_estimator();
  publish_sensor_debug();

  motor_updates();
}

void Payload::update_estimator()
{
  EKF::Measurements measurements;
  sensor_msgs_msg_Imu imu{};
  if (imu_->get_latest_if_updated(last_ekf_imu_update_, imu)) {
    EKF::ImuSample sample{
        static_cast<int64_t>(imu.header.stamp.sec) * 1000000000LL + imu.header.stamp.nanosec,
        Eigen::Vector3f(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z),
        Eigen::Vector3f(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z)
    };
    if (sample.angular_velocity.allFinite() && sample.specific_force.allFinite()) {
      measurements.imu = sample;
      ekf_gyro_ = sample.angular_velocity;
      ekf_has_gyro_ = true;
    }
  }

  // Independent cursors: motor feedback and debug reads cannot consume EKF data.
  double speed;
  if (left_encoder_.encoder.get_latest_angular_velocity_if_updated(last_ekf_left_update_, speed) &&
      std::isfinite(speed)) {
    ekf_left_rad_s_ = speed;
    ekf_left_pending_ = true;
  }
  if (right_encoder_.encoder.get_latest_angular_velocity_if_updated(last_ekf_right_update_, speed) &&
      std::isfinite(speed)) {
    ekf_right_rad_s_ = speed;
    ekf_right_pending_ = true;
  }
  // Wait for both sides, even when their callbacks arrive on different iterations.
  if (ekf_left_pending_ && ekf_right_pending_ && ekf_has_gyro_) {
    measurements.encoders =
        EKF::EncoderSample{static_cast<float>(ekf_left_rad_s_), static_cast<float>(ekf_right_rad_s_), ekf_gyro_};
    ekf_left_pending_ = ekf_right_pending_ = false;
  }

  geometry_msgs_msg_Vector3 camera_position{};
  if (drivers::Camera::instance().get_latest_if_updated(last_ekf_camera_update_, camera_position)) {
    measurements.camera_position = Eigen::Vector3f(camera_position.x, camera_position.y, camera_position.z);
    if (camera_debug_rate_.ready()) {
      ESP_LOGI(
          TAG, "Camera world XYZ [m]: [%.3f, %.3f, %.3f]", camera_position.x, camera_position.y, camera_position.z
      );
    }
  }
  EKF::instance().update(measurements);
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

      // Debug output is capped independently of controller sample processing.
      if (motor.debug_rate.ready()) {
        ESP_LOGD(
            TAG, "%s motor: setpoint=%.3f rad/s measured=%.3f rad/s error=%.3f rad/s pwm=%.3f", motor.debug_name,
            setpoint_rad_s, angular_velocity_rad_s, setpoint_rad_s - angular_velocity_rad_s, motor.pwm
        );
      }
    } else {
      // Seed the controller clock before its first update.
      motor.pwm = 0.0f;
      controller.change_set_point(0.0f);
      controller.change_set_point(setpoint_rad_s);
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
    if (imu_debug_rate_.ready()) {
      ESP_LOGI(
          TAG,
          "IMU accel [m/s^2]: [%.3f, %.3f, %.3f] gyro [rad/s]: [%.3f, %.3f, %.3f] q[wxyz]: [%.3f, %.3f, %.3f, %.3f]",
          latest_imu.linear_acceleration.x, latest_imu.linear_acceleration.y, latest_imu.linear_acceleration.z,
          latest_imu.angular_velocity.x, latest_imu.angular_velocity.y, latest_imu.angular_velocity.z,
          latest_imu.orientation.w, latest_imu.orientation.x, latest_imu.orientation.y, latest_imu.orientation.z
      );
    }
    if (!DDSClient::instance().publish<TopicId::IMU_WRITER>(latest_imu) && publish_warning_rate_.ready()) {
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

  if (encoder_debug_rate_.ready()) {
    ESP_LOGI(
        TAG, "Encoder ticks: left=%lld right=%lld speed [rad/s]: left=%.3f right=%.3f new[left,right]=[%d,%d]",
        static_cast<long long>(latest_motor_state_.left_encoder_count),
        static_cast<long long>(latest_motor_state_.right_encoder_count),
        left_encoder_.encoder.get_latest_angular_velocity(), right_encoder_.encoder.get_latest_angular_velocity(),
        left_updated, right_updated
    );
  }
  if (!DDSClient::instance().publish<TopicId::MOTOR_STATE_WRITER>(latest_motor_state_) &&
      publish_warning_rate_.ready()) {
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
