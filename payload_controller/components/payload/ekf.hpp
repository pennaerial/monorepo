#pragma once

#include <chrono>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "esp_log.h"
#include "imu.hpp"

#if defined(CONFIG_IDF_TARGET_LINUX)
#include "static_mutex.hpp"
#endif

class EKF
{
public:
  EKF() = default;

  void update();
  void init();

  void base_adjustment(float deltaT);
#if defined(CONFIG_IDF_TARGET_LINUX)
  /// Supplies a simulated camera position in the EKF world frame, in meters.
  /// May be called by a native simulator callback; only the latest sample is kept.
  void set_camera_position(const Eigen::Vector3f& camera_position);
  void camera_adjustment();
#endif
  void magnetometer_adjustment();
  void encoder_adjustment();
  void stationary_adjustment();

  void compute_gain(
      Eigen::Ref<Eigen::MatrixXf> sensor_jacobian,
      Eigen::Ref<Eigen::MatrixXf> sensor_noise,
      Eigen::Ref<Eigen::MatrixXf> residual
  );

  void apply_rotation_vector(Eigen::Ref<Eigen::Vector3f> rot_vec);

private:
  using Clock = std::chrono::steady_clock;
  Clock::time_point last_update;
#if defined(CONFIG_IDF_TARGET_LINUX)
  Clock::time_point last_camera_update;
  util::StaticMutex camera_mutex_;
  Eigen::Vector3f pending_camera_position_ = Eigen::Vector3f::Zero();
  bool camera_pending_ = false;
#endif
  Clock::time_point last_encoder_update;
  bool first_update = true;
  uint32_t last_imu_update_ = 0;
  sensor_msgs_msg_Imu imu_sample_{};
  uint32_t last_left_encoder_update_ = 0;
  uint32_t last_right_encoder_update_ = 0;
  double left_encoder_velocity_ = 0.0;
  double right_encoder_velocity_ = 0.0;
  bool left_encoder_pending_ = false;
  bool right_encoder_pending_ = false;

  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

  // Position and velocity are defined at IMU position
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  // Forward is defined as the forward direction of the rover
  Eigen::Quaternionf rotation;

  Eigen::Vector3f accelerometer_bias;
  Eigen::Vector3f gyro_bias;

  // Error state: position, velocity, local attitude error, gyro bias, accel bias.
  Eigen::Matrix<float, 15, 15> covariance;
};
