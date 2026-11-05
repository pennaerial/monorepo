#pragma once

#include <cstdint>
#include <optional>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "esp_log.h"

class EKF
{
public:
  struct ImuSample {
    int64_t timestamp_ns;
    Eigen::Vector3f angular_velocity;  // body frame, rad/s
    Eigen::Vector3f specific_force;    // body frame, m/s^2
  };

  struct EncoderSample {
    float left_rad_s;
    float right_rad_s;
    Eigen::Vector3f angular_velocity;  // latest measured body gyro, rad/s
  };

  // Each populated field represents a new measurement; empty fields are skipped.
  // A batch is a value snapshot, independent of the sensor threads/caches.
  struct Measurements {
    std::optional<ImuSample> imu;
    std::optional<EncoderSample> encoders;
    std::optional<Eigen::Vector3f> camera_position;  // world XYZ, metres, Z-up
  };

  static EKF& instance();
  EKF(const EKF&) = delete;
  EKF& operator=(const EKF&) = delete;

  // init/update and state reads must have a single owner (currently Payload's loop).
  void update(const Measurements& measurements);
  void init(const Eigen::Vector3f& start_position);

  const Eigen::Vector3f& get_position() const
  {
    return position;
  }

private:
  EKF() = default;

  void base_adjustment(float deltaT, const Eigen::Vector3f& raw_gyro, const Eigen::Vector3f& raw_accelero);
  void camera_adjustment(const Eigen::Vector3f& camera_pos);
  void magnetometer_adjustment();
  void encoder_adjustment(const EncoderSample& sample);
  void stationary_adjustment();

  void compute_gain(
      Eigen::Ref<Eigen::MatrixXf> sensor_jacobian,
      Eigen::Ref<Eigen::MatrixXf> sensor_noise,
      Eigen::Ref<Eigen::MatrixXf> residual
  );

  void apply_rotation_vector(Eigen::Ref<Eigen::Vector3f> rot_vec);

  int64_t last_imu_timestamp_ns_ = 0;
  bool first_update = true;

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
