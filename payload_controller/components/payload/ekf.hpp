#pragma once

#include <cstdint>
#include <optional>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "esp_log.h"
#include "log_rate_limiter.hpp"

/// Error-state filter estimating IMU pose, world velocity, and IMU biases; all state access belongs to one thread.
class EKF
{
public:
  /// One timestamped body-frame IMU measurement used for prediction.
  struct ImuSample {
    /// Sensor timestamp in nanoseconds; elapsed sensor time drives prediction on both targets.
    int64_t timestamp_ns;
    /// Measured body angular velocity in rad/s, before estimated gyro bias is removed.
    Eigen::Vector3f angular_velocity;
    /// Measured body specific force (acceleration minus gravity) in m/s^2, including accelerometer bias.
    Eigen::Vector3f specific_force;
  };

  /// New wheel measurements paired with the latest valid gyro for the axle lever-arm correction.
  struct EncoderSample {
    /// Left wheel/output-shaft angular speed in rad/s.
    float left_rad_s;
    /// Right wheel/output-shaft angular speed in rad/s.
    float right_rad_s;
    /// Latest measured body angular velocity in rad/s, before estimated gyro bias is removed.
    Eigen::Vector3f angular_velocity;
  };

  /// Value snapshot of fresh measurements; empty fields are skipped independently without waiting for sensors.
  struct Measurements {
    /// New IMU sample, if available, for timestamp-based prediction.
    std::optional<ImuSample> imu;
    /// New measurements from both wheels, if available, for forward/lateral velocity correction.
    std::optional<EncoderSample> encoders;
    /// New camera-origin position in world XYZ metres (Z-up); the filter accounts for its offset from the IMU.
    std::optional<Eigen::Vector3f> camera_position;
  };

  /// Returns the single filter instance; construction does not initialize its estimate.
  static EKF& instance();
  /// Prevents copying the singleton and its estimator state.
  EKF(const EKF&) = delete;
  /// Prevents assigning another estimator's state to the singleton.
  EKF& operator=(const EKF&) = delete;

  /// Predicts and corrects from the supplied fresh measurements on the estimator's owning thread.
  void update(const Measurements& measurements);
  /// Resets to the supplied world IMU position, zero velocity/biases, identity attitude, and initial covariance.
  void init(const Eigen::Vector3f& start_position);

  /// Returns the estimated world IMU position in metres; read only from the estimator's owning thread.
  const Eigen::Vector3f& get_position() const
  {
    return position;
  }

private:
  /// Restricts construction to instance(); init() must be called before using the filter.
  EKF() = default;

  /// Propagates state/covariance over deltaT seconds using raw body gyro (rad/s) and specific force (m/s^2).
  void base_adjustment(float deltaT, const Eigen::Vector3f& raw_gyro, const Eigen::Vector3f& raw_accelero);
  /// Corrects position/attitude from a fresh world camera position in metres, accounting for its lever arm.
  void camera_adjustment(const Eigen::Vector3f& camera_pos);
  /// Placeholder for a future magnetic-field correction; currently unused and does not update the estimate.
  void magnetometer_adjustment();
  /// Corrects body forward/lateral velocity using wheel speeds and the gyro-based axle lever arm.
  void encoder_adjustment(const EncoderSample& sample);
  /// Placeholder for a future stationary constraint; currently unused and does not update the estimate.
  void stationary_adjustment();

  /// Computes the Kalman gain from the measurement Jacobian/noise/residual and applies state/covariance corrections.
  void compute_gain(
      // Measurement derivative with respect to the 15-component error state.
      Eigen::Ref<Eigen::MatrixXf> sensor_jacobian,
      // Covariance of the measurement noise, in squared measurement units.
      Eigen::Ref<Eigen::MatrixXf> sensor_noise,
      // Observed measurement minus the measurement predicted by the nominal state.
      Eigen::Ref<Eigen::MatrixXf> residual
  );

  /// Right-multiplies the body-to-world quaternion by the supplied local rotation vector in radians and normalizes it.
  void apply_rotation_vector(Eigen::Ref<Eigen::Vector3f> rot_vec);

  /// Last accepted IMU timestamp in nanoseconds; reseeded when sensor time moves backwards.
  int64_t last_imu_timestamp_ns_ = 0;
  /// Makes the first valid IMU sample seed time without integrating an unknown preceding interval.
  bool first_update = true;

  /// Limits recurring state logs to one line per 20 ms on the estimator's owning thread.
  util::LogRateLimiter debug_rate_;
  /// Limits recurring correction-failure warnings to one line per 20 ms.
  util::LogRateLimiter warning_rate_;

  /// Shared 3x3 identity matrix used in Jacobians and covariance propagation.
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

  /// Estimated IMU position in world XYZ metres (Z-up).
  Eigen::Vector3f position;
  /// Estimated velocity of the IMU origin in world XYZ m/s.
  Eigen::Vector3f velocity;
  /// Unit quaternion mapping body vectors (X forward) into the Z-up world frame.
  Eigen::Quaternionf rotation;

  /// Estimated additive body-frame accelerometer bias in m/s^2.
  Eigen::Vector3f accelerometer_bias;
  /// Estimated additive body-frame gyro bias in rad/s.
  Eigen::Vector3f gyro_bias;

  /// Error covariance ordered as world position, world velocity, local attitude error, gyro bias, accel bias (3 each).
  Eigen::Matrix<float, 15, 15> covariance;
};
