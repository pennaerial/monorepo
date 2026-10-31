#include "ekf.hpp"

#include <Eigen/Geometry>
#include <limits>

// TODO Measurement Collection  & GPS fake camera position gen for gazebo sitl
// TODO have camera update whenever data available (not every 100ms)
// ==================================================================
// TODO later: One important detail: Since the predicted axle velocity uses the gyroscope measurement, gyro noise also
// enters this measurement model. A more complete \(R_z\) includes the contribution:
// \[
// E[\boldsymbol\ell^B]_\times
// \Sigma_g
// [\boldsymbol\ell^B]_\times^TE^T
// \]
// TODO Magnetometer updates (with proper offset/junk)
// TODO later no-movement updates (with proper offset/junk)
// Possible optimizations left unimplemented: fixed-size measurement matrices to avoid allocations, caching constant
// noise matrices, reusing rotation matrices, and exploiting sparse covariance blocks.


static const char* TAG = "Kalman Filter";

namespace
{
// Provisional maximum correction rates; match these to the sensors later.
constexpr auto ENCODER_UPDATE_PERIOD = std::chrono::milliseconds(20);
constexpr auto CAMERA_UPDATE_PERIOD = std::chrono::milliseconds(100);

// Covariance Initialization
float INITIAL_POSITION_ERROR_SD = 0.1f;
float INITIAL_VELOCITY_ERROR_SD = 0.2f;
float INITIAL_ROTATION_ERROR_SD = 0.1f;
float INITIAL_GYRO_BIAS_ERROR_SD = 0.01f;
float INITIAL_ACCEL_BIAS_ERROR_SD = 0.05f;

// Continuous-time noise densities; Qc is discretized by multiplying by deltaT.
float GYRO_WHITE_NOISE_SD = 0.0f;
float ACCEL_WHITE_NOISE_SD = 0.0f;
float GYRO_BIAS_RANDOM_WALK_SD = 0.0f;
float ACCEL_BIAS_RANDOM_WALK_SD = 0.0f;

// Low rate sensor noise (Rz matrix noise -> when computing kalman gain)
float CAMERA_READING_ERROR_SD_X = 0;
float CAMERA_READING_ERROR_SD_Y = 0;
float CAMERA_READING_ERROR_SD_Z = 0;

float ENCODER_READING_ERROR_SD_X = 0;
float ENCODER_READING_ERROR_SD_Y = 0;

// Offsets / payload construction
float WHEEL_RADIUS = 0;
Eigen::Vector3f CAMERA_OFFSET_FROM_IMU(0.0, 0.0, 0.0);
Eigen::Vector3f CENTER_AXLE_OFFSET_FROM_IMU(0.0, 0.0, 0.0);
}  // namespace

void EKF::init()
{
  first_update = true;
  position = Eigen::Vector3f::Zero();
  velocity = Eigen::Vector3f::Zero();
  rotation = Eigen::Quaternionf::Identity();
  accelerometer_bias = Eigen::Vector3f::Zero();
  gyro_bias = Eigen::Vector3f::Zero();

  covariance.setZero();
  covariance.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * INITIAL_POSITION_ERROR_SD * INITIAL_POSITION_ERROR_SD;
  covariance.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * INITIAL_VELOCITY_ERROR_SD * INITIAL_VELOCITY_ERROR_SD;
  covariance.block<3, 3>(6, 6) = Eigen::Matrix3f::Identity() * INITIAL_ROTATION_ERROR_SD * INITIAL_ROTATION_ERROR_SD;
  covariance.block<3, 3>(9, 9) = Eigen::Matrix3f::Identity() * INITIAL_GYRO_BIAS_ERROR_SD * INITIAL_GYRO_BIAS_ERROR_SD;
  covariance.block<3, 3>(12, 12) =
      Eigen::Matrix3f::Identity() * INITIAL_ACCEL_BIAS_ERROR_SD * INITIAL_ACCEL_BIAS_ERROR_SD;
}


void EKF::update()
{
  // Monotonic elapsed time on both hardware and Linux SITL, independent of Gazebo.
  const auto now = Clock::now();
  if (first_update) {
    last_update = last_camera_update = last_encoder_update = now;
    first_update = false;
    return;
  }

  const float deltaT = std::chrono::duration<float>(now - last_update).count();
  if (deltaT <= 0.0f) {
    return;
  }
  last_update = now;

  // TODO Populate the measurement placeholders before calling this loop in Payload.
  // Prediction runs on each call; corrections run at most once per period.
  // THIS CODEBELOW WILL BE HIGHLY CHANGED IN UPCOMING PR
  base_adjustment(deltaT);
  if (now - last_encoder_update >= ENCODER_UPDATE_PERIOD) {
    last_encoder_update = now;
    encoder_adjustment();
  }
  if (now - last_camera_update >= CAMERA_UPDATE_PERIOD) {
    last_camera_update = now;
    camera_adjustment();
  }
  // Magnetometer and stationary corrections still need measurement models/conditions.
}

Eigen::Matrix3f skew(const Eigen::Ref<Eigen::Vector3f> v)
{
  Eigen::Matrix3f M;
  M << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return M;
}

void EKF::camera_adjustment()
{
  Eigen::Vector3f camera_pos;

  Eigen::Matrix<float, 3, 15> H = Eigen::Matrix<float, 3, 15>::Zero();
  H.block<3, 3>(0, 0) = I;

  // To take into account camera offset
  Eigen::Matrix3f R_BW = rotation.normalized().toRotationMatrix();
  H.block<3, 3>(0, 6) = -R_BW * skew(CAMERA_OFFSET_FROM_IMU);

  Eigen::Vector3f residual = camera_pos - (position + R_BW * CAMERA_OFFSET_FROM_IMU);

  Eigen::Matrix3f noise = Eigen::Matrix3f::Zero();
  noise(0, 0) = CAMERA_READING_ERROR_SD_X * CAMERA_READING_ERROR_SD_X;
  noise(1, 1) = CAMERA_READING_ERROR_SD_Y * CAMERA_READING_ERROR_SD_Y;
  noise(2, 2) = CAMERA_READING_ERROR_SD_Z * CAMERA_READING_ERROR_SD_Z;

  compute_gain(H, noise, residual);
}

void EKF::encoder_adjustment()
{
  float left_encoder_velo;
  float right_encoder_velo;
  Eigen::Vector3f raw_gyro;

  Eigen::Matrix3f R_WB = rotation.normalized().toRotationMatrix().transpose();

  Eigen::Vector3f corrected_angular_velo = raw_gyro - gyro_bias;
  float forward_body_velo = WHEEL_RADIUS * (left_encoder_velo + right_encoder_velo) / 2.0f;
  Eigen::Vector3f nominal_body_velo = R_WB * velocity;

  Eigen::Matrix<float, 2, 15> H = Eigen::Matrix<float, 2, 15>::Zero();
  Eigen::Matrix<float, 2, 3> E = Eigen::Matrix<float, 2, 3>::Identity();

  H.block<2, 3>(0, 3) = E * R_WB;
  H.block<2, 3>(0, 6) = E * skew(nominal_body_velo);
  H.block<2, 3>(0, 9) = E * skew(CENTER_AXLE_OFFSET_FROM_IMU);

  Eigen::Vector3f body_velocity = nominal_body_velo + corrected_angular_velo.cross(CENTER_AXLE_OFFSET_FROM_IMU);
  Eigen::Vector2f residual(forward_body_velo - body_velocity.x(), -body_velocity.y());

  Eigen::Matrix2f noise = Eigen::Matrix2f::Zero();
  noise(0, 0) = ENCODER_READING_ERROR_SD_X * ENCODER_READING_ERROR_SD_X;
  noise(1, 1) = ENCODER_READING_ERROR_SD_Y * ENCODER_READING_ERROR_SD_Y;

  compute_gain(H, noise, residual);
}

void EKF::magnetometer_adjustment()
{
  // TODO

  Eigen::Matrix<float, 3, 15> H = Eigen::Matrix<float, 3, 15>::Zero();
  H.block<3, 3>(0, 0) = I;
}

void EKF::stationary_adjustment()
{
  // TODO
}

void EKF::compute_gain(
    Eigen::Ref<Eigen::MatrixXf> sensor_jacobian,
    Eigen::Ref<Eigen::MatrixXf> sensor_noise,
    Eigen::Ref<Eigen::MatrixXf> residual
)
{
  const Eigen::MatrixXf PHt = covariance * sensor_jacobian.transpose();
  Eigen::MatrixXf innovation_covariance = sensor_jacobian * PHt + sensor_noise;
  innovation_covariance = (0.5f * (innovation_covariance + innovation_covariance.transpose())).eval();
  if (!innovation_covariance.allFinite() || !residual.allFinite()) {
    ESP_LOGW(TAG, "Skipping correction: non-finite innovation or residual");
    return;
  }

  // Solve S * K^T = (P * H^T)^T instead of explicitly forming S^-1.
  Eigen::LDLT<Eigen::MatrixXf> solver(innovation_covariance);
  if (solver.info() != Eigen::Success || !solver.isPositive() || !solver.vectorD().allFinite() ||
      solver.vectorD().minCoeff() <=
          std::numeric_limits<float>::epsilon() * innovation_covariance.cwiseAbs().maxCoeff()) {
    ESP_LOGW(TAG, "Skipping correction: singular or invalid innovation covariance");
    return;
  }
  Eigen::MatrixXf kalman_gain = solver.solve(PHt.transpose()).transpose();
  Eigen::Matrix<float, 15, 1> dx = kalman_gain * residual;
  if (solver.info() != Eigen::Success || !kalman_gain.allFinite() || !dx.allFinite()) {
    ESP_LOGW(TAG, "Skipping correction: innovation solve failed");
    return;
  }

  // Update pose
  position += dx.segment<3>(0);
  velocity += dx.segment<3>(3);
  Eigen::Vector3f dtheta = dx.segment<3>(6);
  apply_rotation_vector(dtheta);
  gyro_bias += dx.segment<3>(9);
  accelerometer_bias += dx.segment<3>(12);

  // Update covariance
  Eigen::Matrix<float, 15, 15> intermediate = Eigen::Matrix<float, 15, 15>::Identity() - kalman_gain * sensor_jacobian;

  covariance =
      intermediate * covariance * intermediate.transpose() + kalman_gain * sensor_noise * kalman_gain.transpose();
  Eigen::Matrix<float, 15, 15> gamma = Eigen::Matrix<float, 15, 15>::Identity();
  gamma.block<3, 3>(6, 6) = I - 0.5 * skew(dtheta);

  covariance = gamma * covariance * gamma.transpose();
  covariance = (0.5f * (covariance + covariance.transpose())).eval();
}

void EKF::apply_rotation_vector(Eigen::Ref<Eigen::Vector3f> rot_vec)
{
  float angle = rot_vec.norm();
  // Handle the edge case of a zero-rotation vector to avoid division-by-zero
  Eigen::Vector3f axis = (angle > 0.0f) ? rot_vec.normalized() : Eigen::Vector3f::UnitX();
  rotation = rotation * Eigen::Quaternionf(Eigen::AngleAxisf(angle, axis));
  rotation.normalize();
}

void EKF::base_adjustment(float deltaT)
{
  Eigen::Vector3f raw_gyro;
  Eigen::Vector3f raw_accelero;

  // Rotation body -> world
  Eigen::Matrix3f R_BW = rotation.normalized().toRotationMatrix();

  // Process sensor inputs
  Eigen::Vector3f angular_velo = raw_gyro - gyro_bias;
  Eigen::Vector3f corrected_accelero = raw_accelero - accelerometer_bias;
  // World frame is Z-up; accelerometer readings are body-frame specific force.
  Eigen::Vector3f accel = R_BW * corrected_accelero + Eigen::Vector3f(0, 0, -9.81f);


  // Update nominal state
  position = position + velocity * deltaT + 0.5f * accel * deltaT * deltaT;
  velocity = velocity + accel * deltaT;
  Eigen::Vector3f rot_vec = angular_velo * deltaT;
  apply_rotation_vector(rot_vec);


  // Update covariance //

  // Evolution of system matrix
  Eigen::Matrix<float, 15, 15> F = Eigen::Matrix<float, 15, 15>::Zero();
  F.block<3, 3>(0, 3) = I;
  F.block<3, 3>(3, 6) = -R_BW * skew(corrected_accelero);
  F.block<3, 3>(3, 12) = -R_BW;
  F.block<3, 3>(6, 6) = -skew(angular_velo);
  F.block<3, 3>(6, 9) = -I;

  // First-order discretization, suitable for small IMU time steps.
  Eigen::Matrix<float, 15, 15> phi = Eigen::Matrix<float, 15, 15>::Identity() + F * deltaT;

  // Matrix for how noise in one variable affects others
  Eigen::Matrix<float, 15, 12> G = Eigen::Matrix<float, 15, 12>::Zero();
  G.block<3, 3>(3, 3) = -R_BW;
  G.block<3, 3>(6, 0) = -I;
  G.block<3, 3>(9, 6) = I;
  G.block<3, 3>(12, 9) = I;

  // Matrix for noise
  Eigen::Matrix<float, 12, 12> Qc = Eigen::Matrix<float, 12, 12>::Zero();
  Qc.block<3, 3>(0, 0) = GYRO_WHITE_NOISE_SD * GYRO_WHITE_NOISE_SD * I;
  Qc.block<3, 3>(3, 3) = ACCEL_WHITE_NOISE_SD * ACCEL_WHITE_NOISE_SD * I;
  Qc.block<3, 3>(6, 6) = GYRO_BIAS_RANDOM_WALK_SD * GYRO_BIAS_RANDOM_WALK_SD * I;
  Qc.block<3, 3>(9, 9) = ACCEL_BIAS_RANDOM_WALK_SD * ACCEL_BIAS_RANDOM_WALK_SD * I;

  Eigen::Matrix<float, 15, 15> uncertainty = G * Qc * G.transpose() * deltaT;
  covariance = phi * covariance * phi.transpose() + uncertainty;

  // Reduce floating-point asymmetry
  covariance = (0.5f * (covariance + covariance.transpose())).eval();
}
