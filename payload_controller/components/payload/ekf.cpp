#include "ekf.hpp"



static const char* TAG = "Kalman Filter";

void EKF::init() {
    covariance.setZero();
    // TODO Each is equivalanet to standard deviation squared -> create constants for that

    // Position uncertainty (m²)
    covariance.block<3, 3>(0, 0) =
        Eigen::Matrix3f::Identity() * 0.01f;

    // Velocity uncertainty ((m/s)²)
    covariance.block<3, 3>(3, 3) =
        Eigen::Matrix3f::Identity() * 0.04f;

    // Orientation uncertainty (rad²)
    covariance.block<3, 3>(6, 6) =
        Eigen::Matrix3f::Identity() * 0.01f;

    // Gyroscope bias uncertainty ((rad/s)²)
    covariance.block<3, 3>(9, 9) =
        Eigen::Matrix3f::Identity() * 0.0001f;

    // Accelerometer bias uncertainty ((m/s²)²)
    covariance.block<3, 3>(12, 12) =
        Eigen::Matrix3f::Identity() * 0.0025f;

}

// TODO possibly remove these
void EKF::to_world_axes(Eigen::Ref<Eigen::Vector3f> body_coords) {
    Eigen::Matrix3f R = rotation.normalized().toRotationMatrix();
    body_coords = R * body_coords;
}

void EKF::to_body_axes(Eigen::Ref<Eigen::Vector3f> world_coords) {
    Eigen::Matrix3f R = rotation.normalized().toRotationMatrix().transpose();
    world_coords = R * world_coords;
}

void EKF::update() {
    // Here we need to determine the frequency with which we collect sensor measurements
    // Calculate delta time
    // Determine when to execute each part of the filter
}

Eigen::Matrix3f skew(const Eigen::Ref<Eigen::Vector3f> v) {
    Eigen::Matrix3f M;
    M <<     0, -v.z(),  v.y(),
           v.z(),     0, -v.x(),
          -v.y(),  v.x(),     0;
    return M;
}

void EKF::camera_adjustment() {
    Eigen::Vector3f camera_pos;

    Eigen::Matrix<float, 3, 15> H = Eigen::Matrix<float, 3, 15>::Zero();
    H.block<3, 3>(0, 0) = I;

    Eigen::Matrix<float, 3, 15> residual = Eigen::Matrix<float, 3, 15>::Zero();

    // TODO check all matrix dimensions
    
}

void EKF::encoder_adjustment() {


    Eigen::Matrix<float, 3, 15> H = Eigen::Matrix<float, 3, 15>::Zero();
    H.block<3, 3>(0, 0) = I;

}

void EKF::magnetometer_adjustment() {


    Eigen::Matrix<float, 3, 15> H = Eigen::Matrix<float, 3, 15>::Zero();
    H.block<3, 3>(0, 0) = I;

}

void EKF::compute_gain(Eigen::Ref<Eigen::MatrixXf> sensor_jacobian, Eigen::Ref<Eigen::MatrixXf> sensor_noise, Eigen::Ref<Eigen::MatrixXf> residual) {
    Eigen::MatrixXf innovation_covariance = sensor_jacobian * covariance * sensor_jacobian.transpose() + sensor_noise;


    // TODO CHECK Solve the linear system instead of explicitly inverting S.
    // Eigen::LDLT<Eigen::MatrixXd> solver(S);

    // if (solver.info() != Eigen::Success) {
    //     return; // Handle invalid measurement update
    // }

    // Eigen::MatrixXd K =
    //     solver.solve(H * P).transpose();
    Eigen::MatrixXf kalman_gain = covariance * sensor_jacobian.transpose() * innovation_covariance.inverse();

    // Update pose
    Eigen::MatrixXf dx = kalman_gain * residual;
    position += dx.segment<3>(0);
    velocity += dx.segment<3>(3);
    Eigen::Vector3f dtheta = dx.segment<3>(6);
    apply_rotation_vector(dtheta);
    gyro_bias  += dx.segment<3>(9);
    accelerometer_bias += dx.segment<3>(12);

    // Update covariance
    Eigen::Matrix<float, 15, 15> intermediate = Eigen::Matrix<float, 15, 15>::Identity() - kalman_gain * sensor_jacobian;
    
    covariance = intermediate * covariance * intermediate.transpose() + kalman_gain * sensor_noise * kalman_gain.transpose();
    Eigen::Matrix<float, 15, 15> gamma = Eigen::Matrix<float, 15, 15>::Identity();
    gamma.block<3,3>(6, 6) = I - 0.5 * skew(dtheta);

    covariance = gamma * covariance * gamma.transpose();
}

void EKF::apply_rotation_vector(Eigen::Ref<Eigen::Vector3f> rot_vec) {
    float angle = rot_vec.norm();
    // Handle the edge case of a zero-rotation vector to avoid division-by-zero
    Eigen::Vector3f axis = (angle > 1e-7f) ? rot_vec.normalized() : Eigen::Vector3d::UnitX();
    rotation = rotation * Eigen::Quaternionf(Eigen::AngleAxisf(angle, axis));
    rotation.normalize();
}

void EKF::predict_next_state(float deltaT) {
    // TODO Collect measurements
    Eigen::Vector3f raw_gyro;
    Eigen::Vector3f raw_accelero;

    // Rotation body -> world
    Eigen::Matrix3d R_BW = rotation.normalized().toRotationMatrix();
    // Rotation world -> body
    Eigen::Matrix3d R_WB = rotation.normalized().toRotationMatrix().transpose();

    // Process sensor inputs
    Eigen::Vector3f angular_velo = raw_gyro - gyro_bias;
    Eigen::Vector3f corrected_accelero = raw_accelero - accelerometer_bias;
    Eigen::Vector3f accel = R_BW * corrected_accelero + Eigen::Vector3d(0, 0, -9.81);


    // Update nominal state
    position = position + velocity * deltaT + 0.5f * accel * deltaT * deltaT;
    velocity = velocity + accel * deltaT;
    Eigen::Vector3f rot_vec = angular_velo * deltaT;
    apply_rotation_vector(rot_vec);


    // Update covariance //

    // Evolution of system matrix
    Eigen::Matrix<float, 15, 15> F = Eigen::Matrix<float, 15, 15>::Zero();
    F.block<3,3>(0,3)   = I;
    F.block<3,3>(3,6)   = -R_BW * skew(corrected_accelero);
    F.block<3,3>(3,12)  = -R_BW;
    F.block<3,3>(6,6)   = -skew(angular_velo);
    F.block<3,3>(6,9)   = -I;

    Eigen::Matrix<float, 15, 15> phi = Eigen::Matrix<float, 15, 15>::Identity() + F * deltaT;

    // Matrix for how noise in one variable affects others
    Eigen::Matrix<float, 15, 12> G = Eigen::Matrix<float, 15, 12>::Zero();
    G.block<3,3>(3,3)   = -R_BW;
    G.block<3,3>(6,0)   = -I;
    G.block<3,3>(9,6)   = I;
    G.block<3,3>(12,9)  = I;

    // Matrix for noise
    Eigen::Matrix<float, 12, 12> Qc = Eigen::Matrix<float, 12, 12>::Zero();
    Qc.block<3,3>(0,0) = sigma_g * sigma_g * I;
    Qc.block<3,3>(3,3) = sigma_a * sigma_a * I;
    Qc.block<3,3>(6,6) = sigma_bg * sigma_bg * I;
    Qc.block<3,3>(9,9) = sigma_ba * sigma_ba * I;

    Eigen::Matrix<float, 15, 15> uncertainty = G * Qc * G.transpose() * deltaT;
    covariance = phi * covariance * phi.transpose() + uncertainty;

    // Reduce floating-point asymmetry
    covariance = 0.5 * (covariance + covariance.transpose());
}

// TODO camera, encoder, imu offsets