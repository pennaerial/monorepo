#pragma once

#include "esp_log.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

class EKF {
    public:
        EKF() = default;
    
        void update();
        void init();

        void base_adjustment(float deltaT);
        void camera_adjustment();
        void magnetometer_adjustment();
        void encoder_adjustment();
        void stationary_adjustment();

        void compute_gain(Eigen::Ref<Eigen::MatrixXf> sensor_jacobian, Eigen::Ref<Eigen::MatrixXf> sensor_noise, Eigen::Ref<Eigen::MatrixXf> residual);

        void apply_rotation_vector(Eigen::Ref<Eigen::Vector3f> rot_vec);
    
    private:
        const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

        // Position and velocity are defined at IMU position
        Eigen::Vector3f position;
        Eigen::Vector3f velocity;
        // Forward is defined as the forward direction of the rover
        Eigen::Quaternionf rotation;

        Eigen::Vector3f accelerometer_bias;
        Eigen::Vector3f gyro_bias;

        Eigen::Matrix<float, 15, 15> covariance;


};