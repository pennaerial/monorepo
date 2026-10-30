#pragma once

#include "esp_log.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

class EKF {
    public:
        EKF() = default;
    
        void update();
        void init();

        void predict_next_state(float deltaT);

        void camera_adjustment();
        void magnetometer_adjustment();
        void encoder_adjustment();

        void compute_gain(Eigen::Ref<Eigen::MatrixXf> sensor_jacobian, Eigen::Ref<Eigen::MatrixXf> sensor_noise, Eigen::Ref<Eigen::MatrixXf> residual);

        void to_world_axes(Eigen::Ref<Eigen::Vector3f> body_coords);
        void to_body_axes(Eigen::Ref<Eigen::Vector3f> world_coords);

        void apply_rotation_vector(Eigen::Ref<Eigen::Vector3f> rot_vec);
    
    private:
        const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

        Eigen::Vector3f position;
        Eigen::Vector3f velocity;
        Eigen::Quaternionf rotation;

        Eigen::Vector3f accelerometer_bias;
        Eigen::Vector3f gyro_bias;

        Eigen::Matrix<float, 15, 15> covariance;


};