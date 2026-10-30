#pragma once

#include "esp_log.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

class EKF {
    public:
        EKF() = default;
    
        void update();
        void init();
        void estimate();
        void predict_next_state(float deltaT);
        void compute_gain();

        void to_world_axes(Eigen::Ref<Eigen::Vector3f> body_coords);
        void to_body_axes(Eigen::Ref<Eigen::Vector3f> world_coords);
    
    private:
        const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

        Eigen::Vector3f position;
        Eigen::Vector3f velocity;
        Eigen::Quaternionf rotation;

        Eigen::Vector3f accelerometer_bias;
        Eigen::Vector3f gyro_bias;

        Eigen::Matrix<float, 15, 15> covariance;


};