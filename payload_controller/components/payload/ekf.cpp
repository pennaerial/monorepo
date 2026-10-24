#include "ekf.hpp"

#include "Eigen/Dense"

static const char* TAG = "Kalman Filter";

void EKF::init() {
    Eigen::Matrix2d matrix;
    matrix << 1, 2,
              3, 4;

    // Define a 2D vector
    Eigen::Vector2d vector(5, 6);

    // Perform linear algebra operations natively on the ESP32
    Eigen::Vector2d result = matrix * vector;

}
