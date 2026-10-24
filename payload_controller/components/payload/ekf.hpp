#pragma once

#include "esp_log.h"

class EKF {
    public:
        EKF() = default;
    
        void update();
        void init();
        void estimate();
        void predict();
        void compute_gain();

};