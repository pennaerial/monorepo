#pragma once


#include <string>
#include <cstdlib>
#include <mutex>
#include "esp_log.h"

namespace sim {

    struct SimConfig {
        std::string gz_model = "payload_0";
    };

    const SimConfig& get_config();
}
