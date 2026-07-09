#pragma once


#include <string>
#include <cstdlib>
#include <mutex>
#include "esp_log.h"

namespace sitl {

    struct SimConfig {
        const char* gz_model = "payload_0";
        const char* gz_world = "default";
    };

    const SimConfig& get_config();
}
