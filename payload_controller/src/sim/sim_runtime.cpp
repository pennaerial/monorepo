#include "sim_runtime.hpp"
#include "esp_log.h"

namespace sim {
    namespace {
        SimConfig config;
        bool loaded = false;
        std::mutex mtx;
    }


    const SimConfig& get_config() {
        std::lock_guard<std::mutex> lock(mtx);
        if (loaded) {
            return config;
        }

        const char* gz_model = std::getenv("GZ_MODEL");
        if (gz_model == nullptr) {
            ESP_LOGW("SIM_ENV", "No value specified for GZ_MODEL, defaulting to %s", config.gz_model.c_str());
        } else {
            config.gz_model = gz_model;
        }

        loaded = true;;
        return config;
    }
};
