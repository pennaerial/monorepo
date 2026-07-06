#include "sitl_runtime.hpp"
#include "esp_log.h"

static const char* TAG = "SITL_RUNTIME";

namespace sitl {
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
            ESP_LOGW(TAG, "No value specified for GZ_MODEL, defaulting to %s", config.gz_model);
        } else {
            config.gz_model = gz_model;
        }

        const char* gz_world = std::getenv("GZ_WORLD");
        if (gz_world == nullptr) {
            ESP_LOGW(TAG, "No value specified for GZ_WORLD, defaulting to %s", config.gz_world);
        } else {
            config.gz_world = gz_world;
        }


        loaded = true;;
        return config;
    }
};
