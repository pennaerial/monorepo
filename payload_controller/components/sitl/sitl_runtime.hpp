#pragma once


#include <cstdlib>
#include <mutex>
#include <string>

#include "esp_log.h"

namespace sitl
{

struct SimConfig {
  const char* gz_model = "payload_0";
  const char* gz_world = "default";
};

const SimConfig& get_config();
/// Initialize Gazebo workers with the FreeRTOS tick signal blocked. Call before constructing drivers.
void initialize_transport();
}  // namespace sitl
