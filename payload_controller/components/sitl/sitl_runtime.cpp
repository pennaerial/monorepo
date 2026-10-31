#include "sitl_runtime.hpp"

#include <pthread.h>
#include <signal.h>

#include <gz/transport/NodeShared.hh>

#include "esp_log.h"

static const char* TAG = "SITL_RUNTIME";

namespace sitl
{
namespace
{
SimConfig config;
bool loaded = false;
std::mutex mtx;
}  // namespace

void initialize_transport()
{
  // FreeRTOS/Linux uses process-wide SIGALRM for task scheduling. Gazebo's
  // native threads must inherit a mask blocking it, or its handler can suspend
  // a transport worker while it owns a mutex needed by the FreeRTOS task.
  sigset_t tick_signal;
  sigemptyset(&tick_signal);
  sigaddset(&tick_signal, SIGALRM);
  sigset_t previous_mask;
  int error = pthread_sigmask(SIG_BLOCK, &tick_signal, &previous_mask);
  if (error != 0) {
    ESP_LOGE(TAG, "Failed to block tick signal during Gazebo initialization: %d", error);
    std::abort();
  }

  // The shared transport constructor starts discovery, reception and publishing
  // threads. All later driver nodes reuse these workers.
  gz::transport::NodeShared::Instance();

  error = pthread_sigmask(SIG_SETMASK, &previous_mask, nullptr);
  if (error != 0) {
    ESP_LOGE(TAG, "Failed to restore task signal mask after Gazebo initialization: %d", error);
    std::abort();
  }
}


const SimConfig& get_config()
{
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


  loaded = true;
  ;
  return config;
}
};  // namespace sitl
