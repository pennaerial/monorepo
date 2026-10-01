#pragma once

#include "sdkconfig.h"

#if defined(CONFIG_IDF_TARGET_LINUX)
#include <mutex>
#else
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

namespace util
{

/// Small lock wrapper used by drivers that may be called from either app code
/// or simulator callbacks. Linux/SITL callbacks run on native threads, so use
/// std::mutex there; embedded targets use a FreeRTOS static mutex.
class StaticMutex
{
public:
#if defined(CONFIG_IDF_TARGET_LINUX)
  StaticMutex() = default;
  ~StaticMutex() = default;

  void lock()
  {
    mutex_.lock();
  }

  void unlock()
  {
    mutex_.unlock();
  }

#else
  StaticMutex() : handle_(xSemaphoreCreateMutexStatic(&buffer_))
  {
    configASSERT(handle_ != nullptr);
  }

  ~StaticMutex()
  {
    if (handle_ != nullptr) {
      vSemaphoreDelete(handle_);
    }
  }

  void lock()
  {
    xSemaphoreTake(handle_, portMAX_DELAY);
  }

  void unlock()
  {
    xSemaphoreGive(handle_);
  }
#endif

  StaticMutex(const StaticMutex&) = delete;
  StaticMutex& operator=(const StaticMutex&) = delete;
  StaticMutex(StaticMutex&&) = delete;
  StaticMutex& operator=(StaticMutex&&) = delete;

private:
#if defined(CONFIG_IDF_TARGET_LINUX)
  std::mutex mutex_;
#else
  StaticSemaphore_t buffer_;
  SemaphoreHandle_t handle_;
#endif
};

/// RAII scoped lock, mirrors std::lock_guard.
class StaticMutexGuard
{
public:
  explicit StaticMutexGuard(StaticMutex& m) : m_(m)
  {
    m_.lock();
  }

  ~StaticMutexGuard()
  {
    m_.unlock();
  }

  StaticMutexGuard(const StaticMutexGuard&) = delete;
  StaticMutexGuard& operator=(const StaticMutexGuard&) = delete;

private:
  StaticMutex& m_;
};

}  // namespace util
