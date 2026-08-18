#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// TODO: Test that this actually prevents race conditions somehow

namespace util {

/// RAII wrapper around a FreeRTOS mutex semaphore.
class StaticMutex {
public:
  StaticMutex() : handle_(xSemaphoreCreateMutexStatic(&buffer_)) {
    configASSERT(handle_ != nullptr);
  }

  ~StaticMutex() {
    if (handle_ != nullptr) {
      vSemaphoreDelete(handle_);
    }
  }

  // Non-copyable: a FreeRTOS semaphore handle can't be meaningfully
  // duplicated. Non-movable too, for simplicity (add a move ctor
  // later only if you actually need to relocate ownership).
  StaticMutex(const StaticMutex&) = delete;
  StaticMutex& operator=(const StaticMutex&) = delete;
  StaticMutex(StaticMutex&&) = delete;
  StaticMutex& operator=(StaticMutex&&) = delete;

  void lock() { xSemaphoreTake(handle_, portMAX_DELAY); }
  void unlock() { xSemaphoreGive(handle_); }

private:
  StaticSemaphore_t buffer_;
  SemaphoreHandle_t handle_;
};

/// RAII scoped lock, mirrors std::lock_guard.
class StaticMutexGuard {

public:
  explicit StaticMutexGuard(StaticMutex& m) : m_(m) { m_.lock(); }
  ~StaticMutexGuard() { m_.unlock(); }

  StaticMutexGuard(const StaticMutexGuard&) = delete;
  StaticMutexGuard& operator=(const StaticMutexGuard&) = delete;

private:
  StaticMutex& m_;
};

}  // namespace util
