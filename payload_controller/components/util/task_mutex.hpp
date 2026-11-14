#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace util
{
/// For data shared only by FreeRTOS tasks, including Linux SITL tasks.
class TaskMutex
{
public:
  TaskMutex() : handle_(xSemaphoreCreateMutexStatic(&buffer_))
  {
    configASSERT(handle_ != nullptr);
  }

  ~TaskMutex()
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

  TaskMutex(const TaskMutex&) = delete;
  TaskMutex& operator=(const TaskMutex&) = delete;

private:
  StaticSemaphore_t buffer_;
  SemaphoreHandle_t handle_;
};

class TaskMutexGuard
{
public:
  explicit TaskMutexGuard(TaskMutex& mutex) : mutex_(mutex)
  {
    mutex_.lock();
  }
  ~TaskMutexGuard()
  {
    mutex_.unlock();
  }
  TaskMutexGuard(const TaskMutexGuard&) = delete;
  TaskMutexGuard& operator=(const TaskMutexGuard&) = delete;

private:
  TaskMutex& mutex_;
};
}  // namespace util
