#pragma once

#include <cstdint>

#include "geometry_msgs/msg/Vector3.h"
#include "static_mutex.hpp"

namespace drivers
{
// Latest estimated world position in metres (Z-up), with independent reader cursors.
// Producers only write the cache; the estimator owns all EKF state.
class Camera
{
public:
  static Camera& instance();
  void start();
  void write_latest(const geometry_msgs_msg_Vector3& position);
  bool get_latest_if_updated(uint32_t& last_update, geometry_msgs_msg_Vector3& position);

private:
  Camera() = default;
  geometry_msgs_msg_Vector3 position_{};
  uint32_t update_count_ = 0;
  util::StaticMutex mutex_;
};
}  // namespace drivers
