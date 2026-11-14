#include "camera.hpp"

namespace drivers
{
Camera& Camera::instance()
{
  static Camera camera;
  return camera;
}

void Camera::write_latest(const geometry_msgs_msg_Vector3& position)
{
  util::StaticMutexGuard lock(mutex_);
  position_ = position;
  ++update_count_;
}

bool Camera::get_latest_if_updated(uint32_t& last_update, geometry_msgs_msg_Vector3& position)
{
  util::StaticMutexGuard lock(mutex_);
  if (last_update == update_count_) {
    return false;
  }
  position = position_;
  last_update = update_count_;
  return true;
}

#if !defined(CONFIG_IDF_TARGET_LINUX)
// No hardware camera producer yet; the cache stays empty until write_latest is called.
void Camera::start() {}
#endif
}  // namespace drivers
