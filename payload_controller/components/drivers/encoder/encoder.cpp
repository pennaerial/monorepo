#include "encoder.hpp"

namespace drivers
{

void Encoder::write_latest(const int64_t ticks)
{
  util::StaticMutexGuard lock(mtx_);
  ticks_ = ticks;
  ++update_count_;
}

int64_t Encoder::get_latest()
{
  util::StaticMutexGuard lock(mtx_);
  return ticks_;
}

bool Encoder::get_latest_if_updated(uint32_t& last_update_count, int64_t& ticks)
{
  util::StaticMutexGuard lock(mtx_);
  if (update_count_ == last_update_count) {
    return false;
  }

  ticks = ticks_;
  last_update_count = update_count_;
  return true;
}

}  // namespace drivers
