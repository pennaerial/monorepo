#include "encoder.hpp"

#include "sdkconfig.h"

#if defined(CONFIG_IDF_TARGET_LINUX)
#include "encoder_sitl.hpp"
using EncoderBackend = drivers::Encoder_SITL;
#else
#include "encoder_hw.hpp"
using EncoderBackend = drivers::Encoder_HW;
#endif


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


Encoder& get_encoder(Side side)
{
  static EncoderBackend left(Side::LEFT);
  static EncoderBackend right(Side::RIGHT);
  return side == Side::LEFT ? left : right;
}


}  // namespace drivers
