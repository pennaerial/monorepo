#include "encoder.hpp"

namespace drivers
{

Encoder::Encoder(PayloadCorner corner) : corner_(corner) {}

PayloadCorner Encoder::corner() const
{
  return corner_;
}

void Encoder::write_latest(const payload_interfaces_msg_MotorState& msg)
{
  util::StaticMutexGuard lock(mtx_);
  reading_ = msg;
  ++update_count_;
}

payload_interfaces_msg_MotorState Encoder::get_latest()
{
  util::StaticMutexGuard lock(mtx_);
  return reading_;
}

bool Encoder::get_latest_if_updated(uint32_t& last_update_count, payload_interfaces_msg_MotorState& msg)
{
  util::StaticMutexGuard lock(mtx_);
  if (update_count_ == last_update_count) {
    return false;
  }

  msg = reading_;
  last_update_count = update_count_;
  return true;
}

}  // namespace drivers
