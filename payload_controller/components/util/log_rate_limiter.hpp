#pragma once

#include <chrono>

namespace util
{
/// Allows the first log immediately and subsequent logs at most once per 20 ms; use one instance per owner/stream.
class LogRateLimiter
{
public:
  bool ready()
  {
    const auto now = std::chrono::steady_clock::now();
    if (has_logged_ && now - last_log_ < std::chrono::milliseconds(20)) {
      return false;
    }
    last_log_ = now;
    has_logged_ = true;
    return true;
  }

private:
  std::chrono::steady_clock::time_point last_log_{};
  bool has_logged_ = false;
};
}  // namespace util
