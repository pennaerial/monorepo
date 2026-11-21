#pragma once

#include <array>
#include <cstddef>

namespace util
{
/// Bounded FIFO with explicit overrun reporting. The caller supplies synchronization.
/// On overrun, discard the oldest sample to retain the most recent time window.
template <typename T, std::size_t Capacity>
class SampleFifo
{
  static_assert(Capacity > 0);

public:
  void push(const T& sample)
  {
    if (count_ == Capacity) {
      head_ = (head_ + 1) % Capacity;
      --count_;
      ++dropped_;
    }
    samples_[(head_ + count_) % Capacity] = sample;
    ++count_;
  }

  bool pop(T& sample)
  {
    if (count_ == 0) {
      return false;
    }
    sample = samples_[head_];
    head_ = (head_ + 1) % Capacity;
    --count_;
    return true;
  }

  std::size_t size() const
  {
    return count_;
  }

  std::size_t take_dropped()
  {
    const auto dropped = dropped_;
    dropped_ = 0;
    return dropped;
  }

private:
  std::array<T, Capacity> samples_{};
  std::size_t head_ = 0;
  std::size_t count_ = 0;
  std::size_t dropped_ = 0;
};
}  // namespace util
