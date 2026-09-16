#include "mw/time.hpp"

#include <chrono>

namespace mw {
namespace {

// A tiny helper so the two functions below differ only in the clock they name.
template <typename Clock>
std::int64_t now_ns() noexcept {
  const auto since_epoch = Clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch).count();
}

}  // namespace

std::int64_t wall_time_ns() noexcept {
  return now_ns<std::chrono::system_clock>();
}

std::int64_t steady_time_ns() noexcept {
  return now_ns<std::chrono::steady_clock>();
}

}  // namespace mw
