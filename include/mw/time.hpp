// Two clocks, two jobs. Getting them the wrong way round is a classic bug, so
// the library offers one function for each and names them after what they are
// for rather than after which clock they use.

#pragma once

#include <cstdint>

namespace mw {

/// Wall-clock nanoseconds since the Unix epoch.
///
/// Use for TIMESTAMPS — things you will compare across processes, write into a
/// log, or show a human. It can jump backwards when NTP corrects the clock, so
/// never subtract two of these to measure how long something took.
[[nodiscard]] std::int64_t wall_time_ns() noexcept;

/// Nanoseconds from an arbitrary, fixed point in this process.
///
/// Use for DURATIONS — timing a tick, enforcing a deadline. It never jumps and
/// never goes backwards. The absolute value is meaningless; only differences
/// mean anything, and only within one process.
[[nodiscard]] std::int64_t steady_time_ns() noexcept;

}  // namespace mw
