#pragma once
//
// mw/acoustic.hpp  the second transport, and a deliberately different shape.
//
// The in-hull transport (mw/transport.hpp) is a socket: you publish, it either
// queues or drops, and the whole exchange is over in microseconds. Across
// water none of that is true. A frame takes hundreds of milliseconds to
// arrive, the channel can carry only one frame at a time, and whether a given
// byte is worth sending at all depends on what it is and how late it can
// afford to be.
//
// So the Acoustic Bearer is not a socket wrapper. It is one method, and the
// method's name says what it actually offers:
//
//   offer(bytes, priority, deadline) -> bool
//
// "Offer", not "send". The answer is very often no  the channel is busy, or a
// higher-priority frame is already queued, or the deadline has already
// passed. True means the bearer has accepted responsibility for trying, not
// that delivery is confirmed. There is no confirmation. Underwater acoustic
// telemetry does not have to pretend to be reliable.

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace mw {

/// How much it costs to lose this frame. Higher survives contention.
enum class Priority : std::uint8_t {
  kBulk    = 0,  // a scan, a log segment  fine to lose, fine to wait
  kNormal  = 1,  // routine telemetry
  kUrgent  = 2,  // a command, a fault report
};

[[nodiscard]] std::string_view to_string(Priority p) noexcept;

/// An absolute point in time, in nanoseconds since the steady clock's epoch 
/// the same clock as mw::steady_time_ns(). Not a duration: "500 ms from now"
/// is a decision the caller makes once, not a countdown the bearer has to
/// keep re-evaluating.
using Deadline = std::int64_t;

/// The interface every acoustic implementation satisfies  the real modem
/// driver eventually, a lossy loopback for every test until then.
class AcousticBearer {
 public:
  virtual ~AcousticBearer() = default;

  /// Try to send `bytes` before `deadline`. May refuse outright (false) or
  /// accept and still fail to deliver later  this return value covers only
  /// whether the bearer took responsibility for trying.
  [[nodiscard]] virtual bool offer(std::string_view bytes, Priority priority,
                                    Deadline deadline) noexcept = 0;

  /// The largest single frame this bearer can carry. offer() refuses
  /// anything larger without inspecting priority or deadline at all.
  [[nodiscard]] virtual std::size_t max_frame_bytes() const noexcept = 0;
};

}  // namespace mw