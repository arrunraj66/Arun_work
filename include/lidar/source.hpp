#pragma once
//
// lidar/source.hpp — the seam between "wherever scans come from" and
// everything downstream (the publisher, the recorder). One virtual method,
// one virtual destructor, no dependencies beyond scan.hpp.
//
// This is Concept 4 from Lesson 2 applied to our own code, not just the GUI
// team's: poll, don't call back. Every concrete source — real hardware,
// a test double, anything else — is asked "do you have a scan for me," not
// given the chance to reach into caller code on its own thread.

#include "lidar/scan.hpp"

namespace lidar {

/// Anything that can produce Scans, polled rather than pushed.
///
/// Why an abstract class and not pimpl (contrast with Publisher/Subscriber
/// in the middleware): those have exactly one implementation each, so pimpl
/// just hides ZeroMQ. This has more than one implementation that must be
/// swappable at runtime behind the same pointer — real hardware today,
/// something else tomorrow — which is what virtual dispatch is for.
class IScanSource {
 public:
  virtual ~IScanSource() = default;

  /// Waits up to timeout_ms for the next scan.
  ///
  /// Returns true and fills `out` if a scan arrived within the timeout;
  /// returns false on timeout. A timeout is the ordinary, expected outcome
  /// of asking "do you have anything yet" — it is not an error, and an
  /// implementation must not throw for it. `out` is left unspecified on a
  /// false return; callers must not read it.
  [[nodiscard]] virtual bool poll_scan(Scan& out, int timeout_ms) noexcept = 0;
};

}  // namespace lidar
