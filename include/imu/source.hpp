#pragma once
//
// imu/source.hpp — the seam between "wherever IMU readings come from" and
// everything downstream (the publisher, and later, the fusion filter). One
// virtual method, one virtual destructor, no dependencies beyond imu.hpp.
// Exact same shape as lidar::IScanSource -- poll, don't call back.

#include "imu/imu.hpp"

namespace imu {

/// Anything that can produce Imu readings, polled rather than pushed.
///
/// Same "abstract class, not pimpl" reasoning as IScanSource: more than one
/// implementation must be swappable at runtime behind the same pointer --
/// a real SICK sensor's IMU channel today, a simulated one in Webots later,
/// maybe a different vendor after that.
class IImuSource {
 public:
  virtual ~IImuSource() = default;

  /// Waits up to timeout_ms for the next IMU reading.
  ///
  /// Returns true and fills `out` if a reading arrived within the timeout;
  /// returns false on timeout. A timeout is the ordinary, expected outcome
  /// of asking "do you have anything yet" -- it is not an error, and an
  /// implementation must not throw for it. `out` is left unspecified on a
  /// false return; callers must not read it.
  [[nodiscard]] virtual bool poll_imu(Imu& out, int timeout_ms) noexcept = 0;
};

}  // namespace imu
