#pragma once
//
// mw/acoustic_loopback.hpp  a lossy, bandwidth-limited stand-in for a real
// acoustic modem, so everything built on AcousticBearer can be tested before
// there is hardware to test it against.
//
// It is not a simulation of JANUS or any real waveform. It reproduces the
// three properties that matter to a caller: the channel carries one frame at
// a time, a frame takes real time to cross, and frames can vanish with no
// notice. Get comfortable with those three and the real modem, whenever it
// arrives, will not surprise you.
//
// The clock is fake and advanced explicitly with advance(). A test for a
// 333 ms propagation delay should not spend 333 real milliseconds asleep to
// prove it  the same reasoning as the injected clock the node contract will
// use in stage 3.

#include "mw/acoustic.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace mw {

class LoopbackBearer : public AcousticBearer {
 public:
  struct Config {
    std::int64_t propagation_ns   = 333'000'000;  // 500 m at 1500 m/s
    std::int64_t bits_per_second  = 80;            // JANUS C-NAV order of magnitude
    double       loss_probability = 0.05;
    std::size_t  max_frame_bytes  = 64;
    unsigned     rng_seed         = 1;
  };

  /// What became of one accepted frame, visible only to the test or demo that
  /// is watching this bearer from the outside  a real bearer gives the
  /// sender none of this.
  struct Delivery {
    std::string   bytes;
    Priority      priority;
    std::int64_t  sent_at_ns;
    std::int64_t  arrives_at_ns;
  };

  struct Stats {
    std::uint64_t offered        = 0;  // every call to offer()
    std::uint64_t rejected_size  = 0;  // larger than max_frame_bytes
    std::uint64_t rejected_late  = 0;  // could not arrive before its deadline
    std::uint64_t rejected_busy  = 0;  // channel already carrying a frame
    std::uint64_t accepted       = 0;  // channel took it
    std::uint64_t lost           = 0;  // accepted, then the loss roll took it
  };

  explicit LoopbackBearer(Config cfg);

  [[nodiscard]] bool offer(std::string_view bytes, Priority priority,
                            Deadline deadline) noexcept override;
  [[nodiscard]] std::size_t max_frame_bytes() const noexcept override;

  /// Move the fake clock forward. Deliveries whose arrival time has now
  /// passed become visible in delivered().
  void advance(std::int64_t ns) noexcept;

  [[nodiscard]] std::int64_t now() const noexcept { return now_ns_; }

  /// Frames that have actually arrived, oldest first. Cleared by drain().
  [[nodiscard]] const std::vector<Delivery>& delivered() const noexcept;
  void drain() noexcept;

  [[nodiscard]] const Stats& stats() const noexcept { return stats_; }

 private:
  Config       cfg_;
  std::int64_t now_ns_           = 0;
  std::int64_t channel_free_at_  = 0;
  std::mt19937 rng_;
  std::bernoulli_distribution loss_roll_;
  Stats                 stats_;
  std::vector<Delivery> in_flight_;   // arrival time still in the future
  std::vector<Delivery> delivered_;   // arrival time now in the past
};

}  // namespace mw