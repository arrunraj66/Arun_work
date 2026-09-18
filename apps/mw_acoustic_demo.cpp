// mw_acoustic_demo  run the swarm's slot-time arithmetic through the actual
// bearer, instead of just quoting the numbers on a slide.
//
// Ten vehicles, one shared channel, each offering a heartbeat in its own
// TDMA slot. Watch how many actually arrive per real second.

#include "mw/acoustic_loopback.hpp"

#include <cstdio>
#include <string>

int main() {
  mw::LoopbackBearer::Config cfg;
  cfg.propagation_ns   = 333'000'000;   // 500 m at 1500 m/s
  cfg.bits_per_second  = 80;            // JANUS C-NAV, order of magnitude
  cfg.loss_probability = 0.05;
  cfg.max_frame_bytes  = 64;
  cfg.rng_seed         = 7;

  mw::LoopbackBearer bearer(cfg);

  const int vehicles          = 10;
  const std::int64_t guard_ns = 200'000'000;                          // 200 ms
  const std::int64_t slot_ns  = cfg.propagation_ns + guard_ns;        // 533 ms
  const std::int64_t cycle_ns = slot_ns * vehicles;                   // 5.33 s

  std::printf("slot = %ld ms, cycle = %ld ms across %d vehicles (%.3f Hz per vehicle)\n\n",
              static_cast<long>(slot_ns / 1'000'000), static_cast<long>(cycle_ns / 1'000'000),
              vehicles, 1e9 / static_cast<double>(cycle_ns));

  const std::string heartbeat(4, 'H');   // stand-in for a real 4-byte status frame
  int offered = 0, accepted = 0;

  const int cycles_to_run = 5;
  for (int c = 0; c < cycles_to_run; ++c) {
    for (int v = 0; v < vehicles; ++v) {
      ++offered;
      // The deadline a vehicle actually cares about is its OWN next turn,
      // one full cycle away  not the slot boundary, which is only a
      // transmission-order convention. Watch what happens even so.
      const bool ok = bearer.offer(heartbeat, mw::Priority::kNormal,
                                    bearer.now() + cycle_ns);
      if (ok) ++accepted;
      bearer.advance(slot_ns);
    }
  }

  bearer.advance(1'000'000'000);   // let anything still in flight land

  const auto& st = bearer.stats();
  std::printf("offered %d\n", offered);
  std::printf("  rejected (too big for the frame)     : %llu\n",
              static_cast<unsigned long long>(st.rejected_size));
  std::printf("  rejected (could not meet its deadline): %llu\n",
              static_cast<unsigned long long>(st.rejected_late));
  std::printf("  rejected (channel already busy)       : %llu\n",
              static_cast<unsigned long long>(st.rejected_busy));
  std::printf("  accepted (channel took responsibility): %llu\n",
              static_cast<unsigned long long>(st.accepted));
  std::printf("  lost after acceptance (never known)   : %llu\n",
              static_cast<unsigned long long>(st.lost));
  std::printf("  actually delivered                    : %zu\n", bearer.delivered().size());
  std::printf("observed delivery rate: %.3f Hz per vehicle\n",
              static_cast<double>(bearer.delivered().size())
                  / static_cast<double>(vehicles)
                  / (static_cast<double>(cycles_to_run) * static_cast<double>(cycle_ns) / 1e9));
}