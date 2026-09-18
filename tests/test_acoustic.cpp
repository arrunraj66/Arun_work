// LoopbackBearer proves the three properties every real acoustic bearer will
// have: one frame on the channel at a time, real transit time, and frames
// that can simply vanish. No sleeping  the fake clock is advanced by hand.

#include "mw/acoustic_loopback.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-58s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

}  // namespace

int main() {
  // 1. a frame that fits arrives after serialisation + propagation
  {
    mw::LoopbackBearer::Config cfg;
    cfg.propagation_ns   = 300'000'000;   // 300 ms
    cfg.bits_per_second  = 80;
    cfg.loss_probability = 0.0;           // deterministic: nothing lost
    cfg.max_frame_bytes  = 64;
    mw::LoopbackBearer bearer(cfg);

    const std::string frame(8, 'x');      // 8 bytes = 64 bits
    // serialisation = 64 bits / 80 bps = 800 ms, so arrival = 800 + 300 = 1100 ms
    const bool accepted = bearer.offer(frame, mw::Priority::kNormal, 2'000'000'000);
    check(accepted, "a frame within its deadline is accepted");
    check(bearer.delivered().empty(), "nothing has arrived yet at t=0");

    bearer.advance(1'099'000'000);
    check(bearer.delivered().empty(), "1 ms before arrival, still nothing");

    bearer.advance(2'000'000);
    check(bearer.delivered().size() == 1, "arrival lands where the arithmetic says it should");
    check(bearer.delivered()[0].bytes == frame, "the delivered bytes are the bytes that were sent");
  }

  // 2. a deadline that cannot be met is refused before it touches the channel
  {
    mw::LoopbackBearer::Config cfg;
    cfg.propagation_ns  = 300'000'000;
    cfg.bits_per_second = 80;
    mw::LoopbackBearer bearer(cfg);

    const std::string frame(8, 'x');      // needs ~1100 ms total
    const bool accepted = bearer.offer(frame, mw::Priority::kUrgent, 500'000'000);
    check(!accepted, "a deadline the physics cannot meet is refused, not attempted");
    check(bearer.stats().rejected_late == 1, "and counted as rejected-late");
  }

  // 3. the channel is half-duplex: a second frame is refused while the first
  //    is still serialising, even though it is not the same instant
  {
    mw::LoopbackBearer::Config cfg;
    cfg.propagation_ns  = 100'000'000;
    cfg.bits_per_second = 80;
    mw::LoopbackBearer bearer(cfg);

    const std::string frame(8, 'x');      // 800 ms to serialise
    check(bearer.offer(frame, mw::Priority::kNormal, 10'000'000'000), "first frame is accepted");

    bearer.advance(400'000'000);          // channel still busy: 800 ms not up yet
    check(!bearer.offer(frame, mw::Priority::kUrgent, 10'000'000'000),
          "a second frame is refused mid-transmission, regardless of priority");
    check(bearer.stats().rejected_busy == 1, "and counted as rejected-busy");

    bearer.advance(400'000'000);          // now past 800 ms: channel free
    check(bearer.offer(frame, mw::Priority::kNormal, 10'000'000'000),
          "once the channel frees, the next offer is accepted");
  }

  // 4. oversized frames never touch the channel or the clock
  {
    mw::LoopbackBearer::Config cfg;
    cfg.max_frame_bytes = 4;
    mw::LoopbackBearer bearer(cfg);

    const std::string frame(8, 'x');
    check(!bearer.offer(frame, mw::Priority::kBulk, 10'000'000'000),
          "a frame over max_frame_bytes is refused");
    check(bearer.stats().rejected_size == 1, "and counted as rejected-size, not rejected-busy");
  }

  // 5. loss is real: a bearer forced to lose everything accepts but delivers nothing
  {
    mw::LoopbackBearer::Config cfg;
    cfg.propagation_ns   = 10'000'000;
    cfg.bits_per_second  = 8000;
    cfg.loss_probability = 1.0;           // deterministic: everything lost
    mw::LoopbackBearer bearer(cfg);

    const std::string frame(4, 'x');
    const bool accepted = bearer.offer(frame, mw::Priority::kNormal, 10'000'000'000);
    check(accepted, "the bearer still accepts  the sender is never told about loss");

    bearer.advance(1'000'000'000);
    check(bearer.delivered().empty(), "but nothing ever arrives");
    check(bearer.stats().lost == 1, "and the loss is visible only to the stats a test can see");
  }

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
              failures, failures == 1 ? "" : "s");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}