// Stage 2 gate, part one: a message round-trips, and a version mismatch is
// REJECTED rather than misparsed. The second half is the one that matters —
// silently misreading a message from a newer build is how a fleet ends up
// acting on a field that moved.

#include <iostream>
#include <string>

#include "mw/heartbeat.pb.h"
#include "mw/message.hpp"
#include "mw/time.hpp"

namespace {

int failures = 0;

void check(bool condition, const char* what) {
  if (!condition) {
    std::cerr << "FAIL: " << what << '\n';
    ++failures;
  }
}

}  // namespace

int main() {
  // ── build a heartbeat, exactly as a node would ───────────────────────────
  std::uint64_t sequence = 0;
  mw::proto::Heartbeat out;
  const std::uint64_t n = mw::stamp(*out.mutable_header(), "auv-sensors", sequence);

  out.set_node_name("auv-sensors");
  out.set_state(mw::proto::NODE_STATE_RUNNING);
  out.set_tick_count(1234);
  out.set_overruns(0);
  out.set_wcet_us(870);

  check(n == 0, "first stamp returns sequence 0");
  check(sequence == 1, "stamp advanced the caller's counter");

  // ── serialise ────────────────────────────────────────────────────────────
  std::string bytes;
  check(out.SerializeToString(&bytes), "serialise succeeded");
  check(!bytes.empty(), "serialised form is not empty");

  // ── the generic path: read the header without knowing the payload type ───
  mw::proto::Header header;
  const mw::HeaderStatus status = mw::peek_header(bytes, header);
  check(status == mw::HeaderStatus::kOk, "peek_header accepted the message");
  check(header.publisher() == "auv-sensors", "publisher survived");
  check(header.sequence() == 0, "sequence survived");
  check(header.publish_time_ns() > 0, "timestamp was stamped");

  // ── the typed path: full round trip ──────────────────────────────────────
  mw::proto::Heartbeat back;
  check(back.ParseFromString(bytes), "deserialise succeeded");
  check(back.node_name() == "auv-sensors", "node_name survived");
  check(back.state() == mw::proto::NODE_STATE_RUNNING, "state survived");
  check(back.tick_count() == 1234, "tick_count survived");
  check(back.wcet_us() == 870, "wcet_us survived");

  // ── version skew: a message from a newer build must be REFUSED ───────────
  mw::proto::Heartbeat from_the_future = out;
  from_the_future.mutable_header()->set_schema_version(mw::kSchemaVersion + 1);
  std::string future_bytes;
  check(from_the_future.SerializeToString(&future_bytes), "serialise future msg");

  mw::proto::Header ignored;
  check(mw::peek_header(future_bytes, ignored) == mw::HeaderStatus::kSchemaTooNew,
        "a newer schema version is rejected, not guessed at");

  // ── a message with no header at all ──────────────────────────────────────
  mw::proto::Heartbeat headerless;
  headerless.set_node_name("nobody");
  std::string headerless_bytes;
  check(headerless.SerializeToString(&headerless_bytes), "serialise headerless");
  check(mw::peek_header(headerless_bytes, ignored) == mw::HeaderStatus::kMissingHeader,
        "a missing header is detected");

  // ── random bytes must not be mistaken for a message ──────────────────────
  const std::string garbage = "\xff\xff\xff\xff\xff\xff\xff\xff";
  const mw::HeaderStatus g = mw::peek_header(garbage, ignored);
  check(g != mw::HeaderStatus::kOk, "garbage is not accepted");

  // ── the two clocks behave as advertised ──────────────────────────────────
  const std::int64_t a = mw::steady_time_ns();
  const std::int64_t b = mw::steady_time_ns();
  check(b >= a, "steady clock never goes backwards");
  check(mw::wall_time_ns() > 1'600'000'000'000'000'000LL,
        "wall clock is past 2020, so it is a real epoch time");

  if (failures == 0) std::cout << "all message tests passed\n";
  return failures == 0 ? 0 : 1;
}
