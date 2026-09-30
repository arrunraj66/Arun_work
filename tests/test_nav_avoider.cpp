// nav::decide() is a pure function, so this test is pure numbers in,
// numbers out -- no ZeroMQ, no protobuf, no Webots. Covers the three tiers
// (clear / slowing / stopped-and-turning), the sign convention (positive
// turn_rate = toward the more open side = left), and the fail-safe default
// (a sector that has never received data reads as 0.0, which must produce
// a stop, never a cruise).

#include "nav/avoider.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-58s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr nav::AvoiderConfig kDefaultCfg{};  // stop=0.5, slow=2.5, cruise=1.0, max_turn=1.0

}  // namespace

int main() {
  // Tier 1: front clear, sides irrelevant -- straight ahead, no turning.
  {
    const nav::NavCommand cmd = nav::decide({/*front=*/10.0F, /*left=*/1.0F, /*right=*/100.0F}, kDefaultCfg);
    check(cmd.forward_speed == kDefaultCfg.cruise_speed, "tier1: full cruise speed");
    check(cmd.turn_rate == 0.0F, "tier1: no turning even with lopsided sides");
    check(!cmd.obstacle_detected, "tier1: obstacle_detected is false");
  }

  // Tier 3: front obstacle inside stop_distance, sides equal -> forced but
  // deterministic choice (left, per the tie-break rule).
  {
    const nav::NavCommand cmd = nav::decide({/*front=*/0.1F, /*left=*/kInf, /*right=*/kInf}, kDefaultCfg);
    check(cmd.forward_speed == 0.0F, "tier3 (tied sides): forward_speed stops");
    check(cmd.turn_rate == kDefaultCfg.max_turn_rate, "tier3 (tied sides): ties break left");
    check(cmd.obstacle_detected, "tier3: obstacle_detected is true");
  }

  // Tier 3: front close, left is the only open side -> must turn LEFT
  // (positive turn_rate), not right.
  {
    const nav::NavCommand cmd = nav::decide({/*front=*/0.1F, /*left=*/5.0F, /*right=*/0.2F}, kDefaultCfg);
    check(cmd.forward_speed == 0.0F, "tier3 (left open): forward_speed stops");
    check(cmd.turn_rate > 0.0F, "tier3 (left open): turns toward the open side (positive)");
  }

  // Tier 3: mirror of the above -- right is the only open side -> must
  // turn RIGHT (negative turn_rate). This is the sign-convention test that
  // matters most: get this backwards and the vehicle steers INTO
  // obstacles instead of away from them.
  {
    const nav::NavCommand cmd = nav::decide({/*front=*/0.1F, /*left=*/0.2F, /*right=*/5.0F}, kDefaultCfg);
    check(cmd.turn_rate < 0.0F, "tier3 (right open): turns toward the open side (negative)");
  }

  // Tier 2: front partway into the slow band -> partial speed, partial
  // turn, both strictly between the tier-1 and tier-3 extremes.
  {
    const float midpoint = (kDefaultCfg.stop_distance + kDefaultCfg.slow_distance) / 2.0F;
    const nav::NavCommand cmd = nav::decide({/*front=*/midpoint, /*left=*/10.0F, /*right=*/1.0F}, kDefaultCfg);
    check(cmd.forward_speed > 0.0F && cmd.forward_speed < kDefaultCfg.cruise_speed,
          "tier2: forward_speed is a partial value, not 0 or full cruise");
    check(cmd.turn_rate > 0.0F && cmd.turn_rate <= kDefaultCfg.max_turn_rate,
          "tier2: turns toward the more open left side, within bounds");
    check(cmd.obstacle_detected, "tier2: obstacle_detected is true");
  }

  // Tier 2 boundary: exactly at slow_distance should still count as "in
  // the band" (the <= in decide()'s tier-2 condition), producing urgency
  // 0 -- full cruise speed, zero turn, but obstacle_detected still true.
  {
    const nav::NavCommand cmd =
        nav::decide({/*front=*/kDefaultCfg.slow_distance, /*left=*/5.0F, /*right=*/5.0F}, kDefaultCfg);
    check(std::abs(cmd.forward_speed - kDefaultCfg.cruise_speed) < 1e-5F,
          "tier2 boundary: urgency 0 gives full cruise speed");
    check(cmd.obstacle_detected, "tier2 boundary: still counts as obstacle_detected");
  }

  // Fail-safe default: front == 0.0 (the sentinel nav_avoid_main uses for
  // "no scan received on this topic yet") must stop the vehicle, exactly
  // like a real close-range obstacle would -- missing data must never
  // look like a clear path.
  {
    const nav::NavCommand cmd = nav::decide({/*front=*/0.0F, /*left=*/0.0F, /*right=*/0.0F}, kDefaultCfg);
    check(cmd.forward_speed == 0.0F, "fail-safe: no data yet stops the vehicle, not cruises it");
  }

  // turn_rate must never exceed max_turn_rate even with a maximally
  // lopsided side reading (one side inf, the other tiny).
  {
    const nav::NavCommand cmd = nav::decide({/*front=*/0.1F, /*left=*/0.001F, /*right=*/kInf}, kDefaultCfg);
    check(std::abs(cmd.turn_rate) <= kDefaultCfg.max_turn_rate, "turn_rate stays within max_turn_rate");
  }

  if (failures > 0) {
    std::printf("%d check(s) FAILED\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("all checks passed\n");
  return EXIT_SUCCESS;
}
