// mw_executor_demo  the same executor the test drives with a fake clock,
// driven here by the real one.
//
// Nothing about the Executor changes between the two. The test hands it
// numbers it invented; this program hands it mw::steady_time_ns(). That the
// class cannot tell the difference is the entire reason the timing behaviour
// of this system is testable at all.
//
// Three nodes at three rates, one of which deliberately overruns its budget,
// so the report at the end has something in the overrun column.
//
// Note what the nodes do NOT do: talk to each other. There is no way for them
// to, yet  moving data between nodes by topic name is layer L3, the next
// stage. A node at this stage computes and nothing more, which is exactly how
// much of the contract is actually built.

#include "mw/executor.hpp"
#include "mw/node.hpp"
#include "mw/time.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

namespace {

constexpr std::int64_t kMs = 1'000'000;

/// A pretend depth sensor: cheap, frequent, well inside its budget.
class DepthNode final : public mw::INode {
 public:
  [[nodiscard]] mw::NodeSpec spec() const override {
    mw::NodeSpec s;
    s.name      = "depth";
    s.period_ns = 10 * kMs;   // 100 Hz
    s.budget_ns = 1 * kMs;
    s.publishes = {"nav/depth"};
    return s;
  }

  void configure() override {}
  void start() override {}
  void stop() noexcept override {}

  void tick(std::int64_t now_ns) override {
    // The only input is the time it was handed. Same times in, same depth
    // out, on any machine, forever -- that is what replayable means.
    const double t = static_cast<double>(now_ns) / 1e9;
    depth_m = 12.0 + 0.5 * std::sin(t);
    ++count;
  }

  double depth_m = 0.0;
  long   count   = 0;
};

/// A pretend controller: slower, and it occasionally takes far too long.
class ControlNode final : public mw::INode {
 public:
  [[nodiscard]] mw::NodeSpec spec() const override {
    mw::NodeSpec s;
    s.name       = "control";
    s.period_ns  = 20 * kMs;   // 50 Hz
    s.budget_ns  = 2 * kMs;    // and every tenth tick will blow straight through it
    s.subscribes = {"nav/depth"};
    s.publishes  = {"cmd/thrust"};
    return s;
  }

  void configure() override {}
  void start() override {}
  void stop() noexcept override {}

  void tick(std::int64_t /*now_ns*/) override {
    ++count;
    if (count % 10 == 0) {
      // Stand-in for the thing that always turns out to be in the hot path:
      // a lock held too long, an allocation, a log line flushed to disk.
      const std::int64_t until = mw::steady_time_ns() + 4 * kMs;
      while (mw::steady_time_ns() < until) { /* burning the budget */ }
    }
  }

  long count = 0;
};

/// A pretend telemetry node: slow, cheap, and nothing ever goes wrong with it.
class TelemetryNode final : public mw::INode {
 public:
  [[nodiscard]] mw::NodeSpec spec() const override {
    mw::NodeSpec s;
    s.name       = "telemetry";
    s.period_ns  = 500 * kMs;  // 2 Hz
    s.budget_ns  = 5 * kMs;
    s.subscribes = {"nav/depth", "cmd/thrust"};
    return s;
  }

  void configure() override {}
  void start() override {}
  void stop() noexcept override {}
  void tick(std::int64_t /*now_ns*/) override { ++count; }

  long count = 0;
};

const char* rate_text(std::int64_t period_ns) {
  static std::string buf;
  if (period_ns == 0) return "every step";
  buf = std::to_string(1e9 / static_cast<double>(period_ns));
  buf = buf.substr(0, buf.find('.') + 2) + " Hz";
  return buf.c_str();
}

}  // namespace

int main() {
  constexpr std::int64_t kPeriod = 5 * kMs;   // the executor wakes at 200 Hz
  constexpr std::int64_t kRunFor = 2'000 * kMs;

  DepthNode     depth;
  ControlNode   control;
  TelemetryNode telemetry;

  mw::Executor ex(kPeriod);
  ex.add(depth);       // tick order is add order, every cycle, no exceptions
  ex.add(control);
  ex.add(telemetry);

  ex.configure_all();
  ex.start_all();

  std::printf("executor waking every %ld ms, running for %ld ms\n\n",
              static_cast<long>(kPeriod / kMs), static_cast<long>(kRunFor / kMs));

  // The real-time loop. Five lines, and it is the only place in the whole
  // system that sleeps. Note it schedules against a fixed origin rather than
  // sleeping for a period each time round: sleeping "5 ms from now" accrues
  // every late wake-up into permanent drift, while sleeping until
  // "origin + n * period" does not.
  const std::int64_t origin = mw::steady_time_ns();
  for (std::int64_t n = 0; n * kPeriod < kRunFor; ++n) {
    const std::int64_t target = origin + n * kPeriod;
    const std::int64_t now    = mw::steady_time_ns();
    if (now < target) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(target - now));
    }
    ex.step(mw::steady_time_ns() - origin);
  }

  ex.stop_all();

  std::printf("%-12s %-11s %8s %8s %9s %9s %9s %9s\n", "node", "rate", "ticks", "missed",
              "faults", "over", "mean us", "max us");
  for (const auto& r : ex.reports()) {
    const double mean_us = r.ticks ? static_cast<double>(r.total_ns) / static_cast<double>(r.ticks) / 1000.0 : 0.0;
    std::printf("%-12s %-11s %8llu %8llu %9llu %9llu %9.1f %9.1f\n", r.name.c_str(),
                rate_text(r.period_ns), static_cast<unsigned long long>(r.ticks),
                static_cast<unsigned long long>(r.missed),
                static_cast<unsigned long long>(r.faults),
                static_cast<unsigned long long>(r.overruns), mean_us,
                static_cast<double>(r.max_ns) / 1000.0);
  }

  std::printf("\n%llu executor steps, depth ticked %ld times, last depth %.3f m\n",
              static_cast<unsigned long long>(ex.steps()), depth.count, depth.depth_m);
  std::printf("control ticked %ld times, telemetry %ld times\n", control.count, telemetry.count);
}