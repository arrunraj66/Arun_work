// The node contract and the executor, tested with a fake clock and no
// sleeping anywhere  the same discipline as test_acoustic. Every timing
// property below is checked by handing step() a number, not by waiting.
//
// The one exception is the budget-overrun test, which has to burn real CPU
// time because a budget is a claim about real time. It burns milliseconds,
// not seconds, and it is the only test here that touches the real clock.

#include "mw/executor.hpp"
#include "mw/node.hpp"
#include "mw/time.hpp"

#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-62s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

/// A node that records everything that was done to it. Nothing here does any
/// real work  the point is to observe the executor, not to compute anything.
class SpyNode final : public mw::INode {
 public:
  SpyNode(std::string name, std::int64_t period_ns, std::int64_t budget_ns = 0)
      : name_(std::move(name)), period_ns_(period_ns), budget_ns_(budget_ns) {}

  [[nodiscard]] mw::NodeSpec spec() const override {
    mw::NodeSpec s;
    s.name      = name_;
    s.period_ns = period_ns_;
    s.budget_ns = budget_ns_;
    return s;
  }

  void configure() override {
    ++configures;
    if (fail_configure) throw std::runtime_error("configure refused");
  }

  void start() override {
    ++starts;
    if (fail_start) throw std::runtime_error("start refused");
  }

  void tick(std::int64_t now_ns) override {
    seen.push_back(now_ns);
    if (burn_ns > 0) {
      const std::int64_t until = mw::steady_time_ns() + burn_ns;
      while (mw::steady_time_ns() < until) { /* deliberately burning time */ }
    }
    if (throw_every > 0 && seen.size() % throw_every == 0) {
      throw std::runtime_error("tick refused");
    }
  }

  void stop() noexcept override { stop_order->push_back(name_); }

  std::string  name_;
  std::int64_t period_ns_;
  std::int64_t budget_ns_;

  std::vector<std::int64_t> seen;
  int configures = 0;
  int starts     = 0;

  bool         fail_configure = false;
  bool         fail_start     = false;
  std::size_t  throw_every    = 0;
  std::int64_t burn_ns        = 0;

  std::vector<std::string>* stop_order = nullptr;
};

constexpr std::int64_t kMs = 1'000'000;

}  // namespace

int main() {
  std::vector<std::string> stops;

  // 1. Lifecycle order is enforced, not merely documented.
  {
    mw::Executor ex(10 * kMs);
    SpyNode n("a", 0);
    n.stop_order = &stops;
    ex.add(n);

    bool threw = false;
    try { ex.step(0); } catch (const std::logic_error&) { threw = true; }
    check(threw, "step() before start_all() throws rather than ticking");

    threw = false;
    try { ex.start_all(); } catch (const std::logic_error&) { threw = true; }
    check(threw, "start_all() before configure_all() throws");

    ex.configure_all();
    ex.start_all();
    check(n.configures == 1 && n.starts == 1, "configure() and start() are each called exactly once");

    threw = false;
    try { ex.add(n); } catch (const std::logic_error&) { threw = true; }
    check(threw, "a node cannot be added once the executor is running");
    ex.stop_all();
  }

  // 2. The node is handed exactly the time the caller supplied. This is the
  //    property the whole replay story rests on.
  {
    mw::Executor ex(10 * kMs);
    SpyNode n("a", 0);
    n.stop_order = &stops;
    ex.add(n);
    ex.configure_all();
    ex.start_all();

    ex.step(1'000);
    ex.step(11'000);
    ex.step(21'000);
    ex.stop_all();

    check(n.seen == std::vector<std::int64_t>({1'000, 11'000, 21'000}),
          "tick() receives exactly the now_ns that step() was given");
    check(ex.steps() == 3, "the executor counts its own steps");
  }

  // 3. A node with period 0 ticks every step; a slower node ticks on its own
  //    period, with the executor's period as the base rate.
  {
    mw::Executor ex(10 * kMs);
    SpyNode fast("fast", 0);            // every step
    SpyNode slow("slow", 30 * kMs);     // every third step
    fast.stop_order = &stops;
    slow.stop_order = &stops;
    ex.add(fast);
    ex.add(slow);
    ex.configure_all();
    ex.start_all();

    for (int i = 0; i < 9; ++i) ex.step(static_cast<std::int64_t>(i) * 10 * kMs);
    ex.stop_all();

    check(fast.seen.size() == 9, "a period-0 node ticks on every step");
    check(slow.seen.size() == 3, "a 30 ms node ticks three times across 90 ms");
    check(slow.seen == std::vector<std::int64_t>({0, 30 * kMs, 60 * kMs}),
          "and it ticks on its own period boundaries, not the executor's");
  }

  // 4. Being late drops the missed ticks and counts them, rather than firing
  //    a catch-up burst.
  {
    mw::Executor ex(10 * kMs);
    SpyNode n("a", 10 * kMs);
    n.stop_order = &stops;
    ex.add(n);
    ex.configure_all();
    ex.start_all();

    ex.step(0);            // first tick, schedule starts
    ex.step(55 * kMs);     // five periods late: slots at 10,20,30,40 skipped
    ex.stop_all();

    check(n.seen.size() == 2, "a late step runs the node once, not once per missed period");
    check(ex.reports()[0].missed == 4, "and the four skipped slots are counted");
  }

  // 5. A throwing tick is contained: counted, and the next node still runs.
  {
    mw::Executor ex(10 * kMs);
    SpyNode bad("bad", 0);
    SpyNode good("good", 0);
    bad.throw_every  = 1;   // throws on every tick
    bad.stop_order   = &stops;
    good.stop_order  = &stops;
    ex.add(bad);
    ex.add(good);
    ex.configure_all();
    ex.start_all();

    for (int i = 0; i < 4; ++i) ex.step(static_cast<std::int64_t>(i) * 10 * kMs);
    ex.stop_all();

    check(ex.reports()[0].faults == 4, "every throwing tick is counted as a fault");
    check(ex.reports()[0].ticks == 4, "a faulted tick still counts as a tick that happened");
    check(good.seen.size() == 4, "a node that throws does not stop the node after it");
  }

  // 6. Budget overruns are measured against the real clock. The only test
  //    here that burns time, and it burns 3 ms.
  {
    mw::Executor ex(10 * kMs);
    SpyNode heavy("heavy", 0, /*budget_ns=*/1 * kMs);
    SpyNode light("light", 0, /*budget_ns=*/50 * kMs);
    heavy.burn_ns   = 3 * kMs;
    heavy.stop_order = &stops;
    light.stop_order = &stops;
    ex.add(heavy);
    ex.add(light);
    ex.configure_all();
    ex.start_all();

    ex.step(0);
    ex.stop_all();

    check(ex.reports()[0].overruns == 1, "a tick over its declared budget is counted as an overrun");
    check(ex.reports()[0].max_ns >= 3 * kMs, "and the measured duration reflects real elapsed time");
    check(ex.reports()[1].overruns == 0, "a tick inside its budget is not");
    check(ex.reports()[1].budget_ns == 50 * kMs, "the report carries the budget the node declared");
  }

  // 7. Teardown mirrors construction.
  {
    stops.clear();
    mw::Executor ex(10 * kMs);
    SpyNode a("a", 0), b("b", 0), c("c", 0);
    a.stop_order = &stops;
    b.stop_order = &stops;
    c.stop_order = &stops;
    ex.add(a);
    ex.add(b);
    ex.add(c);
    ex.configure_all();
    ex.start_all();
    ex.step(0);
    ex.stop_all();

    check(stops == std::vector<std::string>({"c", "b", "a"}),
          "stop() runs in reverse order of add()");

    const std::size_t before = stops.size();
    ex.stop_all();
    check(stops.size() == before, "stop_all() is safe to call twice");
  }

  // 8. A node that refuses to start does not leave its neighbours running.
  {
    stops.clear();
    mw::Executor ex(10 * kMs);
    SpyNode ok("ok", 0);
    SpyNode bad("bad", 0);
    bad.fail_start = true;
    ok.stop_order  = &stops;
    bad.stop_order = &stops;
    ex.add(ok);
    ex.add(bad);
    ex.configure_all();

    bool threw = false;
    try { ex.start_all(); } catch (const std::runtime_error&) { threw = true; }
    check(threw, "start_all() reports a node that refused to start");
    check(stops == std::vector<std::string>({"ok"}),
          "and the nodes already started are stopped again");
  }

  // 9. A node that refuses to configure is named in the error.
  {
    mw::Executor ex(10 * kMs);
    SpyNode bad("navigator", 0);
    bad.fail_configure = true;
    bad.stop_order     = &stops;
    ex.add(bad);

    std::string message;
    try { ex.configure_all(); } catch (const std::runtime_error& e) { message = e.what(); }
    check(message.find("navigator") != std::string::npos,
          "a configure failure names the node that caused it");
  }

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures,
              failures == 1 ? "" : "s");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}