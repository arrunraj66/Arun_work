#include "mw/executor.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

#include "mw/time.hpp"

namespace mw {
namespace {

// Where the executor is in its own lifecycle. The node contract says
// configure-then-start-then-tick; something has to actually enforce that, and
// a misuse should fail loudly on the first call rather than quietly produce a
// node that was ticked before it was started.
enum class Phase { kBuilding, kConfigured, kRunning, kStopped };

const char* to_text(Phase p) noexcept {
  switch (p) {
    case Phase::kBuilding:   return "building";
    case Phase::kConfigured: return "configured";
    case Phase::kRunning:    return "running";
    case Phase::kStopped:    return "stopped";
  }
  return "unknown";
}

}  // namespace

struct Executor::Impl {
  explicit Impl(std::int64_t period) : period_ns(period) {
    if (period <= 0) {
      throw std::invalid_argument("mw::Executor: period_ns must be positive");
    }
  }

  void require(Phase expected, const char* what) const {
    if (phase != expected) {
      throw std::logic_error(std::string("mw::Executor::") + what + ": executor is " +
                             to_text(phase) + ", expected " + to_text(expected));
    }
  }

  std::int64_t period_ns;
  Phase        phase = Phase::kBuilding;
  std::uint64_t step_count = 0;

  // Three parallel vectors rather than one vector of structs, and that is
  // deliberate: reports is handed out by const reference as the public
  // result, while nodes and next_due are private bookkeeping the caller has
  // no business seeing. Keeping them apart means reports() can return exactly
  // what it means to return, with no "ignore these fields" comment.
  std::vector<INode*>      nodes;
  std::vector<std::int64_t> next_due;
  std::vector<Report>       reports;

  // How many nodes start_all() got through. Only meaningful if it threw.
  std::size_t started = 0;
};

Executor::Executor(std::int64_t period_ns) : impl_(std::make_unique<Impl>(period_ns)) {}

Executor::~Executor() {
  // A destroyed executor must not leave nodes running. stop_all() is
  // idempotent and noexcept, so this is safe even if the caller already did
  // it, and safe during stack unwinding.
  if (impl_) stop_all();
}

Executor::Executor(Executor&&) noexcept            = default;
Executor& Executor::operator=(Executor&&) noexcept = default;

void Executor::add(INode& node) {
  impl_->require(Phase::kBuilding, "add");

  const NodeSpec spec = node.spec();
  if (spec.period_ns < 0 || spec.budget_ns < 0) {
    throw std::invalid_argument("mw::Executor::add: node '" + spec.name +
                                "' declares a negative period or budget");
  }

  Report r;
  r.name      = spec.name;
  r.period_ns = spec.period_ns;  // 0 stays 0: "every step"
  r.budget_ns = spec.budget_ns;

  impl_->nodes.push_back(&node);
  // Every node is due on the first step, whenever that turns out to be.
  impl_->next_due.push_back(std::numeric_limits<std::int64_t>::min());
  impl_->reports.push_back(std::move(r));
}

void Executor::configure_all() {
  impl_->require(Phase::kBuilding, "configure_all");

  for (std::size_t i = 0; i < impl_->nodes.size(); ++i) {
    try {
      impl_->nodes[i]->configure();
    } catch (const std::exception& e) {
      throw std::runtime_error("mw::Executor::configure_all: node '" + impl_->reports[i].name +
                               "' failed to configure: " + e.what());
    }
  }
  impl_->phase = Phase::kConfigured;
}

void Executor::start_all() {
  impl_->require(Phase::kConfigured, "start_all");

  impl_->started = 0;
  for (std::size_t i = 0; i < impl_->nodes.size(); ++i) {
    try {
      impl_->nodes[i]->start();
      ++impl_->started;
    } catch (const std::exception& e) {
      // Unwind by hand: everything already started gets stopped, newest
      // first, before the failure is reported. Leaving three nodes running
      // because the fourth refused to start is how a vehicle ends up in a
      // state nobody wrote down.
      for (std::size_t j = impl_->started; j > 0; --j) {
        impl_->nodes[j - 1]->stop();
      }
      impl_->started = 0;
      throw std::runtime_error("mw::Executor::start_all: node '" + impl_->reports[i].name +
                               "' failed to start: " + e.what());
    }
  }
  impl_->phase = Phase::kRunning;
}

void Executor::step(std::int64_t now_ns) {
  impl_->require(Phase::kRunning, "step");
  ++impl_->step_count;

  for (std::size_t i = 0; i < impl_->nodes.size(); ++i) {
    Report& r = impl_->reports[i];

    // period 0 means "every time the executor wakes" -- there is no schedule
    // to consult, and nothing that can ever be missed.
    if (r.period_ns > 0 && now_ns < impl_->next_due[i]) continue;

    // TWO CLOCKS, AND THEY ARE NOT THE SAME CLOCK.
    //
    // now_ns is the LOGICAL clock: what the node is told the time is. On
    // replay it comes out of a recording, so the node behaves identically.
    //
    // steady_time_ns() below is the MEASUREMENT clock: how long this tick
    // really took on this machine, right now. That number is a property of
    // the hardware and the load, it is different on every run, and it must
    // never reach the node  a node that saw it would stop being replayable.
    // It exists only so the executor can compare against the budget.
    const std::int64_t began = steady_time_ns();
    try {
      impl_->nodes[i]->tick(now_ns);
    } catch (...) {
      // The firewall. One node throwing is one node's problem; the rest of
      // the cycle still runs, and the fault is counted so a supervisor can
      // act on a pattern rather than on a single bad tick.
      ++r.faults;
    }
    const std::int64_t took = steady_time_ns() - began;

    ++r.ticks;
    r.last_ns   = took;
    r.total_ns += took;
    if (took > r.max_ns) r.max_ns = took;
    if (r.budget_ns > 0 && took > r.budget_ns) ++r.overruns;

    // Reschedule. If we are so late that more than one period has already
    // gone by, those ticks are DROPPED, not run back to back.
    //
    // This is a real choice and the alternative is worse. Catching up means
    // running a burst of ticks with stale times, which for a control loop
    // produces a lurch, and which digs the hole deeper because the burst
    // itself takes time. Dropping keeps the loop in real time and leaves a
    // counter that says exactly how much was lost.
    if (r.period_ns == 0) continue;  // nothing to reschedule

    const std::int64_t due = impl_->next_due[i];
    if (due == std::numeric_limits<std::int64_t>::min()) {
      // First tick ever: the schedule starts here, nothing was missed.
      impl_->next_due[i] = now_ns + r.period_ns;
    } else {
      // Slots at due, due+period, due+2*period, ... are all in the past by
      // now. We serviced one of them; the other k were skipped.
      const std::int64_t skipped = (now_ns - due) / r.period_ns;
      r.missed += static_cast<std::uint64_t>(skipped);
      impl_->next_due[i] = due + (skipped + 1) * r.period_ns;
    }
  }
}

void Executor::stop_all() noexcept {
  if (impl_->phase != Phase::kRunning) {
    impl_->phase = Phase::kStopped;
    return;
  }
  for (std::size_t i = impl_->nodes.size(); i > 0; --i) {
    impl_->nodes[i - 1]->stop();
  }
  impl_->phase = Phase::kStopped;
}

std::int64_t Executor::period_ns() const noexcept { return impl_->period_ns; }

std::uint64_t Executor::steps() const noexcept { return impl_->step_count; }

const std::vector<Executor::Report>& Executor::reports() const noexcept { return impl_->reports; }

}  // namespace mw