#pragma once
//
// mw/executor.hpp — layer L2, the thing that runs nodes.
//
// One executor owns one loop and ticks a list of nodes in a fixed order. It
// is the only object in the system that decides when anything happens, which
// is the point: with one scheduler and no node-owned threads, the order of
// work in a cycle is a property you can read off the code rather than a race
// you have to reason about.
//
// step(now_ns) takes the time as an argument for the same reason tick(now)
// does. A test drives it with a fake clock and no sleeping at all; the real
// vehicle drives it from mw::steady_time_ns() in a timed loop. The executor
// cannot tell the difference, and that is what makes the timing behaviour of
// the whole system testable in milliseconds instead of minutes.
//
// Note there is no run() here. Owning the real-time loop — sleeping until the
// next period, deciding what to do about a node that keeps faulting — is the
// supervisor's job, in the next stage. This class does one pass and reports.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mw/node.hpp"

namespace mw {

class Executor {
 public:
  /// What the executor observed about one node. Numbers only — the executor
  /// counts and measures, it does not judge.
  struct Report {
    std::string name;
    std::int64_t period_ns = 0;  ///< from the node's spec; 0 means every step
    std::int64_t budget_ns = 0;  ///< copied from the node's spec

    std::uint64_t ticks    = 0;  ///< successful and faulted calls to tick()
    std::uint64_t faults   = 0;  ///< calls to tick() that threw
    std::uint64_t overruns = 0;  ///< calls that took longer than budget_ns
    std::uint64_t missed   = 0;  ///< scheduled ticks skipped because we were late

    std::int64_t last_ns  = 0;   ///< duration of the most recent tick
    std::int64_t max_ns   = 0;   ///< worst tick seen so far
    std::int64_t total_ns = 0;   ///< sum, for computing a mean
  };

  /// `period_ns` is how often the caller intends to call step(). The executor
  /// does not enforce it — it cannot, it does not own the loop — but it needs
  /// the number to schedule a node whose spec says period_ns == 0.
  explicit Executor(std::int64_t period_ns);
  ~Executor();

  Executor(const Executor&)            = delete;
  Executor& operator=(const Executor&) = delete;
  Executor(Executor&&) noexcept;
  Executor& operator=(Executor&&) noexcept;

  /// Register a node. The executor stores a REFERENCE: the node must outlive
  /// the executor, and the caller keeps ownership. An executor that owned its
  /// nodes would have to decide how they were allocated, and that is not its
  /// business.
  ///
  /// Order matters. Nodes tick in the order they were added, every cycle.
  void add(INode& node);

  /// configure() every node, in order. Throws on the first failure, with the
  /// offending node's name in the message. Nothing has started yet, so there
  /// is nothing to unwind.
  void configure_all();

  /// start() every node, in order. If one throws, every node already started
  /// is stopped again, in reverse order, before the exception is rethrown —
  /// a half-started system is never left running.
  void start_all();

  /// One pass: tick every node that is due at `now_ns`, in order, measuring
  /// each. Never throws, never blocks, never allocates.
  void step(std::int64_t now_ns);

  /// stop() every node in REVERSE order — teardown mirrors construction, so
  /// a node is never asked to shut down after something it depends on
  /// already has. Safe to call more than once.
  void stop_all() noexcept;

  [[nodiscard]] std::int64_t period_ns() const noexcept;
  [[nodiscard]] std::uint64_t steps() const noexcept;
  [[nodiscard]] const std::vector<Report>& reports() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mw
