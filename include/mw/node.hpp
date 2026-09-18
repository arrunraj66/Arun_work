#pragma once
//
// mw/node.hpp  layer L2, the node contract.
//
// A node is a unit of work with a declared shape and a declared cost. Five
// methods, and the list of what it is NOT allowed to own is longer and more
// important than the list of what it does:
//
//   A node owns no thread.  Something else decides when it runs.
//   A node owns no socket.  Something else decides how its data moves.
//   A node owns no file.    Something else decides what is persisted.
//   A node owns no clock.   The time arrives as an argument to tick().
//
// That last one is the whole reason this header exists in the shape it does.
// A node that calls mw::wall_time_ns() inside tick() can never be replayed,
// because on replay the clock says "now" and the recording says "last
// Tuesday". A node that is HANDED the time replays perfectly: feed it the
// same times in the same order and it produces the same outputs, on a desk,
// six months later. Deterministic replay is not a feature you add at the end
//  it is a property you either preserve in this contract or lose forever.
//
// Retro-fitting an injected clock means touching every node that already
// exists, which is why it goes in before there is a second node.

#include <cstdint>
#include <string>
#include <vector>

namespace mw {

/// What a node declares about itself before it runs: who it is, how often it
/// needs to run, how long it may take, and which topics it touches.
///
/// This is a description, not configuration. The executor reads it to build
/// the schedule; the supervisor will later read it to check that every topic
/// somebody subscribes to is published by somebody. Both need it before the
/// node has done any work at all, which is why spec() is const and callable
/// on a node that has not been configured yet.
struct NodeSpec {
  std::string name;

  /// How often tick() should be called, in nanoseconds. 0 means "every time
  /// the executor wakes", whatever the executor's own period happens to be.
  std::int64_t period_ns = 0;

  /// How long one call to tick() may take before it counts as an overrun.
  /// 0 means "not declared", and no overrun is ever recorded.
  ///
  /// This is a budget, not a limit: nothing here stops a node exceeding it.
  /// A hard kill mid-tick would leave whatever the node was touching in an
  /// unknown state. The executor measures and counts; deciding what to do
  /// about a node that keeps overrunning belongs to the supervisor.
  std::int64_t budget_ns = 0;

  std::vector<std::string> publishes;
  std::vector<std::string> subscribes;
};

/// Everything the executor is allowed to ask of a node.
///
/// The order is fixed and the executor enforces it:
///
///   spec()        may be called at any time, and changes nothing
///   configure()   once, before start()    read settings, size buffers
///   start()       once, after configure()  acquire what running needs
///   tick(now)     many times, after start()
///   stop()        once, after the last tick
///
/// The split between configure() and start() is not ceremony. configure() is
/// where everything that could fail on bad input fails  a missing file, a
/// nonsense parameter, a buffer that cannot be sized. start() is where the
/// node commits to running. Separating them means a fleet of nodes can all be
/// configured first, and a single bad parameter is discovered before ANY node
/// has started doing anything that would then need undoing.
class INode {
 public:
  virtual ~INode() = default;

  [[nodiscard]] virtual NodeSpec spec() const = 0;

  /// Read configuration, size buffers, allocate. May throw: this is the
  /// phase where failure is expected and cheap.
  virtual void configure() = 0;

  /// Commit to running. May throw.
  virtual void start() = 0;

  /// Do one unit of work. `now_ns` is the time, handed in.
  ///
  /// NOT noexcept, deliberately, and this is a real decision rather than an
  /// oversight. A node that hits something impossible should be able to say
  /// so in the normal C++ way instead of swallowing it and returning a bool
  /// nobody checks. The executor catches whatever comes out, counts it, and
  /// keeps the other nodes running  it is the firewall, so that one broken
  /// node cannot take the vehicle with it.
  ///
  /// What tick() must not do: block, wait on a lock it might not get, or
  /// allocate on a path that runs every tick. It has a budget, and the
  /// executor is going to measure it.
  virtual void tick(std::int64_t now_ns) = 0;

  /// Release what start() acquired. noexcept, because this runs during
  /// shutdown  possibly a shutdown already caused by something going wrong 
  /// and a throw here would replace the original problem with a worse one.
  virtual void stop() noexcept = 0;
};

}  // namespace mw