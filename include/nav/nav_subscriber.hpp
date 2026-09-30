#pragma once
//
// nav/nav_subscriber.hpp — NavSubscriber, same shape as lidar::
// CloudSubscriber. This is what the Webots controller (and later, a real
// thruster driver) links -- it never needs to know NavCommand travels as
// protobuf over ZeroMQ underneath.

#include <cstdint>
#include <memory>
#include <string>

#include "nav/command.hpp"

namespace nav {

/// Receives NavCommands from a NavPublisher.
class NavSubscriber {
 public:
  /// `endpoint` is a ZeroMQ address, which this subscriber CONNECTS to.
  /// `topic` must match what the publisher sends under.
  ///
  /// Same slow-joiner caveat as every other Subscriber in this project: a
  /// NavSubscriber started after the publisher misses whatever was sent
  /// before it finished connecting. For a control loop polled every
  /// simulation step this is a non-issue -- the next command arrives
  /// within one tick either way.
  explicit NavSubscriber(std::string endpoint, std::string topic = "nav.cmd");
  ~NavSubscriber();

  NavSubscriber(const NavSubscriber&) = delete;
  NavSubscriber& operator=(const NavSubscriber&) = delete;
  NavSubscriber(NavSubscriber&&) noexcept;
  NavSubscriber& operator=(NavSubscriber&&) noexcept;

  /// True and `out` filled if a command arrived inside the timeout; false
  /// on timeout OR on a message that failed to parse (counted in
  /// malformed() either way) -- same "false just means nothing usable
  /// arrived this call" contract as IScanSource::poll_scan. Pass
  /// timeout_ms = 0 for a non-blocking poll, the way a per-simulation-step
  /// control loop wants: "is there a fresher command than the one I'm
  /// already acting on? if not, keep using the last one."
  [[nodiscard]] bool poll_command(NavCommand& out, int timeout_ms) noexcept;

  /// Messages received on our topic that could not be parsed as a
  /// NavCommand. Should be 0.
  [[nodiscard]] std::uint64_t malformed() const noexcept;

  /// The topic this subscriber is filtering on.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav
