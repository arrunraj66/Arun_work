#pragma once
//
// nav/nav_publisher.hpp — NavPublisher, same shape as lidar::CloudPublisher.
// Built on mw::Publisher; hides that a NavCommand has to become bytes
// before it can leave this process. Neither <zmq.hpp>, the protobuf
// headers, nor mw/transport.hpp appear here.

#include <cstdint>
#include <memory>
#include <string>

#include "nav/command.hpp"

namespace nav {

/// Publishes NavCommands to anyone who subscribes -- in practice, right
/// now, the Webots controller. Same fan-out, fire-and-forget contract as
/// every other Publisher in this project: never blocks, a command sent
/// with nobody listening is simply gone (which is fine -- the consumer
/// just keeps acting on its last-received command; see nav_avoid_main.cpp
/// and the Webots controller integration notes).
class NavPublisher {
 public:
  /// `endpoint` is a ZeroMQ address, which this publisher BINDS.
  /// `topic` defaults to "nav.cmd" -- distinct from every lidar topic so
  /// nothing downstream can mix them up.
  explicit NavPublisher(std::string endpoint, std::string topic = "nav.cmd");
  ~NavPublisher();

  NavPublisher(const NavPublisher&) = delete;
  NavPublisher& operator=(const NavPublisher&) = delete;
  NavPublisher(NavPublisher&&) noexcept;
  NavPublisher& operator=(NavPublisher&&) noexcept;

  /// Serialises `cmd` and sends it. Returns false if the send queue was
  /// full and the command was dropped.
  [[nodiscard]] bool publish(const NavCommand& cmd) noexcept;

  /// How many commands this publisher has dropped since it was created.
  [[nodiscard]] std::uint64_t dropped() const noexcept;

  /// The topic this publisher sends under.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav
