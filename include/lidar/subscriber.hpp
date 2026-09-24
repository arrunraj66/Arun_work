#pragma once
//
// lidar/subscriber.hpp — ScanSubscriber, Step 5: receive scans published
// by a ScanPublisher, possibly from another machine.
//
// Note what it derives from. A subscriber IS an IScanSource: something you
// ask "do you have a scan for me, wait up to N milliseconds." That is the
// same question you ask a SickScanSource, and it deserves the same answer
// shape. The consequence is that anything written against IScanSource --
// the recorder, a GUI, a future obstacle detector -- works unchanged
// whether the scans come off the sensor in this process or over the
// network from a different one. Nothing above this line has to know which.
//
// Neither <zmq.hpp>, the protobuf headers, nor mw/transport.hpp appear
// here, same discipline as everywhere else.

#include <cstdint>
#include <memory>
#include <string>

#include "lidar/source.hpp"

namespace lidar {

/// Receives Scans from a ScanPublisher.
class ScanSubscriber final : public IScanSource {
 public:
  /// `endpoint` is a ZeroMQ address, which this subscriber CONNECTS to --
  /// the mirror of the publisher's bind:
  ///   tcp://192.168.12.240:5556
  ///   ipc:///tmp/lidar.sock
  ///
  /// `topic` must match what the publisher sends under. ZeroMQ filters on
  /// it, so messages for other topics never reach this process at all.
  ///
  /// A note that will otherwise cost you an afternoon: PUB/SUB drops
  /// anything sent before a subscriber has finished connecting (the "slow
  /// joiner" problem). A subscriber started after the publisher will miss
  /// the first few scans, and that is ZeroMQ working as designed, not a
  /// bug here. For a 25 Hz sensor it is a non-issue; for anything where
  /// the first message matters, the answer is a different socket pattern,
  /// not a sleep.
  explicit ScanSubscriber(std::string endpoint, std::string topic = "lidar.scan");
  ~ScanSubscriber() override;

  ScanSubscriber(const ScanSubscriber&) = delete;
  ScanSubscriber& operator=(const ScanSubscriber&) = delete;
  ScanSubscriber(ScanSubscriber&&) noexcept;
  ScanSubscriber& operator=(ScanSubscriber&&) noexcept;

  /// IScanSource's contract, unchanged: true and `out` filled if a scan
  /// arrived inside the timeout, false on timeout. A message that arrives
  /// but fails to parse is counted in malformed() and treated as "nothing
  /// arrived" -- a corrupt sample must not look like a real one.
  [[nodiscard]] bool poll_scan(Scan& out, int timeout_ms) noexcept override;

  /// Messages received on our topic that could not be parsed as a Scan.
  /// Should be 0; anything else means a version mismatch between the
  /// publisher's wire format and ours.
  [[nodiscard]] std::uint64_t malformed() const noexcept;

  /// The topic this subscriber is filtering on.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar
