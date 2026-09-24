#pragma once
//
// lidar/publisher.hpp — ScanPublisher, Step 4: get scans out of this
// process and onto the network.
//
// Built ON TOP of mw::Publisher rather than on ZeroMQ directly. The
// middleware already owns "send a (topic, payload) pair over a PUB socket,
// never block, count what you drop" — reimplementing that here would mean
// two pieces of code to keep correct instead of one. What this class adds
// is the one thing the middleware deliberately does not know about:
// turning a lidar::Scan into bytes.
//
// Neither <zmq.hpp>, the protobuf headers, nor mw/transport.hpp appear
// here. A consumer of lidar::lidar links none of them.

#include <cstdint>
#include <memory>
#include <string>

#include "lidar/scan.hpp"

namespace lidar {

/// Publishes Scans to anyone who subscribes. Fan-out and fire-and-forget:
/// a publisher never blocks waiting for a subscriber, and a scan sent with
/// nobody listening is simply gone.
class ScanPublisher {
 public:
  /// `endpoint` is a ZeroMQ address, which this publisher BINDS:
  ///   tcp://*:5556          — any machine on the network may connect
  ///   ipc:///tmp/lidar.sock — same machine only, no TCP stack in the way
  ///
  /// `topic` is the subscription key a ScanSubscriber must match. It is
  /// sent as a separate first frame, so filtering happens in ZeroMQ rather
  /// than by deserialising every message and throwing most away.
  explicit ScanPublisher(std::string endpoint, std::string topic = "lidar.scan");
  ~ScanPublisher();

  // Owns a bound socket; two objects cannot share one. Moving is fine.
  ScanPublisher(const ScanPublisher&) = delete;
  ScanPublisher& operator=(const ScanPublisher&) = delete;
  ScanPublisher(ScanPublisher&&) noexcept;
  ScanPublisher& operator=(ScanPublisher&&) noexcept;

  /// Serialises `scan` and sends it. Returns false if the send queue was
  /// full and the scan was dropped.
  ///
  /// A false return is NOT an error worth stopping for: it means a
  /// subscriber is too slow, and dropping the sample is the correct
  /// behaviour for live sensor data. Check dropped() if you want to know
  /// how often it is happening.
  [[nodiscard]] bool publish(const Scan& scan) noexcept;

  /// How many scans this publisher has dropped since it was created.
  [[nodiscard]] std::uint64_t dropped() const noexcept;

  /// The topic this publisher sends under — a subscriber must match it.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar
