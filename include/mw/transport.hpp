#pragma once
//
// mw/transport.hpp  layer L1, the in-hull transport.
//
// A thin wrapper over ZeroMQ. Three jobs and no more:
//   * own the ZeroMQ context for a process,
//   * send a (topic, payload) pair,
//   * receive a (topic, payload) pair, with a timeout.
//
// It knows nothing about protobuf. Whoever calls publish() has already turned
// a message into bytes; whoever calls receive() decides how to parse them.
// That is what keeps L1 a transport and not an application.
//
// Note what is NOT in this header: <zmq.hpp>.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace mw {

/// The ZeroMQ context. One per process, created before any socket and
/// destroyed after the last one.
class Context {
 public:
  Context();
  ~Context();

  Context(const Context&)            = delete;
  Context& operator=(const Context&) = delete;
  Context(Context&&) noexcept;
  Context& operator=(Context&&) noexcept;

  /// Escape hatch: the underlying void* ctx, for code that needs raw zmq.
  [[nodiscard]] void* native() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  // Both sockets need the real zmq::context_t inside Impl. Friendship keeps
  // that access inside this header instead of leaking a public accessor.
  friend class Publisher;
  friend class Subscriber;
};

/// One received frame pair.
struct Message {
  std::string topic;
  std::string payload;
};

/// A ZeroMQ PUB socket. Fan-out, fire and forget, never blocks on a slow peer.
class Publisher {
 public:
  /// `endpoint` is a ZeroMQ address: inproc://name, ipc:///tmp/name,
  /// tcp://*:5555. The publisher binds it.
  Publisher(Context& ctx, std::string_view endpoint);
  ~Publisher();

  Publisher(const Publisher&)            = delete;
  Publisher& operator=(const Publisher&) = delete;
  Publisher(Publisher&&) noexcept;
  Publisher& operator=(Publisher&&) noexcept;

  /// Send two frames: the topic, then the payload.
  /// Returns false if the send queue was full  the sample is dropped, and
  /// dropped() counts it. It does not block and it does not throw.
  [[nodiscard]] bool publish(std::string_view topic, std::string_view payload) noexcept;

  /// How many samples this publisher has dropped since it was created.
  [[nodiscard]] std::uint64_t dropped() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// A ZeroMQ SUB socket. Receives nothing at all until subscribe() is called.
class Subscriber {
 public:
  Subscriber(Context& ctx, std::string_view endpoint);
  ~Subscriber();

  Subscriber(const Subscriber&)            = delete;
  Subscriber& operator=(const Subscriber&) = delete;
  Subscriber(Subscriber&&) noexcept;
  Subscriber& operator=(Subscriber&&) noexcept;

  /// Accept every topic whose name starts with `prefix`.
  /// An empty prefix means "everything". Call it as often as you like.
  void subscribe(std::string_view prefix);
  void unsubscribe(std::string_view prefix);

  /// Wait up to `timeout_ms` for one message.
  /// Returns nothing if the timeout expired  that is normal, not an error.
  [[nodiscard]] std::optional<Message> receive(int timeout_ms) noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mw