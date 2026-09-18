#pragma once
//
// mw/transport.hpp  layer L1, the in-hull transport.
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

  // All four socket types need the real zmq::context_t inside Impl.
  // Friendship keeps that access inside this header instead of leaking a
  // public accessor.
  friend class Publisher;
  friend class Subscriber;
  friend class Requester;
  friend class Replier;
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
  /// Returns false if the send queue was full  the sample is dropped, and
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
  /// Returns nothing if the timeout expired  that is normal, not an error.
  [[nodiscard]] std::optional<Message> receive(int timeout_ms) noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// A ZeroMQ REQ socket, for pull-based request/reply instead of PUB/SUB's
/// push-based fan-out. Where a Subscriber only ever sees what was published
/// WHILE it was listening, a Requester can ask for something specific,
/// whenever it wants it -- the right shape for "give me the latest scan" or
/// "give me everything between these two timestamps", and a workable backup
/// path when the live push stream is unavailable.
///
/// ZeroMQ's REQ socket has one hard rule: send, then receive, then send,
/// then receive, strictly alternating, never two sends or two receives in a
/// row. That is exactly what request() below does and the only thing it
/// does -- there is no separate send() or receive() to misuse out of order.
///
/// The harder problem is what happens when nobody answers. A plain REQ
/// socket has no real timeout: if the reply never comes, the socket is
/// stuck waiting to receive forever, and it will not accept a new request
/// either, because it is still owed a reply to the last one. request()
/// solves this the standard way (ZeroMQ's own guide calls it the "Lazy
/// Pirate" pattern): poll for the reply with a timeout, and if it never
/// arrives, throw the socket away and open a fresh one. The caller sees a
/// clean std::nullopt either way.
class Requester {
 public:
  Requester(Context& ctx, std::string_view endpoint);
  ~Requester();

  Requester(const Requester&)            = delete;
  Requester& operator=(const Requester&) = delete;
  Requester(Requester&&) noexcept;
  Requester& operator=(Requester&&) noexcept;

  /// Send `request` and wait up to `timeout_ms` for a reply.
  /// Returns nothing on timeout -- the socket is silently reopened so the
  /// NEXT call starts clean, rather than leaving REQ's send/receive
  /// alternation stuck on a reply that is never coming.
  [[nodiscard]] std::optional<std::string> request(std::string_view payload,
                                                    int timeout_ms) noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// A ZeroMQ REP socket: the other end of a Requester. Binds an endpoint and
/// answers one request at a time.
///
/// Like Requester, this has one method rather than separate receive/send
/// calls, because REP's alternation rule is the server's problem too: having
/// received a request, the ONLY thing this socket may legally do next is
/// send exactly one reply. serve_one() enforces that shape directly instead
/// of trusting a caller to get two separate calls right in order.
class Replier {
 public:
  Replier(Context& ctx, std::string_view endpoint);
  ~Replier();

  Replier(const Replier&)            = delete;
  Replier& operator=(const Replier&) = delete;
  Replier(Replier&&) noexcept;
  Replier& operator=(Replier&&) noexcept;

  /// Wait up to `timeout_ms` for one request. If one arrives, `handler` is
  /// called with its payload and must return the bytes to reply with --
  /// handing back a std::string rather than taking an out-parameter makes it
  /// impossible to forget to reply, which REP's state machine cannot
  /// tolerate. Returns true if a request was served, false on timeout (the
  /// ordinary case, not an error).
  template <typename Handler>
  bool serve_one(int timeout_ms, Handler&& handler) {
    std::optional<std::string> req = receive(timeout_ms);
    if (!req.has_value()) return false;
    reply(handler(*req));
    return true;
  }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  // The two halves serve_one() glues together. Private, and deliberately not
  // usable separately -- see the class comment.
  [[nodiscard]] std::optional<std::string> receive(int timeout_ms) noexcept;
  void reply(std::string_view payload) noexcept;
};

}  // namespace mw