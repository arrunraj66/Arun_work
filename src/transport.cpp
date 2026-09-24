#include "mw/transport.hpp"

#include <zmq.hpp>

#include <chrono>
#include <string>
#include <utility>

namespace mw {
namespace {

// Queue depth, in messages, on both ends. When a publisher's queue is full
// ZeroMQ's PUB socket silently discards; we make that visible instead.
constexpr int kHighWaterMark = 1000;

// On close, throw away anything still queued instead of waiting for it.
// Without this a process can hang in its own destructor.
constexpr int kLingerMs = 0;

std::string to_string(std::string_view sv) { return std::string(sv); }

zmq::const_buffer as_buffer(std::string_view sv) noexcept {
  return zmq::const_buffer(sv.data(), sv.size());
}

}  // namespace

// ---------- Context ----------

struct Context::Impl {
  zmq::context_t ctx{1};   // 1 I/O thread: enough for ipc and modest tcp
};

Context::Context() : impl_(std::make_unique<Impl>()) {}
Context::~Context() = default;
Context::Context(Context&&) noexcept = default;
Context& Context::operator=(Context&&) noexcept = default;

void* Context::native() const noexcept { return impl_->ctx.handle(); }

// ---------- Publisher ----------

struct Publisher::Impl {
  zmq::socket_t sock;
  std::uint64_t dropped = 0;

  Impl(zmq::context_t& ctx, const std::string& endpoint)
      : sock(ctx, zmq::socket_type::pub) {
    sock.set(zmq::sockopt::sndhwm, kHighWaterMark);
    sock.set(zmq::sockopt::linger, kLingerMs);
    sock.bind(endpoint);
  }
};

Publisher::Publisher(Context& ctx, std::string_view endpoint)
    : impl_(std::make_unique<Impl>(ctx.impl_->ctx, to_string(endpoint))) {}

Publisher::~Publisher() = default;
Publisher::Publisher(Publisher&&) noexcept = default;
Publisher& Publisher::operator=(Publisher&&) noexcept = default;

bool Publisher::publish(std::string_view topic, std::string_view payload) noexcept {
  try {
    const auto sent_topic =
        impl_->sock.send(as_buffer(topic), zmq::send_flags::sndmore | zmq::send_flags::dontwait);
    if (!sent_topic) { ++impl_->dropped; return false; }

    const auto sent_body = impl_->sock.send(as_buffer(payload), zmq::send_flags::dontwait);
    if (!sent_body) { ++impl_->dropped; return false; }
    return true;
  } catch (const zmq::error_t&) {
    ++impl_->dropped;
    return false;
  }
}

std::uint64_t Publisher::dropped() const noexcept { return impl_->dropped; }

// ---------- Subscriber ----------

struct Subscriber::Impl {
  zmq::socket_t sock;

  Impl(zmq::context_t& ctx, const std::string& endpoint)
      : sock(ctx, zmq::socket_type::sub) {
    sock.set(zmq::sockopt::rcvhwm, kHighWaterMark);
    sock.set(zmq::sockopt::linger, kLingerMs);
    sock.connect(endpoint);
  }
};

Subscriber::Subscriber(Context& ctx, std::string_view endpoint)
    : impl_(std::make_unique<Impl>(ctx.impl_->ctx, to_string(endpoint))) {}

Subscriber::~Subscriber() = default;
Subscriber::Subscriber(Subscriber&&) noexcept = default;
Subscriber& Subscriber::operator=(Subscriber&&) noexcept = default;

void Subscriber::subscribe(std::string_view prefix) {
  impl_->sock.set(zmq::sockopt::subscribe, to_string(prefix));
}

void Subscriber::unsubscribe(std::string_view prefix) {
  impl_->sock.set(zmq::sockopt::unsubscribe, to_string(prefix));
}

std::optional<Message> Subscriber::receive(int timeout_ms) noexcept {
  try {
    impl_->sock.set(zmq::sockopt::rcvtimeo, timeout_ms);

    zmq::message_t topic;
    if (!impl_->sock.recv(topic, zmq::recv_flags::none)) return std::nullopt;
    if (!topic.more()) return std::nullopt;   // malformed: topic with no payload

    zmq::message_t body;
    if (!impl_->sock.recv(body, zmq::recv_flags::none)) return std::nullopt;

    return Message{topic.to_string(), body.to_string()};
  } catch (const zmq::error_t&) {
    return std::nullopt;
  }
}

// ---------- Requester ----------

namespace {

// Build a fresh REQ socket connected to `endpoint`. Pulled out on its own
// because Requester::request() calls this twice: once at construction, and
// again whenever a timeout forces the old socket to be thrown away.
zmq::socket_t make_req_socket(zmq::context_t& ctx, const std::string& endpoint) {
  zmq::socket_t sock(ctx, zmq::socket_type::req);
  sock.set(zmq::sockopt::linger, kLingerMs);
  // No RCVHWM/SNDHWM here on purpose: REQ/REP is one in flight at a time by
  // construction, a queue depth would never come into play.
  sock.connect(endpoint);
  return sock;
}

}  // namespace

struct Requester::Impl {
  zmq::context_t& ctx;   // reference: Context outlives every socket built on it
  std::string     endpoint;
  zmq::socket_t   sock;

  Impl(zmq::context_t& c, std::string ep)
      : ctx(c), endpoint(std::move(ep)), sock(make_req_socket(ctx, endpoint)) {}
};

Requester::Requester(Context& ctx, std::string_view endpoint)
    : impl_(std::make_unique<Impl>(ctx.impl_->ctx, to_string(endpoint))) {}

Requester::~Requester()                                = default;
Requester::Requester(Requester&&) noexcept              = default;
Requester& Requester::operator=(Requester&&) noexcept   = default;

std::optional<std::string> Requester::request(std::string_view payload,
                                              int timeout_ms) noexcept {
  try {
    if (!impl_->sock.send(as_buffer(payload), zmq::send_flags::none)) {
      // The send itself would need to block (send queue depth is 1 for REQ)
      // -- meaning a previous request is still outstanding. Recover the same
      // way a timeout does: this socket is in a bad state, replace it.
      impl_->sock = make_req_socket(impl_->ctx, impl_->endpoint);
      return std::nullopt;
    }

    zmq::pollitem_t item{impl_->sock.handle(), 0, ZMQ_POLLIN, 0};
    zmq::poll(&item, 1, std::chrono::milliseconds(timeout_ms));

    if ((item.revents & ZMQ_POLLIN) == 0) {
      // Timed out. The Lazy Pirate fix: a REQ socket that sent but never
      // received is stuck -- it will refuse a new send until it gets the
      // reply it is owed, which may never come. Throwing it away and
      // reconnecting a fresh one is what makes the NEXT call to request()
      // start clean instead of hanging on a dead server forever.
      impl_->sock = make_req_socket(impl_->ctx, impl_->endpoint);
      return std::nullopt;
    }

    zmq::message_t reply;
    if (!impl_->sock.recv(reply, zmq::recv_flags::none)) return std::nullopt;
    return reply.to_string();
  } catch (const zmq::error_t&) {
    impl_->sock = make_req_socket(impl_->ctx, impl_->endpoint);
    return std::nullopt;
  }
}

// ---------- Replier ----------

struct Replier::Impl {
  zmq::socket_t sock;

  Impl(zmq::context_t& ctx, const std::string& endpoint)
      : sock(ctx, zmq::socket_type::rep) {
    sock.set(zmq::sockopt::linger, kLingerMs);
    sock.bind(endpoint);
  }
};

Replier::Replier(Context& ctx, std::string_view endpoint)
    : impl_(std::make_unique<Impl>(ctx.impl_->ctx, to_string(endpoint))) {}

Replier::~Replier()                              = default;
Replier::Replier(Replier&&) noexcept             = default;
Replier& Replier::operator=(Replier&&) noexcept  = default;

std::optional<std::string> Replier::receive(int timeout_ms) noexcept {
  try {
    impl_->sock.set(zmq::sockopt::rcvtimeo, timeout_ms);
    zmq::message_t req;
    if (!impl_->sock.recv(req, zmq::recv_flags::none)) return std::nullopt;
    return req.to_string();
  } catch (const zmq::error_t&) {
    return std::nullopt;
  }
}

void Replier::reply(std::string_view payload) noexcept {
  try {
    // Once receive() has returned a request, REP's state machine guarantees
    // this send is legal and will not block on queue depth -- there is
    // exactly one reply owed, and this is it.
    (void)impl_->sock.send(as_buffer(payload), zmq::send_flags::none);
  } catch (const zmq::error_t&) {
    // Nothing to recover: the request is already consumed either way, and
    // serve_one()'s caller only ever learns "a request was served".
  }
}

}  // namespace mw