#include "mw/transport.hpp"

#include <zmq.hpp>

#include <string>

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

}  // namespace mw