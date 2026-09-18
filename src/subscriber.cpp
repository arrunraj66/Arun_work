#include "lidar/subscriber.hpp"

#include <optional>
#include <string>
#include <utility>

#include "mw/transport.hpp"
#include "scan_codec.hpp"

namespace lidar {

struct ScanSubscriber::Impl {
  Impl(std::string ep, std::string t)
      : endpoint(std::move(ep)), topic(std::move(t)), subscriber(ctx, endpoint) {
    // A SUB socket receives NOTHING until it subscribes -- that is ZeroMQ's
    // design, not an oversight. Doing it here means a constructed
    // ScanSubscriber is already listening, with no second step a caller
    // could forget, the same reasoning as SickScanSource connecting in its
    // constructor.
    subscriber.subscribe(topic);
  }

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  std::string endpoint;
  std::string topic;

  // ctx declared before subscriber: it must outlive the socket.
  mw::Context ctx;
  mw::Subscriber subscriber;

  std::uint64_t malformed = 0;

  // Reused across calls, same reasoning as the publisher's.
  proto::LidarScan wire;
};

ScanSubscriber::ScanSubscriber(std::string endpoint, std::string topic)
    : impl_(std::make_unique<Impl>(std::move(endpoint), std::move(topic))) {}

ScanSubscriber::~ScanSubscriber() = default;
ScanSubscriber::ScanSubscriber(ScanSubscriber&&) noexcept = default;
ScanSubscriber& ScanSubscriber::operator=(ScanSubscriber&&) noexcept = default;

bool ScanSubscriber::poll_scan(Scan& out, int timeout_ms) noexcept {
  try {
    std::optional<mw::Message> msg = impl_->subscriber.receive(timeout_ms);
    if (!msg.has_value()) {
      return false;  // timeout: the ordinary case, per IScanSource's contract
    }

    if (!impl_->wire.ParseFromString(msg->payload)) {
      // Arrived on our topic but isn't a Scan we understand. Counted, not
      // thrown, and reported as "nothing arrived" -- a corrupt sample must
      // never be handed back looking like a real measurement.
      ++impl_->malformed;
      return false;
    }

    out = from_proto(impl_->wire);
    return true;
  } catch (...) {
    return false;  // declared noexcept; see ScanPublisher::publish
  }
}

std::uint64_t ScanSubscriber::malformed() const noexcept { return impl_->malformed; }

const std::string& ScanSubscriber::topic() const noexcept { return impl_->topic; }

}  // namespace lidar