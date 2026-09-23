#include "lidar/cloud_subscriber.hpp"

#include <optional>
#include <string>
#include <utility>

#include "cloud_codec.hpp"
#include "mw/transport.hpp"

namespace lidar {

struct CloudSubscriber::Impl {
  Impl(std::string ep, std::string t)
      : endpoint(std::move(ep)), topic(std::move(t)), subscriber(ctx, endpoint) {
    subscriber.subscribe(topic);
  }

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  std::string endpoint;
  std::string topic;

  mw::Context ctx;
  mw::Subscriber subscriber;

  std::uint64_t malformed = 0;

  proto::LidarPointCloud wire;
};

CloudSubscriber::CloudSubscriber(std::string endpoint, std::string topic)
    : impl_(std::make_unique<Impl>(std::move(endpoint), std::move(topic))) {}

CloudSubscriber::~CloudSubscriber() = default;
CloudSubscriber::CloudSubscriber(CloudSubscriber&&) noexcept = default;
CloudSubscriber& CloudSubscriber::operator=(CloudSubscriber&&) noexcept = default;

bool CloudSubscriber::poll_cloud(PointCloud& out, int timeout_ms) noexcept {
  try {
    std::optional<mw::Message> msg = impl_->subscriber.receive(timeout_ms);
    if (!msg.has_value()) {
      return false;
    }

    if (!impl_->wire.ParseFromString(msg->payload)) {
      ++impl_->malformed;
      return false;
    }

    out = from_proto(impl_->wire);
    return true;
  } catch (...) {
    return false;
  }
}

std::uint64_t CloudSubscriber::malformed() const noexcept { return impl_->malformed; }

const std::string& CloudSubscriber::topic() const noexcept { return impl_->topic; }

}  // namespace lidar