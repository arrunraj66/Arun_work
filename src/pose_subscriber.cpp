#include "nav/pose_subscriber.hpp"

#include <optional>
#include <string>
#include <utility>

#include "mw/transport.hpp"
#include "waypoint_codec.hpp"

namespace nav {

struct PoseSubscriber::Impl {
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

  proto::VehiclePose wire;
};

PoseSubscriber::PoseSubscriber(std::string endpoint, std::string topic)
    : impl_(std::make_unique<Impl>(std::move(endpoint), std::move(topic))) {}

PoseSubscriber::~PoseSubscriber() = default;
PoseSubscriber::PoseSubscriber(PoseSubscriber&&) noexcept = default;
PoseSubscriber& PoseSubscriber::operator=(PoseSubscriber&&) noexcept = default;

bool PoseSubscriber::poll_pose(VehiclePose& out, int timeout_ms) noexcept {
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

std::uint64_t PoseSubscriber::malformed() const noexcept { return impl_->malformed; }

const std::string& PoseSubscriber::topic() const noexcept { return impl_->topic; }

}  // namespace nav
