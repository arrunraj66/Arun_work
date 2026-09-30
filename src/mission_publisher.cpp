#include "nav/mission_publisher.hpp"

#include <string>
#include <utility>

#include "mw/transport.hpp"
#include "waypoint_codec.hpp"

namespace nav {

struct MissionPublisher::Impl {
  Impl(std::string ep, std::string t)
      : endpoint(std::move(ep)), topic(std::move(t)), publisher(ctx, endpoint) {}

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  std::string endpoint;
  std::string topic;

  mw::Context ctx;
  mw::Publisher publisher;

  proto::WaypointMission wire;
};

MissionPublisher::MissionPublisher(std::string endpoint, std::string topic)
    : impl_(std::make_unique<Impl>(std::move(endpoint), std::move(topic))) {}

MissionPublisher::~MissionPublisher() = default;
MissionPublisher::MissionPublisher(MissionPublisher&&) noexcept = default;
MissionPublisher& MissionPublisher::operator=(MissionPublisher&&) noexcept = default;

bool MissionPublisher::publish(const WaypointMission& mission) noexcept {
  try {
    to_proto(mission, impl_->wire);
    const std::string bytes = impl_->wire.SerializeAsString();
    return impl_->publisher.publish(impl_->topic, bytes);
  } catch (...) {
    return false;
  }
}

std::uint64_t MissionPublisher::dropped() const noexcept { return impl_->publisher.dropped(); }

const std::string& MissionPublisher::topic() const noexcept { return impl_->topic; }

}  // namespace nav
