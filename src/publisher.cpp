#include "lidar/publisher.hpp"

#include <string>
#include <utility>

#include "mw/transport.hpp"
#include "scan_codec.hpp"

namespace lidar {

struct ScanPublisher::Impl {
  Impl(std::string ep, std::string t)
      : endpoint(std::move(ep)), topic(std::move(t)), publisher(ctx, endpoint) {}

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  std::string endpoint;
  std::string topic;

  // One ZeroMQ context per publisher. The middleware's own rule is "one
  // context per process", and a process that already has an mw::Context
  // for its other traffic would ideally share it -- but taking an
  // mw::Context& here would put mw/transport.hpp into lidar/publisher.hpp,
  // and then every consumer of lidar::lidar would need the middleware's
  // headers to compile. Owning one costs a single background IO thread,
  // which is the cheaper of the two prices.
  //
  // Declaration order matters: ctx must outlive publisher, so it is
  // declared first and destroyed last.
  mw::Context ctx;
  mw::Publisher publisher;

  // Reused across calls rather than constructed per scan -- the same
  // reasoning as ScanRecorder::Impl::wire.
  proto::LidarScan wire;
};

ScanPublisher::ScanPublisher(std::string endpoint, std::string topic)
    : impl_(std::make_unique<Impl>(std::move(endpoint), std::move(topic))) {}

ScanPublisher::~ScanPublisher() = default;
ScanPublisher::ScanPublisher(ScanPublisher&&) noexcept = default;
ScanPublisher& ScanPublisher::operator=(ScanPublisher&&) noexcept = default;

bool ScanPublisher::publish(const Scan& scan) noexcept {
  try {
    to_proto(scan, impl_->wire);
    const std::string bytes = impl_->wire.SerializeAsString();
    return impl_->publisher.publish(impl_->topic, bytes);
  } catch (...) {
    // Declared noexcept, so a throw here would terminate the process.
    // Serialising allocates, and an allocation failure is not a reason to
    // kill a vehicle -- report it the same way a full send queue is
    // reported, as one lost scan.
    return false;
  }
}

std::uint64_t ScanPublisher::dropped() const noexcept { return impl_->publisher.dropped(); }

const std::string& ScanPublisher::topic() const noexcept { return impl_->topic; }

}  // namespace lidar
