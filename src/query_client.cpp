#include "lidar/query_client.hpp"

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mw/transport.hpp"
#include "query_wire.hpp"

namespace lidar {

struct ScanQueryClient::Impl {
  mw::Context   ctx;      // declared before requester: must outlive the socket
  mw::Requester requester;
  int           timeout_ms;
  bool          last_ok = false;

  Impl(std::string endpoint, int timeout)
      : requester(ctx, endpoint), timeout_ms(timeout) {}
};

ScanQueryClient::ScanQueryClient(std::string endpoint, int timeout_ms)
    : impl_(std::make_unique<Impl>(std::move(endpoint), timeout_ms)) {}

ScanQueryClient::~ScanQueryClient()                                       = default;
ScanQueryClient::ScanQueryClient(ScanQueryClient&&) noexcept              = default;
ScanQueryClient& ScanQueryClient::operator=(ScanQueryClient&&) noexcept   = default;

std::optional<Scan> ScanQueryClient::latest() noexcept {
  try {
    const std::optional<std::string> reply = impl_->requester.request("LATEST", impl_->timeout_ms);
    if (!reply.has_value()) {
      impl_->last_ok = false;
      return std::nullopt;
    }
    impl_->last_ok = true;

    const std::vector<Scan> scans = decode_scans(*reply);
    if (scans.empty()) return std::nullopt;   // reached the server; it just has nothing yet
    return scans.front();
  } catch (...) {
    // decode_scans() throwing means the reply was malformed -- a protocol
    // mismatch, not a timeout. Reported the same way as unreachable: the
    // caller could not get a trustworthy answer either way.
    impl_->last_ok = false;
    return std::nullopt;
  }
}

std::vector<Scan> ScanQueryClient::range(std::int64_t start_ns, std::int64_t end_ns) noexcept {
  try {
    const std::string request =
        "RANGE " + std::to_string(start_ns) + " " + std::to_string(end_ns);
    const std::optional<std::string> reply = impl_->requester.request(request, impl_->timeout_ms);
    if (!reply.has_value()) {
      impl_->last_ok = false;
      return {};
    }
    impl_->last_ok = true;
    return decode_scans(*reply);
  } catch (...) {
    impl_->last_ok = false;
    return {};
  }
}
std::vector<Scan> ScanQueryClient::recent(std::int64_t window_ns) noexcept {
  try {
    const std::string request = "RECENT " + std::to_string(window_ns);
    const std::optional<std::string> reply = impl_->requester.request(request, impl_->timeout_ms);
    if (!reply.has_value()) {
      impl_->last_ok = false;
      return {};
    }
    impl_->last_ok = true;
    return decode_scans(*reply);
  } catch (...) {
    impl_->last_ok = false;
    return {};
  }
}
bool ScanQueryClient::ok() const noexcept { return impl_->last_ok; }

}  // namespace lidar