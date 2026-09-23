#include "sick_cloud_source.hpp"

#include <sick_scan_xd_api/sick_scan_api.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>

namespace lidar {

namespace {

int find_field_offset(const SickScanPointFieldArray& fields, const char* name) {
  for (std::uint64_t i = 0; i < fields.size; ++i) {
    if (std::strcmp(fields.buffer[i].name, name) == 0) {
      return static_cast<int>(fields.buffer[i].offset);
    }
  }
  return -1;
}

int find_intensity_offset(const SickScanPointFieldArray& fields) {
  const int offset = find_field_offset(fields, "i");
  return offset >= 0 ? offset : find_field_offset(fields, "intensity");
}

template <typename T>
T read_field(const std::uint8_t* point, int offset) {
  T value{};
  std::memcpy(&value, point + offset, sizeof(T));
  return value;
}

constexpr std::int64_t kMaxPlausibleScanTimeNs = 5'000'000'000LL;  // 5 seconds

}  // namespace

struct SickCloudSource::Impl {
  static std::map<SickScanApiHandle, Impl*>& registry() {
    static std::map<SickScanApiHandle, Impl*> instances;
    return instances;
  }
  static std::mutex& registry_mutex() {
    static std::mutex m;
    return m;
  }

  explicit Impl(Config c) : cfg(std::move(c)) {
    if (cfg.hostname.empty()) {
      throw std::runtime_error("SickCloudSource: Config::hostname is empty");
    }
    if (cfg.udp_receiver_ip.empty()) {
      throw std::runtime_error(
          "SickCloudSource: Config::udp_receiver_ip is empty -- it must be THIS machine's "
          "IP address on the sensor's subnet (find it with: ip -4 addr show). The sensor "
          "pushes scan data to that address over UDP; without it the start handshake fails.");
    }
    if (cfg.launch_file.empty()) {
      throw std::runtime_error(
          "SickCloudSource: Config::launch_file is empty -- it must be the path to "
          "sick_scan_xd's launch/sick_multiscan.launch (or sick_picoscan.launch)");
    }
    if (cfg.queue_capacity == 0) {
      throw std::runtime_error("SickCloudSource: Config::queue_capacity must be at least 1");
    }

    handle = SickScanApiCreate(0, nullptr);
    if (handle == nullptr) {
      throw std::runtime_error("SickCloudSource: SickScanApiCreate returned null");
    }

    {
      const std::lock_guard<std::mutex> lock(registry_mutex());
      registry()[handle] = this;
    }

    if (SickScanApiRegisterPolarPointCloudMsg(handle, &Impl::on_polar_cloud) !=
        SICK_SCAN_API_SUCCESS) {
      unregister_self();
      SickScanApiRelease(handle);
      handle = nullptr;
      throw std::runtime_error("SickCloudSource: SickScanApiRegisterPolarPointCloudMsg failed");
    }
    callback_registered = true;

    std::string launch_args = cfg.launch_file +                             //
                              " hostname:=" + cfg.hostname +                //
                              " udp_receiver_ip:=" + cfg.udp_receiver_ip +  //
                              " custom_pointclouds:=" + cfg.cloud_name;
    // Only appended when non-default -- omitting them entirely when 0
    // leaves the launch file's own port free to keep working exactly as
    // it always has for a single-sensor setup.
    if (cfg.udp_port != 0) {
      launch_args += " udp_port:=" + std::to_string(cfg.udp_port);
    }
    if (cfg.imu_udp_port != 0) {
      launch_args += " imu_udp_port:=" + std::to_string(cfg.imu_udp_port);
    }

    const std::int32_t rc = SickScanApiInitByLaunchfile(handle, launch_args.c_str());
    if (rc != SICK_SCAN_API_SUCCESS) {
      SickScanApiDeregisterPolarPointCloudMsg(handle, &Impl::on_polar_cloud);
      callback_registered = false;
      unregister_self();
      SickScanApiRelease(handle);
      handle = nullptr;
      throw std::runtime_error("SickCloudSource: SickScanApiInitByLaunchfile failed (rc=" +
                               std::to_string(rc) + ") for args: " + launch_args);
    }
  }

  ~Impl() {
    if (handle != nullptr) {
      if (callback_registered) {
        SickScanApiDeregisterPolarPointCloudMsg(handle, &Impl::on_polar_cloud);
      }
      unregister_self();
      SickScanApiClose(handle);
      SickScanApiRelease(handle);
    }
  }

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  void unregister_self() {
    const std::lock_guard<std::mutex> lock(registry_mutex());
    registry().erase(handle);
  }

  static void on_polar_cloud(SickScanApiHandle api_handle, const SickScanPointCloudMsg* msg) {
    if (msg == nullptr) return;
    Impl* self = nullptr;
    {
      const std::lock_guard<std::mutex> lock(registry_mutex());
      const auto it = registry().find(api_handle);
      if (it == registry().end()) return;
      self = it->second;
    }
    self->enqueue(*msg);
  }

  void enqueue(const SickScanPointCloudMsg& msg) {
    PointCloud cloud;
    if (!to_cloud(msg, cloud)) return;

    const std::lock_guard<std::mutex> lock(queue_mutex);
    while (queue.size() >= cfg.queue_capacity) {
      queue.pop_front();
      ++dropped;
    }
    queue.push_back(std::move(cloud));
    queue_cv.notify_one();
  }

  bool to_cloud(const SickScanPointCloudMsg& msg, PointCloud& out) {
    const int offset_range = find_field_offset(msg.fields, "range");
    const int offset_azimuth = find_field_offset(msg.fields, "azimuth");
    const int offset_elevation = find_field_offset(msg.fields, "elevation");
    const int offset_intensity = find_intensity_offset(msg.fields);
    const int offset_echo = find_field_offset(msg.fields, "echo");

    if (offset_range < 0 || offset_azimuth < 0 || offset_elevation < 0) return false;
    if (msg.data.buffer == nullptr || msg.point_step == 0) return false;

    const std::uint64_t num_points =
        static_cast<std::uint64_t>(msg.width) * static_cast<std::uint64_t>(msg.height);
    if (num_points * msg.point_step > msg.data.size) return false;

    out.ranges.clear();
    out.azimuths.clear();
    out.elevations.clear();
    out.intensities.clear();
    out.ranges.reserve(num_points);
    out.azimuths.reserve(num_points);
    out.elevations.reserve(num_points);
    if (offset_intensity >= 0) out.intensities.reserve(num_points);

    for (std::uint64_t i = 0; i < num_points; ++i) {
      const std::uint8_t* point = msg.data.buffer + i * msg.point_step;

      if (offset_echo >= 0 && read_field<std::int8_t>(point, offset_echo) != 0) {
        continue;
      }

      out.ranges.push_back(read_field<float>(point, offset_range));
      out.azimuths.push_back(read_field<float>(point, offset_azimuth));
      out.elevations.push_back(read_field<float>(point, offset_elevation));
      if (offset_intensity >= 0) {
        out.intensities.push_back(read_field<float>(point, offset_intensity));
      }
    }

    const std::int64_t stamp_ns =
        static_cast<std::int64_t>(msg.header.timestamp_sec) * 1'000'000'000LL +
        static_cast<std::int64_t>(msg.header.timestamp_nsec);

    out.frame_id = cfg.frame_id;
    out.stamp_ns = stamp_ns;
    const std::int64_t previous = last_stamp_ns.exchange(stamp_ns);
    const std::int64_t delta_ns = stamp_ns - previous;
    out.scan_time_ns =
        (previous > 0 && delta_ns > 0 && delta_ns <= kMaxPlausibleScanTimeNs) ? delta_ns : 0;

    out.range_min = cfg.range_min;
    out.range_max = cfg.range_max;
    return true;
  }

  Config cfg;
  SickScanApiHandle handle = nullptr;
  bool callback_registered = false;

  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::deque<PointCloud> queue;
  std::uint64_t dropped = 0;

  std::atomic<std::int64_t> last_stamp_ns{0};
};

SickCloudSource::SickCloudSource(Config cfg) : impl_(std::make_unique<Impl>(std::move(cfg))) {}
SickCloudSource::~SickCloudSource() = default;
SickCloudSource::SickCloudSource(SickCloudSource&&) noexcept = default;
SickCloudSource& SickCloudSource::operator=(SickCloudSource&&) noexcept = default;

bool SickCloudSource::poll_cloud(PointCloud& out, int timeout_ms) noexcept {
  try {
    std::unique_lock<std::mutex> lock(impl_->queue_mutex);
    if (!impl_->queue_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                  [this] { return !impl_->queue.empty(); })) {
      return false;
    }
    out = std::move(impl_->queue.front());
    impl_->queue.pop_front();
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace lidar