#include "query_wire.hpp"

#include <stdexcept>

#include "lidar_scan.pb.h"
#include "scan_codec.hpp"

namespace lidar {
namespace {

void put_u32_le(std::string& out, std::uint32_t v) {
  out.push_back(static_cast<char>(v & 0xFFU));
  out.push_back(static_cast<char>((v >> 8) & 0xFFU));
  out.push_back(static_cast<char>((v >> 16) & 0xFFU));
  out.push_back(static_cast<char>((v >> 24) & 0xFFU));
}

std::uint32_t get_u32_le(const std::string& in, std::size_t at) {
  return static_cast<std::uint32_t>(static_cast<unsigned char>(in[at])) |
         (static_cast<std::uint32_t>(static_cast<unsigned char>(in[at + 1])) << 8) |
         (static_cast<std::uint32_t>(static_cast<unsigned char>(in[at + 2])) << 16) |
         (static_cast<std::uint32_t>(static_cast<unsigned char>(in[at + 3])) << 24);
}

}  // namespace

std::string encode_scans(const std::vector<Scan>& scans) {
  std::string out;
  put_u32_le(out, static_cast<std::uint32_t>(scans.size()));

  proto::LidarScan wire;   // reused across the loop, same reasoning as
                           // ScanPublisher/ScanSubscriber's reused `wire`
  for (const Scan& scan : scans) {
    to_proto(scan, wire);
    const std::string bytes = wire.SerializeAsString();
    put_u32_le(out, static_cast<std::uint32_t>(bytes.size()));
    out += bytes;
  }
  return out;
}

std::vector<Scan> decode_scans(const std::string& bytes) {
  if (bytes.size() < 4) {
    throw std::runtime_error("lidar::decode_scans: response shorter than a count prefix");
  }

  std::vector<Scan> scans;
  const std::uint32_t count = get_u32_le(bytes, 0);
  scans.reserve(count);

  std::size_t pos = 4;
  proto::LidarScan wire;
  for (std::uint32_t i = 0; i < count; ++i) {
    if (pos + 4 > bytes.size()) {
      throw std::runtime_error("lidar::decode_scans: truncated before a length prefix");
    }
    const std::uint32_t length = get_u32_le(bytes, pos);
    pos += 4;

    if (pos + length > bytes.size()) {
      throw std::runtime_error("lidar::decode_scans: truncated before a scan's declared end");
    }
    if (!wire.ParseFromArray(bytes.data() + pos, static_cast<int>(length))) {
      throw std::runtime_error("lidar::decode_scans: a scan's bytes did not parse as protobuf");
    }
    pos += length;

    scans.push_back(from_proto(wire));
  }
  return scans;
}

}  // namespace lidar