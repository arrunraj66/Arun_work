// Round-trip: build a Frame by hand (with a small but real byte pattern,
// standing in for JPEG bytes -- this test doesn't need OpenCV to prove the
// codec preserves bytes exactly), serialise it, deserialise it back, assert
// every field survived unchanged. Same shape as test_imu_codec.cpp.

#include "camera/frame.hpp"
#include "camera_codec.hpp"

#include <cstdio>
#include <cstdlib>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-58s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

}  // namespace

int main() {
  camera::Frame original;
  original.frame_id = "front_camera";
  original.stamp_ns = 1'758'900'000'000'000'000LL;
  original.width = 640;
  original.height = 480;
  original.encoding = "jpeg";
  // Bytes 0x00..0xFF and back down -- deliberately includes 0x00 and bytes
  // with the high bit set, the two things most likely to get mangled if
  // this were handled as a C string anywhere instead of a byte buffer.
  original.data.reserve(512);
  for (int i = 0; i < 256; ++i) original.data.push_back(static_cast<std::uint8_t>(i));
  for (int i = 255; i >= 0; --i) original.data.push_back(static_cast<std::uint8_t>(i));

  camera::proto::Frame wire;
  camera::to_proto(original, wire);

  check(wire.frame_id() == original.frame_id, "wire frame_id matches");
  check(wire.stamp_ns() == original.stamp_ns, "wire stamp_ns matches");
  check(wire.width() == original.width, "wire width matches");
  check(wire.height() == original.height, "wire height matches");
  check(wire.encoding() == original.encoding, "wire encoding matches");
  check(wire.data().size() == original.data.size(), "wire data size matches");

  const std::string bytes = wire.SerializeAsString();
  camera::proto::Frame wire2;
  check(wire2.ParseFromString(bytes), "reparses from serialised bytes");

  const camera::Frame round_tripped = camera::from_proto(wire2);
  check(round_tripped.frame_id == original.frame_id, "round-trip frame_id matches");
  check(round_tripped.stamp_ns == original.stamp_ns, "round-trip stamp_ns matches");
  check(round_tripped.width == original.width, "round-trip width matches");
  check(round_tripped.height == original.height, "round-trip height matches");
  check(round_tripped.encoding == original.encoding, "round-trip encoding matches");
  check(round_tripped.data == original.data,
        "round-trip data matches byte-for-byte (incl. 0x00 and high-bit bytes)");

  if (failures > 0) {
    std::printf("%d check(s) FAILED\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("all checks passed\n");
  return EXIT_SUCCESS;
}