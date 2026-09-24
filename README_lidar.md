# lidar

A small C++20 library for getting 2D LiDAR scans out of a SICK picoScan150 —
live off the sensor, over the network, or replayed from a recording — and for
recording them to disk.

It is built so that you need to know about exactly one type, `lidar::Scan`,
and one interface, `lidar::IScanSource`. Protobuf, SQLite, ZeroMQ and the
SICK vendor SDK are all used internally and none of them appear in any header
you include or any line of your own CMake.

## Building and installing

```bash
git clone <repo> auv_middleware
cd auv_middleware
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
cmake --install build --prefix ~/.local
```

The SICK SDK is optional. Without it everything in this document still builds
and works; only the code that talks directly to a physical sensor is skipped.

## Using it

```cmake
find_package(lidar 0.1 REQUIRED)
target_link_libraries(my_gui PRIVATE lidar::lidar)
```

If you installed somewhere CMake does not search by default, point it there:
`cmake -S . -B build -DCMAKE_PREFIX_PATH=$HOME/.local`.

That is the whole of the build integration. There is no `find_package(Protobuf)`,
no `find_package(SQLite3)`, no ZeroMQ. Those live inside `liblidar.so`. You do
need the protobuf and ZeroMQ *runtime* libraries present on the machine
(`libprotobuf32`, `libzmq5` on Ubuntu) — the dynamic loader pulls them in — but
nothing about them enters your build.

## The one type: `lidar::Scan`

```cpp
#include "lidar/scan.hpp"
```

A full revolution of measurements:

```cpp
struct Scan {
  std::string  frame_id;             // which sensor produced this
  std::int64_t stamp_ns;             // when the scan STARTED, ns since epoch
  std::int64_t scan_time_ns;         // how long one revolution took
  std::int64_t time_increment_ns;    // time between consecutive samples

  float angle_min, angle_max;        // first and last sample angle, radians
  float angle_increment;             // angular step, radians

  float range_min, range_max;        // sensor limits, metres

  std::uint8_t echo_index;           // which echo this is; 0 = nearest
  std::uint8_t echo_count;           // how many echoes were configured

  std::vector<float> ranges;         // one distance per step, metres
  std::vector<float> intensities;    // return strength per step; may be empty
};
```

Angles are not stored per sample — they are implied by the index, so use the
two free functions rather than computing them yourself:

```cpp
float a  = lidar::angle_of(scan, i);   // radians
bool  ok = lidar::is_valid(scan, i);   // false means "no return", not 0 m
```

**Always check `is_valid` before using `ranges[i]`.** A step where nothing was
detected still has a number in it; `is_valid` is what distinguishes a real
measurement from an absence of one. Converting a scan to points looks like:

```cpp
for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
  if (!lidar::is_valid(scan, i)) continue;
  const float a = lidar::angle_of(scan, i);
  const float x = scan.ranges[i] * std::cos(a);
  const float y = scan.ranges[i] * std::sin(a);
  // ... plot (x, y)
}
```

On a picoScan150 expect roughly 950–1000 samples per scan spanning about
±2.41 rad (±138°) at 25 Hz.

## The one interface: `lidar::IScanSource`

```cpp
#include "lidar/source.hpp"
```

```cpp
class IScanSource {
 public:
  virtual ~IScanSource() = default;
  virtual bool poll_scan(Scan& out, int timeout_ms) noexcept = 0;
};
```

You ask it for a scan; it does not call you back on a thread you do not own.
`true` means `out` was filled. `false` means the timeout elapsed with nothing
to report — that is the normal idle case, not an error, and it never throws.
On `false`, `out` is unspecified: do not read it.

Write your code against `IScanSource&` and it does not matter where the scans
came from.

## Getting a source

**Over the network** — the usual case for GUI code, and it needs no sensor and
no vendor SDK:

```cpp
#include "lidar/subscriber.hpp"

lidar::ScanSubscriber source("tcp://192.168.12.240:5556");
// or "ipc:///tmp/lidar.sock" when the publisher is on this same machine

lidar::Scan scan;
while (running) {
  if (!source.poll_scan(scan, 100)) continue;   // nothing yet, try again
  draw(scan);
}
```

Two things to know about the transport. First, it drops rather than blocks: if
your consumer falls behind, scans are discarded, and the display shows recent
data rather than lagging further and further behind reality. Second, anything
published before your subscriber has finished connecting is lost — this is how
PUB/SUB works and is not a bug. At 25 Hz it costs you the first frame or two.
`source.malformed()` counts messages that arrived but could not be parsed; it
should stay 0, and a rising number means a version mismatch between the two ends.

**From a recording**, for replaying a dive without the vehicle:

```cpp
#include "lidar/reader.hpp"

lidar::ScanReader reader("dive.log", "dive.db");
for (std::uint64_t i = 0; i < reader.count(); ++i) {
  draw(reader.get(i));
}
```

`reader.find_by_time_range(start_ns, end_ns)` returns just the scans in a time
window, using the SQLite index rather than scanning the log.

**From the sensor itself** — only for the process that owns the hardware. This
is the one part that needs the SICK SDK, and GUI code should not use it; run
`scan_publisher_main` instead and subscribe.

## Recording

```cpp
#include "lidar/recorder.hpp"

lidar::ScanRecorder rec("dive.log", "dive.db");
rec.record(scan);        // appends to the log and indexes it, both or neither
```

Two files: an append-only log holding the scans, and a SQLite index holding one
row each so a reader can jump straight to a scan without reading everything
before it. Both are opened for append — recording again later adds to what is
there. `record()` throws on I/O failure. The `dump_scans` tool prints the
contents of a pair from the command line.

Budget about 8 KB per scan with intensities included — measured at 7,963 bytes
for a 977-sample picoScan150 scan. That is roughly 195 KB/s at 25 Hz, or
0.7 GB per hour of continuous recording.

## Publishing

Only the process that owns the sensor needs this:

```cpp
#include "lidar/publisher.hpp"

lidar::ScanPublisher pub("tcp://*:5556");
if (!pub.publish(scan)) { /* queue full; scan dropped, see pub.dropped() */ }
```

`publish()` never blocks and never throws. A `false` return means a slow
subscriber caused the scan to be dropped, which for live sensor data is the
correct behaviour, not an error to stop for.

## The headers, in full

| Header | What it gives you |
| --- | --- |
| `lidar/scan.hpp` | `Scan`, `angle_of`, `is_valid` |
| `lidar/source.hpp` | `IScanSource` |
| `lidar/subscriber.hpp` | `ScanSubscriber` — scans over the network |
| `lidar/reader.hpp` | `ScanReader` — scans from a recording |
| `lidar/recorder.hpp` | `ScanRecorder` — scans to disk |
| `lidar/publisher.hpp` | `ScanPublisher` — scans onto the network |

## Threading

Nothing here is thread-safe across instances of the same object. Give each
thread its own `ScanSubscriber` / `ScanReader` / `ScanRecorder`, or hold a lock
around it. Separate objects in separate threads are fine.
