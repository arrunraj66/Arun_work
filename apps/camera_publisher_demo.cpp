// camera_publisher_demo -- the first step for the Logitech webcam, same
// shape as sick_scan_source_demo was for the lidar: prove the hardware and
// the network path work in isolation, standalone, before any of this gets
// wired into a library or a supervised service.
//
// What it does, every loop iteration:
//   1. grab one frame from the webcam (OpenCV's cv::VideoCapture)
//   2. JPEG-encode it in memory (cv::imencode) -- this is what keeps each
//      published message small; a raw 640x480 color frame is ~900 KB, the
//      same frame JPEG-encoded is usually 30-80 KB
//   3. publish the JPEG bytes over ZeroMQ (mw::Publisher, the exact same
//      transport class the lidar side already uses) on topic "camera.frame"
//
// No custom wire format yet -- the payload IS the JPEG bytes, nothing else.
// A timestamp/sequence header, and a proper ScanRecorder-style library
// wrapper, are the natural next steps once this is proven end to end, the
// same order the lidar side was built in.
//
// Ctrl+C handling copies scan_publisher_recorder_main.cpp exactly: a
// std::atomic<bool> flag set by our own SIGINT handler, checked once per
// loop iteration. Unlike the lidar sensor, cv::VideoCapture::open() and
// ::read() are local USB calls, not a network handshake a vendor SDK can
// block inside of -- so there is no equivalent of connect_with_stop_check()
// needed here; a plain flag check is enough.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include "mw/transport.hpp"

namespace {

std::atomic<bool> g_stop_requested{false};

void handle_sigint(int /*signal_number*/) { g_stop_requested.store(true); }

}  // namespace

int main(int argc, char** argv) {
  // Every setting has a sane default so `./camera_publisher_demo` with no
  // arguments just works on the first webcam at /dev/video0. Override
  // whichever ones you need: ./camera_publisher_demo <camera_index>
  // <endpoint> <fps> <jpeg_quality>
  const int camera_index         = argc > 1 ? std::atoi(argv[1]) : 0;
  const std::string endpoint     = argc > 2 ? argv[2] : "tcp://*:5562";
  const double target_fps        = argc > 3 ? std::atof(argv[3]) : 10.0;
  const int jpeg_quality         = argc > 4 ? std::atoi(argv[4]) : 80;  // 0-100

  std::signal(SIGINT, handle_sigint);

  std::printf("camera_publisher_demo: opening camera index %d ...\n", camera_index);
  cv::VideoCapture cap(camera_index);
  if (!cap.isOpened()) {
    std::fprintf(stderr,
                  "camera_publisher_demo: could not open camera index %d "
                  "(is it plugged in? try `ls /dev/video*` and `v4l2-ctl --list-devices`)\n",
                  camera_index);
    return 1;
  }
  std::printf("camera_publisher_demo: camera open, frame size %dx%d\n",
              static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
              static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)));

  mw::Context ctx;
  mw::Publisher publisher(ctx, endpoint);
  std::printf("camera_publisher_demo: publishing topic 'camera.frame' on %s\n", endpoint.c_str());

  const auto frame_interval =
      std::chrono::duration<double>(target_fps > 0.0 ? 1.0 / target_fps : 0.0);
  const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};

  std::uint64_t published_count = 0;
  std::uint64_t empty_frame_count = 0;
  const auto run_start = std::chrono::steady_clock::now();

  cv::Mat frame;
  std::vector<uchar> jpeg_bytes;

  while (!g_stop_requested.load()) {
    const auto loop_start = std::chrono::steady_clock::now();

    if (!cap.read(frame) || frame.empty()) {
      // A dropped/empty frame is normal occasionally with a USB webcam --
      // count it and keep going, the same "note it, don't crash" approach
      // as the lidar side's dropped-sample counters.
      ++empty_frame_count;
    } else {
      cv::imencode(".jpg", frame, jpeg_bytes, encode_params);
      const std::string payload(reinterpret_cast<const char*>(jpeg_bytes.data()),
                                 jpeg_bytes.size());
      if (publisher.publish("camera.frame", payload)) {
        ++published_count;
      }

      if (published_count % 30 == 0 && published_count > 0) {
        std::printf(
            "camera_publisher_demo: %llu frames published, %llu empty, %llu dropped\n",
            static_cast<unsigned long long>(published_count),
            static_cast<unsigned long long>(empty_frame_count),
            static_cast<unsigned long long>(publisher.dropped()));
      }
    }

    if (frame_interval.count() > 0.0) {
      const auto elapsed = std::chrono::steady_clock::now() - loop_start;
      const auto remaining =
          std::chrono::duration_cast<std::chrono::milliseconds>(frame_interval) -
          std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
      if (remaining.count() > 0) {
        std::this_thread::sleep_for(remaining);
      }
    }
  }

  const auto run_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - run_start).count();
  std::printf(
      "\ncamera_publisher_demo: stopping -- %llu published, %llu empty, %llu dropped, "
      "%.1fs\n",
      static_cast<unsigned long long>(published_count),
      static_cast<unsigned long long>(empty_frame_count),
      static_cast<unsigned long long>(publisher.dropped()), run_seconds);
  return 0;
}