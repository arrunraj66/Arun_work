#!/usr/bin/env python3
"""cloud_subscriber_viewer.py -- Step 7: see the 3D point cloud, the way
camera_subscriber.py let you see the webcam feed.

Same shape as camera_subscriber.py: a plain pyzmq SUB socket, a receive
loop with a timeout so Ctrl+C always works, print status to the terminal,
render a window. The one real difference is what arrives on the wire: not
JPEG bytes, but a serialised LidarPointCloud protobuf message -- so this
also needs the generated Python protobuf module for lidar_point_cloud.proto,
which is NOT checked into the repo (same "generated files are build output,
never source" rule the C++ side follows -- see proto/CMakeLists.txt).

One-time setup, from the repo root:
    mkdir -p scripts/generated
    protoc --python_out=scripts/generated --proto_path=proto proto/lidar_point_cloud.proto
    pip install --break-system-packages pyzmq numpy matplotlib   # if not already present

Usage:
    python3 scripts/cloud_subscriber_viewer.py [endpoint] [topic] [--max-points N]

Defaults: tcp://127.0.0.1:5580, lidar.cloud, --max-points 3000

--max-points matters more than it looks: a real frame is ~9300 points, and
matplotlib's 3D scatter is a software renderer -- redrawing all 9300 points
every ~50ms would fall behind the sensor immediately. Downsampling (every
Nth point, chosen at random each frame so the pattern doesn't alias) keeps
the picture responsive; it is a VIEWER for verification, not a full-fidelity
recorder -- recording the untouched cloud is Step 8's job, not this
script's.
"""
import argparse
import os
import sys

import numpy as np
import zmq

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "generated"))
try:
    import lidar_point_cloud_pb2 as cloud_pb
except ImportError:
    sys.stderr.write(
        "cloud_subscriber_viewer: can't import lidar_point_cloud_pb2.\n"
        "  Generate it first (one-time, from the repo root):\n"
        "    mkdir -p scripts/generated\n"
        "    protoc --python_out=scripts/generated --proto_path=proto "
        "proto/lidar_point_cloud.proto\n"
    )
    sys.exit(1)

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt  # noqa: E402  (must follow matplotlib.use())


def to_xyz(ranges, azimuths, elevations):
    """Vectorised version of lidar::to_xyz() in point_cloud.hpp -- MUST stay
    in exact sync with that function's formula. If one changes, so does the
    other, or this viewer silently shows a different picture than the C++
    side computes."""
    horizontal = ranges * np.cos(elevations)
    x = horizontal * np.cos(azimuths)
    y = horizontal * np.sin(azimuths)
    z = ranges * np.sin(elevations)
    return x, y, z


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("endpoint", nargs="?", default="tcp://127.0.0.1:5580")
    parser.add_argument("topic", nargs="?", default="lidar.cloud")
    parser.add_argument(
        "--max-points", type=int, default=3000, help="downsample to at most this many points/frame"
    )
    args = parser.parse_args()

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(args.endpoint)
    sub.setsockopt_string(zmq.SUBSCRIBE, args.topic)
    sub.setsockopt(zmq.RCVTIMEO, 1000)  # ms -- same pattern as camera_subscriber.py

    print(f"cloud_subscriber_viewer: connecting to {args.endpoint}, topic '{args.topic}' "
          f"-- window opens on first frame")
    print("cloud_subscriber_viewer: close the window, or Ctrl+C here, to stop")

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")
    scatter = ax.scatter([], [], [], s=1)
    ax.set_xlabel("x (m, forward)")
    ax.set_ylabel("y (m, left)")
    ax.set_zlabel("z (m, up)")
    title = ax.set_title("waiting for first frame ...")
    plt.ion()
    plt.show()

    msg = cloud_pb.LidarPointCloud()
    frames_received = 0

    try:
        while plt.fignum_exists(fig.number):
            try:
                topic_bytes, payload = sub.recv_multipart(flags=0)
            except zmq.Again:
                continue  # ordinary timeout -- gives the window a chance to redraw/close
            except ValueError:
                # A publisher not sending exactly [topic, payload] would land
                # here; treated as one bad frame, not a reason to crash.
                continue

            if not msg.ParseFromString(payload):
                continue  # corrupt frame -- skip it, don't crash the viewer over one bad sample

            frames_received += 1

            ranges = np.asarray(msg.ranges, dtype=np.float32)
            azimuths = np.asarray(msg.azimuths, dtype=np.float32)
            elevations = np.asarray(msg.elevations, dtype=np.float32)

            # Same is_valid() rule as point_cloud.hpp: strictly inside
            # (range_min, range_max). A sensor with nothing to see does not
            # get to masquerade as a real point at 0.0 m.
            valid = (ranges > msg.range_min) & (ranges < msg.range_max)
            ranges, azimuths, elevations = ranges[valid], azimuths[valid], elevations[valid]

            if ranges.size > args.max_points:
                idx = np.random.choice(ranges.size, args.max_points, replace=False)
                ranges, azimuths, elevations = ranges[idx], azimuths[idx], elevations[idx]

            x, y, z = to_xyz(ranges, azimuths, elevations)

            scatter._offsets3d = (x, y, z)
            # Colour by height -- the fastest visual read on whether a point
            # is floor-level, sensor-level, or overhead, which is exactly
            # the question the elevation-sign check cares about.
            #
            # set_clim() every frame, explicitly, rather than relying on
            # set_array()'s own autoscale: whether a Collection re-norms its
            # colour range automatically on every set_array() call varies by
            # matplotlib version, and a stale colour range would silently
            # make every frame look the same shade regardless of height.
            if z.size:
                scatter.set_array(z)
                scatter.set_cmap("viridis")
                scatter.set_clim(float(z.min()), float(z.max()))

            title.set_text(
                f"{msg.frame_id}  frame #{frames_received}  "
                f"{ranges.size}/{len(msg.ranges)} points shown  "
                f"z range [{z.min():.2f}, {z.max():.2f}] m" if ranges.size else
                f"{msg.frame_id}  frame #{frames_received}  0 valid points"
            )

            # Autoscale the axes to the current frame's extent -- a fixed
            # range would either clip a big room or waste most of the plot
            # on empty space for a small one.
            if ranges.size:
                max_extent = float(np.max(np.abs(np.concatenate([x, y, z])))) or 1.0
                ax.set_xlim(-max_extent, max_extent)
                ax.set_ylim(-max_extent, max_extent)
                ax.set_zlim(-max_extent, max_extent)

            fig.canvas.draw_idle()
            fig.canvas.flush_events()

    except KeyboardInterrupt:
        pass
    finally:
        print(f"\ncloud_subscriber_viewer: stopping -- {frames_received} frames received")
        sub.close()
        ctx.term()


if __name__ == "__main__":
    main()
