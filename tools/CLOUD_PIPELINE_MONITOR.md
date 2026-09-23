# 3D LiDAR pipeline monitor

`cloud_pipeline_monitor.py` is the 3D equivalent of the earlier 2D scan
monitor. It remains brokerless: the monitor is simply another ZeroMQ
subscriber, so it can run at the same time as `cloud_gui`.

## Pipeline

```text
SICK multiScan136
  -> UDP scan packets on port 2125
  -> sick_scan_xd
  -> SickCloudSource
  -> lidar::PointCloud
  -> LidarPointCloud Protobuf
  -> ZeroMQ PUB tcp://*:5580 / lidar.cloud
       -> cloud_gui
       -> cloud_pipeline_monitor.py
            -> cloud_messages table
            -> cloud_samples table
```

There is no central broker. `cloud_publisher_main` binds port 5580 and each
subscriber independently connects to it.

## Install dependencies

```bash
sudo apt update
sudo apt install python3-zmq python3-protobuf protobuf-compiler sqlite3
```

The script automatically runs `protoc` when the generated Python file is
missing or older than `proto/lidar_point_cloud.proto`.

## Run

First start the existing 3D publisher:

```bash
./build/apps/cloud_publisher_main \
  ~/sick_scan_ws/sick_scan_xd/launch/sick_multiscan.launch \
  192.168.12.223 \
  192.168.12.240 \
  "tcp://*:5580" \
  lidar.cloud \
  2125 \
  7513
```

Then start the monitor from the repository root:

```bash
python3 tools/cloud_pipeline_monitor.py \
  --endpoint tcp://127.0.0.1:5580 \
  --topic lidar.cloud \
  --database cloud_metrics.sqlite3 \
  --sample-points 12
```

Use `tcp://192.168.12.240:5580` instead when the monitor runs on another
computer. `192.168.12.223` is the sensor, but the monitor connects to the
middleware publisher computer.

Run a fixed 60-second measurement with:

```bash
python3 tools/cloud_pipeline_monitor.py --duration 60
```

## What the timing fields mean

| Field | Meaning |
|---|---|
| `receive_wait_us` | Time waiting for the next ZeroMQ message. This mainly reflects cloud interval; it is not pure network latency. |
| `parse_us` | CPU time used to decode the Protobuf payload. |
| `database_us` | Time used to insert the summary, sample points, and commit SQLite. |
| `source_age_us` | Receive epoch minus sensor timestamp. It is shown only when the two clocks appear compatible. |
| `scan_time_ns` | Frame interval reported by the C++ `SickCloudSource`. |

The script prints measurements in microseconds. It also calculates traffic per
second, minute, and hour from the actual message bytes observed during each
reporting window.

## Why only sample points are stored

One cloud contains roughly 9,000 points and four float arrays. Saving every
point from every cloud would make SQLite grow very quickly and would add enough
database work to disturb the measurement itself.

The monitor therefore stores:

- one `cloud_messages` row for every cloud;
- 12 evenly distributed XYZ points by default in `cloud_samples`;
- no full duplicate cloud payload.

Change the number using `--sample-points`. Use `--sample-points 0` to store
only metadata.

## Inspect summary data

```bash
sqlite3 -header -column cloud_metrics.sqlite3 \
  "SELECT id, frame_id, point_count, message_bytes, parse_us, database_us
   FROM cloud_messages ORDER BY id DESC LIMIT 10;"
```

## Inspect sample XYZ points

```bash
sqlite3 -header -column cloud_metrics.sqlite3 \
  "SELECT message_id, point_index, x_m, y_m, z_m, intensity
   FROM cloud_samples ORDER BY message_id DESC, point_index LIMIT 20;"
```

## Calculate recorded duration and average rate

```bash
sqlite3 -header -column cloud_metrics.sqlite3 \
  "SELECT COUNT(*) AS clouds,
          ROUND((MAX(received_ns)-MIN(received_ns))/1e9, 2) AS seconds,
          ROUND(COUNT(*)/((MAX(received_ns)-MIN(received_ns))/1e9), 2) AS clouds_per_second
   FROM cloud_messages;"
```

## Calculate measured data volume

```bash
sqlite3 -header -column cloud_metrics.sqlite3 \
  "SELECT SUM(message_bytes) AS total_wire_bytes,
          ROUND(AVG(message_bytes), 1) AS average_message_bytes,
          ROUND(SUM(message_bytes)/1048576.0, 2) AS total_mib
   FROM cloud_messages;"
```

## Important clock warning

The sensor web page previously showed a 1970 system time. If the sensor stamp
and Ubuntu clock do not share the same epoch, a calculated end-to-end latency
would be meaningless. The script detects implausible differences and prints
`clock mismatch/unavailable` instead of displaying a false latency number.
