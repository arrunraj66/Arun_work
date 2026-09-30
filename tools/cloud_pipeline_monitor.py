#!/usr/bin/env python3
"""Monitor the AUV 3D LiDAR PUB/SUB pipeline and record metrics in SQLite.

The publisher sends two ZeroMQ frames: [topic, protobuf payload].  This tool
subscribes without introducing a broker, decodes LidarPointCloud, measures each
stage, stores compact per-cloud metadata, and stores a small XYZ sample instead
of duplicating every point from every cloud.
"""

from __future__ import annotations

import argparse
import importlib
import math
import os
from pathlib import Path
import signal
import sqlite3
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Any

try:
    import zmq
except ImportError as error:
    raise SystemExit(
        "pyzmq is missing. Install it with: sudo apt install python3-zmq"
    ) from error


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
PROTO_FILE = PROJECT_ROOT / "proto" / "lidar_point_cloud.proto"
GENERATED_DIR = SCRIPT_DIR / "generated"
GENERATED_MODULE = GENERATED_DIR / "lidar_point_cloud_pb2.py"


def load_protobuf_module() -> Any:
    """Generate and import the Python class matching the C++ wire schema."""
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    needs_generation = (
        not GENERATED_MODULE.exists()
        or GENERATED_MODULE.stat().st_mtime < PROTO_FILE.stat().st_mtime
    )
    if needs_generation:
        command = [
            "protoc",
            f"--proto_path={PROTO_FILE.parent}",
            f"--python_out={GENERATED_DIR}",
            str(PROTO_FILE),
        ]
        try:
            subprocess.run(command, check=True)
        except FileNotFoundError as error:
            raise SystemExit(
                "protoc is missing. Install it with: "
                "sudo apt install protobuf-compiler python3-protobuf"
            ) from error
        except subprocess.CalledProcessError as error:
            raise SystemExit(f"protoc failed with status {error.returncode}") from error

    sys.path.insert(0, str(GENERATED_DIR))
    return importlib.import_module("lidar_point_cloud_pb2")


def create_database(path: Path) -> sqlite3.Connection:
    """Open SQLite and create the two tables used by this monitor."""
    connection = sqlite3.connect(path)
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("PRAGMA synchronous=NORMAL")
    connection.executescript(
        """
        CREATE TABLE IF NOT EXISTS cloud_messages (
            id               INTEGER PRIMARY KEY AUTOINCREMENT,
            received_ns      INTEGER NOT NULL,
            sensor_stamp_ns  INTEGER NOT NULL,
            frame_id         TEXT NOT NULL,
            topic            TEXT NOT NULL,
            point_count      INTEGER NOT NULL,
            message_bytes    INTEGER NOT NULL,
            receive_wait_us  REAL NOT NULL,
            parse_us         REAL NOT NULL,
            database_us      REAL NOT NULL,
            source_age_us    REAL,
            scan_time_ns     INTEGER NOT NULL,
            range_min_m      REAL NOT NULL,
            range_max_m      REAL NOT NULL
        );

        CREATE INDEX IF NOT EXISTS idx_cloud_messages_received
        ON cloud_messages(received_ns);

        CREATE TABLE IF NOT EXISTS cloud_samples (
            message_id       INTEGER NOT NULL,
            point_index      INTEGER NOT NULL,
            x_m              REAL NOT NULL,
            y_m              REAL NOT NULL,
            z_m              REAL NOT NULL,
            range_m          REAL NOT NULL,
            azimuth_rad      REAL NOT NULL,
            elevation_rad    REAL NOT NULL,
            intensity        REAL,
            FOREIGN KEY(message_id) REFERENCES cloud_messages(id)
        );

        CREATE INDEX IF NOT EXISTS idx_cloud_samples_message
        ON cloud_samples(message_id);
        """
    )
    connection.commit()
    return connection


def xyz(range_m: float, azimuth: float, elevation: float) -> tuple[float, float, float]:
    """Use the same x-forward, y-left, z-up conversion as point_cloud.hpp."""
    horizontal = range_m * math.cos(elevation)
    return (
        horizontal * math.cos(azimuth),
        horizontal * math.sin(azimuth),
        range_m * math.sin(elevation),
    )


def choose_sample_indices(point_count: int, requested: int) -> list[int]:
    """Spread sample rows through the cloud instead of taking one local patch."""
    sample_count = min(point_count, max(0, requested))
    if sample_count == 0:
        return []
    if sample_count == 1:
        return [point_count // 2]
    return [
        round(index * (point_count - 1) / (sample_count - 1))
        for index in range(sample_count)
    ]


def plausible_source_age_us(received_ns: int, sensor_stamp_ns: int) -> float | None:
    """Return end-to-end age only when both clocks appear to share epoch time."""
    age_ns = received_ns - sensor_stamp_ns
    if 0 <= age_ns <= 60_000_000_000:
        return age_ns / 1_000.0
    return None


def store_cloud(
    connection: sqlite3.Connection,
    cloud: Any,
    topic: str,
    received_ns: int,
    message_bytes: int,
    receive_wait_us: float,
    parse_us: float,
    source_age_us: float | None,
    sample_points: int,
) -> tuple[int, float]:
    """Insert one cloud summary and selected XYZ points; return id and time."""
    database_start_ns = time.perf_counter_ns()
    cursor = connection.execute(
        """
        INSERT INTO cloud_messages(
            received_ns, sensor_stamp_ns, frame_id, topic, point_count,
            message_bytes, receive_wait_us, parse_us, database_us,
            source_age_us, scan_time_ns, range_min_m, range_max_m
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, 0.0, ?, ?, ?, ?)
        """,
        (
            received_ns,
            cloud.stamp_ns,
            cloud.frame_id,
            topic,
            len(cloud.ranges),
            message_bytes,
            receive_wait_us,
            parse_us,
            source_age_us,
            cloud.scan_time_ns,
            cloud.range_min,
            cloud.range_max,
        ),
    )
    message_id = int(cursor.lastrowid)

    common_count = min(len(cloud.ranges), len(cloud.azimuths), len(cloud.elevations))
    sample_rows = []
    for point_index in choose_sample_indices(common_count, sample_points):
        range_m = float(cloud.ranges[point_index])
        azimuth = float(cloud.azimuths[point_index])
        elevation = float(cloud.elevations[point_index])
        point_x, point_y, point_z = xyz(range_m, azimuth, elevation)
        intensity = (
            float(cloud.intensities[point_index])
            if point_index < len(cloud.intensities)
            else None
        )
        sample_rows.append(
            (
                message_id,
                point_index,
                point_x,
                point_y,
                point_z,
                range_m,
                azimuth,
                elevation,
                intensity,
            )
        )

    connection.executemany(
        """
        INSERT INTO cloud_samples(
            message_id, point_index, x_m, y_m, z_m, range_m,
            azimuth_rad, elevation_rad, intensity
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        sample_rows,
    )
    connection.commit()
    database_us = (time.perf_counter_ns() - database_start_ns) / 1_000.0
    connection.execute(
        "UPDATE cloud_messages SET database_us = ? WHERE id = ?",
        (database_us, message_id),
    )
    connection.commit()
    return message_id, database_us


def database_size(path: Path) -> int:
    """Include SQLite's WAL and shared-memory side files when present."""
    return sum(
        candidate.stat().st_size
        for candidate in (path, Path(f"{path}-wal"), Path(f"{path}-shm"))
        if candidate.exists()
    )


def human_bytes(value: float) -> str:
    """Format byte counts using binary units."""
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if value < 1024.0 or unit == "TiB":
            return f"{value:.2f} {unit}"
        value /= 1024.0
    return f"{value:.2f} TiB"


@dataclass
class WindowStatistics:
    """Measurements accumulated between once-per-second terminal reports."""

    messages: int = 0
    bytes: int = 0
    points: int = 0
    wait_us: list[float] = field(default_factory=list)
    parse_us: list[float] = field(default_factory=list)
    database_us: list[float] = field(default_factory=list)
    source_age_us: list[float] = field(default_factory=list)

    def reset(self) -> None:
        self.messages = 0
        self.bytes = 0
        self.points = 0
        self.wait_us.clear()
        self.parse_us.clear()
        self.database_us.clear()
        self.source_age_us.clear()


def mean_or_zero(values: list[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def print_flow() -> None:
    print(
        "\n3D LiDAR data flow\n"
        "  multiScan136 (UDP 2125)\n"
        "       -> sick_scan_xd\n"
        "       -> SickCloudSource / lidar::PointCloud\n"
        "       -> Protobuf LidarPointCloud\n"
        "       -> ZeroMQ PUB tcp://*:5580, topic lidar.cloud\n"
        "       -> this monitor SUB\n"
        "       -> timing + size metrics\n"
        "       -> SQLite cloud_messages + cloud_samples\n"
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5580")
    parser.add_argument("--topic", default="lidar.cloud")
    parser.add_argument("--database", type=Path, default=Path("cloud_metrics.sqlite3"))
    parser.add_argument(
        "--sample-points",
        type=int,
        default=12,
        help="XYZ sample rows saved per cloud; 0 disables samples",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after this many seconds; 0 means run until Ctrl+C",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    protobuf_module = load_protobuf_module()
    connection = create_database(arguments.database)

    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.RCVHWM, 1000)
    subscriber.setsockopt(zmq.LINGER, 0)
    subscriber.setsockopt_string(zmq.SUBSCRIBE, arguments.topic)
    subscriber.connect(arguments.endpoint)

    running = True

    def request_stop(_signal_number: int, _frame: Any) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    print_flow()
    print(f"Endpoint : {arguments.endpoint}")
    print(f"Topic    : {arguments.topic}")
    print(f"Database : {arguments.database.resolve()}")
    print(f"Samples  : {arguments.sample_points} points/cloud")
    print("Waiting for clouds; start the publisher first. Press Ctrl+C to stop.\n")

    poller = zmq.Poller()
    poller.register(subscriber, zmq.POLLIN)
    started = time.monotonic()
    last_report = started
    previous_database_size = database_size(arguments.database)
    total_messages = 0
    malformed = 0
    window = WindowStatistics()

    try:
        while running:
            if arguments.duration > 0.0 and time.monotonic() - started >= arguments.duration:
                break

            wait_start_ns = time.perf_counter_ns()
            events = dict(poller.poll(timeout=250))
            if subscriber not in events:
                continue

            frames = subscriber.recv_multipart()
            received_ns = time.time_ns()
            receive_wait_us = (time.perf_counter_ns() - wait_start_ns) / 1_000.0
            if len(frames) != 2:
                malformed += 1
                continue

            topic_bytes, payload = frames
            topic = topic_bytes.decode("utf-8", errors="replace")
            parse_start_ns = time.perf_counter_ns()
            cloud = protobuf_module.LidarPointCloud()
            try:
                cloud.ParseFromString(payload)
            except Exception:
                malformed += 1
                continue
            parse_us = (time.perf_counter_ns() - parse_start_ns) / 1_000.0

            point_count = min(
                len(cloud.ranges), len(cloud.azimuths), len(cloud.elevations)
            )
            message_bytes = len(topic_bytes) + len(payload)
            source_age_us = plausible_source_age_us(received_ns, cloud.stamp_ns)
            _message_id, database_us = store_cloud(
                connection,
                cloud,
                topic,
                received_ns,
                message_bytes,
                receive_wait_us,
                parse_us,
                source_age_us,
                arguments.sample_points,
            )

            total_messages += 1
            if total_messages == 1 and point_count > 0:
                sample_index = point_count // 2
                sample_x, sample_y, sample_z = xyz(
                    float(cloud.ranges[sample_index]),
                    float(cloud.azimuths[sample_index]),
                    float(cloud.elevations[sample_index]),
                )
                sample_intensity = (
                    float(cloud.intensities[sample_index])
                    if sample_index < len(cloud.intensities)
                    else float("nan")
                )
                print(
                    "First decoded sample: "
                    f"index={sample_index}, x={sample_x:.3f} m, "
                    f"y={sample_y:.3f} m, z={sample_z:.3f} m, "
                    f"intensity={sample_intensity:.2f}\n"
                )
            window.messages += 1
            window.bytes += message_bytes
            window.points += point_count
            window.wait_us.append(receive_wait_us)
            window.parse_us.append(parse_us)
            window.database_us.append(database_us)
            if source_age_us is not None:
                window.source_age_us.append(source_age_us)

            now = time.monotonic()
            elapsed = now - last_report
            if elapsed < 1.0:
                continue

            rate_hz = window.messages / elapsed
            bytes_per_second = window.bytes / elapsed
            current_database_size = database_size(arguments.database)
            database_growth = max(0, current_database_size - previous_database_size)
            database_growth_per_second = database_growth / elapsed
            age_text = (
                f"{mean_or_zero(window.source_age_us):.1f} us"
                if window.source_age_us
                else "clock mismatch/unavailable"
            )
            print(
                f"clouds={total_messages:7d}  rate={rate_hz:6.2f}/s  "
                f"points={window.points // max(window.messages, 1):6d}/cloud  "
                f"message={human_bytes(window.bytes / max(window.messages, 1))}"
            )
            print(
                f"  receive-wait={mean_or_zero(window.wait_us):9.1f} us  "
                f"protobuf={mean_or_zero(window.parse_us):8.1f} us  "
                f"sqlite={mean_or_zero(window.database_us):9.1f} us  "
                f"source-age={age_text}"
            )
            print(
                f"  wire rate: {human_bytes(bytes_per_second)}/s | "
                f"{human_bytes(bytes_per_second * 60.0)}/min | "
                f"{human_bytes(bytes_per_second * 3600.0)}/hour"
            )
            print(
                f"  SQLite: {human_bytes(current_database_size)} total | "
                f"growth {human_bytes(database_growth_per_second)}/s | "
                f"{human_bytes(database_growth_per_second * 60.0)}/min | "
                f"{human_bytes(database_growth_per_second * 3600.0)}/hour | "
                f"malformed={malformed}\n"
            )

            previous_database_size = current_database_size
            last_report = now
            window.reset()
    finally:
        subscriber.close(linger=0)
        context.term()
        connection.close()

    print(
        f"Stopped: {total_messages} valid clouds, {malformed} malformed, "
        f"SQLite size {human_bytes(database_size(arguments.database))}."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
