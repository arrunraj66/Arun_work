#!/usr/bin/env bash
#
# scripts/start_lidar3d_service.sh — starts the 3D (multiScan136) publisher
# as a supervised background service, the 3D counterpart of
# start_lidar_service.sh.
#
# Deliberately simpler than the 2D script for now: only ONE binary
# (cloud_publisher_main) under supervision, no recorder and no query backup
# server yet -- those are Step 8, not built yet. When they exist this script
# grows a second `supervise` call, copying the exact pattern below.
#
# The supervisor loop, signal handling (double-Ctrl+C safety, forwarding a
# real SIGINT so the sensor gets its stop command before this exits) and
# backoff are copied verbatim from start_lidar_service.sh's `supervise()` --
# same reasoning applies unchanged, see the comments over there for the full
# explanation of why each piece exists.
#
# THIS SCRIPT ALSO BAKES IN THE SICK SDK PATH so `cmake -S . -B build` never
# needs to guess it -- see cmake/SickScanXd.cmake for the matching default.
#
# Run: ./scripts/start_lidar3d_service.sh
# Stop: Ctrl+C (once) in the same terminal.

set -u

# ---------------------------------------------------------------------------
# Settings -- the only part of this file you should need to touch.
# ---------------------------------------------------------------------------
BUILD_DIR="$HOME/Documents/auv_middleware/build"

SICK_LAUNCH_FILE="$HOME/sick_scan_ws/sick_scan_xd/launch/sick_multiscan.launch"
SENSOR_IP="192.168.12.223"
THIS_MACHINE_IP="192.168.12.240"

PUBLISH_ENDPOINT="tcp://*:5580"
PUBLISH_TOPIC="lidar.cloud"

# Distinct from the 2D service's UDP ports (2115/7503, sick_picoscan.launch's
# own defaults) specifically so BOTH services can run on this machine at the
# same time without one sensor's driver silently stealing the other's port.
UDP_PORT=2125
IMU_UDP_PORT=7513

RESTART_BACKOFF_INITIAL_S=2
RESTART_BACKOFF_MAX_S=30
MIN_HEALTHY_UPTIME_S=30
# ---------------------------------------------------------------------------

PUBLISHER_BIN="$BUILD_DIR/apps/cloud_publisher_main"

if [[ ! -x "$PUBLISHER_BIN" ]]; then
  echo "start_lidar3d_service: missing or not executable: $PUBLISHER_BIN" >&2
  echo "  (build it first: cmake --build \"$BUILD_DIR\" -j4)" >&2
  exit 1
fi

# supervise LABEL PIDFILE CMD... -- identical to start_lidar_service.sh's
# version; see that file's comment block for the full explanation of the
# trap/backoff/double-Ctrl+C handling.
supervise() {
  local label="$1" pidfile="$2"; shift 2
  trap - INT
  local stopping=0
  local child_pid=""
  trap 'stopping=1; [[ -n "$child_pid" ]] && kill -INT "$child_pid" 2>/dev/null; trap "" INT TERM' INT TERM
  local backoff="$RESTART_BACKOFF_INITIAL_S"

  while [[ "$stopping" -eq 0 ]]; do
    local start_s=$SECONDS
    ( trap - INT; exec "$@" ) &
    child_pid=$!
    echo "$child_pid" > "$pidfile"

    wait "$child_pid"
    local rc=$?

    if [[ "$stopping" -eq 1 ]]; then
      break
    fi

    local ran_s=$(( SECONDS - start_s ))
    if [[ "$ran_s" -ge "$MIN_HEALTHY_UPTIME_S" ]]; then
      backoff="$RESTART_BACKOFF_INITIAL_S"
    fi
    echo "start_lidar3d_service: $label exited unexpectedly (code $rc) after ${ran_s}s -- restarting in ${backoff}s" >&2

    sleep "$backoff"
    if [[ "$stopping" -eq 1 ]]; then
      break
    fi
    backoff=$(( backoff * 2 ))
    [[ "$backoff" -gt "$RESTART_BACKOFF_MAX_S" ]] && backoff="$RESTART_BACKOFF_MAX_S"
  done

  wait "$child_pid" 2>/dev/null
  echo "0" > "$pidfile"
}

PUBLISHER_PIDFILE="$(mktemp)"
PUBLISHER_SUPERVISOR_PID=""

cleanup() {
  trap '' INT TERM
  echo ""
  echo "start_lidar3d_service: stopping..."
  [[ -n "$PUBLISHER_SUPERVISOR_PID" ]] && kill -INT "$PUBLISHER_SUPERVISOR_PID" 2>/dev/null
  wait "$PUBLISHER_SUPERVISOR_PID" 2>/dev/null
  rm -f "$PUBLISHER_PIDFILE"
  echo "start_lidar3d_service: stopped."
}
trap cleanup INT TERM

echo "start_lidar3d_service: starting 3D publisher (udp_port=$UDP_PORT imu_udp_port=$IMU_UDP_PORT, port ${PUBLISH_ENDPOINT##*:}) ..."
supervise "cloud publisher" "$PUBLISHER_PIDFILE" \
    "$PUBLISHER_BIN" "$SICK_LAUNCH_FILE" "$SENSOR_IP" "$THIS_MACHINE_IP" \
    "$PUBLISH_ENDPOINT" "$PUBLISH_TOPIC" "$UDP_PORT" "$IMU_UDP_PORT" &
PUBLISHER_SUPERVISOR_PID=$!

echo "start_lidar3d_service: running. live: tcp://$THIS_MACHINE_IP:${PUBLISH_ENDPOINT##*:}  topic: $PUBLISH_TOPIC"
echo "start_lidar3d_service: will auto-restart if it exits; Ctrl+C to stop for real."

wait "$PUBLISHER_SUPERVISOR_PID"
