#!/usr/bin/env bash
#
# scripts/start_lidar_service_2.sh  sensor 2's own independent copy of
# start_lidar_service.sh. See that file for the full explanation of why the
# script is built the way it is (supervisor loops, Ctrl+C handling, etc) --
# this file mirrors it exactly, just with sensor 2's own settings below.
#
# THIS SCRIPT IS SENSOR 2 ONLY. Run both scripts in two separate terminals
# to run both sensors together -- they cannot affect each other.

set -u

BUILD_DIR="$HOME/Documents/auv_middleware/build"

SICK_LAUNCH_FILE="$HOME/sick_scan_ws/sick_scan_xd/launch/sick_picoscan.launch"
SENSOR_IP="192.168.12.222"
THIS_MACHINE_IP="192.168.12.240"

LOG_PATH="$HOME/lidar_data/dive2.log"
DB_PATH="$HOME/lidar_data/dive2.db"

PUBLISH_ENDPOINT="tcp://*:5557"
QUERY_ENDPOINT="tcp://*:5561"

RESTART_BACKOFF_INITIAL_S=2
RESTART_BACKOFF_MAX_S=30
MIN_HEALTHY_UPTIME_S=30

PUBLISHER_BIN="$BUILD_DIR/apps/scan_publisher_recorder_main"
QUERY_SERVER_BIN="$BUILD_DIR/apps/scan_query_server_main"

for bin in "$PUBLISHER_BIN" "$QUERY_SERVER_BIN"; do
  if [[ ! -x "$bin" ]]; then
    echo "start_lidar_service_2: missing or not executable: $bin" >&2
    echo "  (build it first: cmake --build \"$BUILD_DIR\" -j4)" >&2
    exit 1
  fi
done

mkdir -p "$(dirname "$LOG_PATH")"

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
    echo "start_lidar_service_2: $label exited unexpectedly (code $rc) after ${ran_s}s -- restarting in ${backoff}s" >&2

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
QUERY_SERVER_PIDFILE="$(mktemp)"
PUBLISHER_SUPERVISOR_PID=""
QUERY_SERVER_SUPERVISOR_PID=""

cleanup() {
  trap '' INT TERM
  echo ""
  echo "start_lidar_service_2: stopping..."
  [[ -n "$PUBLISHER_SUPERVISOR_PID" ]]    && kill -INT "$PUBLISHER_SUPERVISOR_PID"    2>/dev/null
  [[ -n "$QUERY_SERVER_SUPERVISOR_PID" ]] && kill -INT "$QUERY_SERVER_SUPERVISOR_PID" 2>/dev/null
  wait "$PUBLISHER_SUPERVISOR_PID" "$QUERY_SERVER_SUPERVISOR_PID" 2>/dev/null
  rm -f "$PUBLISHER_PIDFILE" "$QUERY_SERVER_PIDFILE"
  echo "start_lidar_service_2: stopped."
}
trap cleanup INT TERM

echo "start_lidar_service_2: starting publisher+recorder (log: $LOG_PATH, port 5557) ..."
supervise "publisher+recorder" "$PUBLISHER_PIDFILE" \
    "$PUBLISHER_BIN" "$SICK_LAUNCH_FILE" "$SENSOR_IP" "$THIS_MACHINE_IP" \
    "$LOG_PATH" "$DB_PATH" "$PUBLISH_ENDPOINT" &
PUBLISHER_SUPERVISOR_PID=$!

sleep 2

echo "start_lidar_service_2: starting query backup server (port 5561) ..."
supervise "query server" "$QUERY_SERVER_PIDFILE" \
    "$QUERY_SERVER_BIN" "$LOG_PATH" "$DB_PATH" "$QUERY_ENDPOINT" &
QUERY_SERVER_SUPERVISOR_PID=$!

echo "start_lidar_service_2: both running. live: tcp://$THIS_MACHINE_IP:5557  backup: tcp://$THIS_MACHINE_IP:5561"
echo "start_lidar_service_2: either one will auto-restart if it exits; Ctrl+C to stop both for real."

wait "$PUBLISHER_SUPERVISOR_PID" "$QUERY_SERVER_SUPERVISOR_PID"