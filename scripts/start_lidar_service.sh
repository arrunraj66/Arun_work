#!/usr/bin/env bash
#
# scripts/start_lidar_service.sh — the one command that starts everything the
# GUI team depends on: the live push stream (port 5556) and the pull-based
# backup query service (port 5560), both reading/writing the same recording.
#
# This is a thin wrapper, not a rewrite -- it runs the exact two already-built
# and already-tested binaries (scan_publisher_recorder_main,
# scan_query_server_main) as background jobs of one shell, and makes sure
# Ctrl+C stops BOTH of them cleanly instead of leaving one orphaned. That
# orphaned-process problem is exactly what caused the "no scan within 1000ms"
# confusion earlier in this project (a suspended scan_publisher_main still
# holding the UDP port) -- this script exists so that mistake can't happen
# from here on: there is only one thing to start, and only one thing to stop.
#
# AUTO-RESTART: each of the two binaries is run under its own small
# supervisor loop, not just backgrounded directly. If either one exits --
# a crash, the sensor connection dying in a way scan_publisher_recorder_main's
# own watchdog couldn't recover from, anything -- its supervisor restarts it
# automatically, with a growing backoff (2s, 4s, 8s, ... capped at 30s) so a
# genuinely broken binary doesn't spin the CPU restarting hundreds of times a
# second. The backoff resets back to 2s once a restarted process has stayed
# up for a healthy stretch (30s), so one bad flap doesn't leave the service
# limping at a 30s restart delay forever afterward. Critically, the two
# supervisors are independent: the query server crashing does not stop the
# publisher/recorder, and vice versa -- previously (via `wait -n` on the raw
# children) EITHER one exiting for ANY reason tore down the whole service,
# which is exactly what turned the SQLite "database is locked" bug into a
# full outage instead of one restart.
#
# Edit the settings block below once for your machine, then just run:
#   ./scripts/start_lidar_service.sh
# Stop everything with Ctrl+C (once) in the same terminal.

set -u

# ---------------------------------------------------------------------------
# Settings -- the only part of this file you should need to touch.
# ---------------------------------------------------------------------------
BUILD_DIR="$HOME/Documents/auv_middleware/build"

SICK_LAUNCH_FILE="$HOME/sick_scan_ws/sick_scan_xd/launch/sick_picoscan.launch"
SENSOR_IP="192.168.12.222"
THIS_MACHINE_IP="192.168.12.240"

LOG_PATH="$HOME/lidar_data/dive.log"
DB_PATH="$HOME/lidar_data/dive.db"

PUBLISH_ENDPOINT="tcp://*:5556"
QUERY_ENDPOINT="tcp://*:5560"

# How long a restarted process must stay up before its own backoff resets
# back down to RESTART_BACKOFF_INITIAL_S. How far the backoff is allowed to
# grow in between.
RESTART_BACKOFF_INITIAL_S=2
RESTART_BACKOFF_MAX_S=30
MIN_HEALTHY_UPTIME_S=30
# ---------------------------------------------------------------------------

PUBLISHER_BIN="$BUILD_DIR/apps/scan_publisher_recorder_main"
QUERY_SERVER_BIN="$BUILD_DIR/apps/scan_query_server_main"

for bin in "$PUBLISHER_BIN" "$QUERY_SERVER_BIN"; do
  if [[ ! -x "$bin" ]]; then
    echo "start_lidar_service: missing or not executable: $bin" >&2
    echo "  (build it first: cmake --build \"$BUILD_DIR\" -j4)" >&2
    exit 1
  fi
done

mkdir -p "$(dirname "$LOG_PATH")"

# supervise LABEL CMD...
#
# Runs CMD in a restart loop: start it, wait for it, and if it exits on its
# own (not because we told it to stop) restart it after a backoff. Exits
# once told to stop (via SIGINT/SIGTERM to THIS function's own process --
# see the trap below), after making sure the child it is currently running
# has actually exited.
#
# Two signal-handling details matter here, both learned the hard way
# earlier in this project:
#
# 1. Bash gives an asynchronous (`&`) command in a non-interactive script
#    SIGINT disposition "ignored" by default, so Ctrl+C on the script's own
#    process group can't accidentally kill a background job. That ignored
#    disposition survives exec() into a real compiled binary and would
#    silently swallow the SIGINT this function sends it below -- so every
#    process this function starts, INCLUDING THIS FUNCTION ITSELF (it also
#    runs as `supervise ... &` from main, below), gets `trap - INT` as its
#    very first action, resetting that disposition back to normal before
#    doing anything else.
#
# 2. Once disposition is back to normal, an explicit `trap '...' INT TERM`
#    here catches Ctrl+C/`kill` directed at this function's own PID, marks
#    `stopping=1`, and forwards a real SIGINT to whichever child is
#    currently running (by PID, captured in a variable the trap closure can
#    see). The main `wait "$child_pid"` below is interrupted by that
#    trapped signal and returns immediately -- not 30s later mid-backoff --
#    which is why the backoff sleep is a plain foreground `sleep`, not
#    another background job: only a FOREGROUND command is interrupted
#    promptly by a trapped signal partway through.
#
# 3. A SECOND Ctrl+C, pressed while the first is still being handled (e.g.
#    someone presses it again because the terminal briefly looks stuck
#    while the real binary's shutdown sequence runs), is real bash
#    behavior that bit this exact script on real hardware: bash's `wait`
#    returns immediately on ANY trapped signal, even one delivered while an
#    earlier delivery of the same signal is still being acted on -- so a
#    second Ctrl+C could make the FINAL `wait "$child_pid"` below (the one
#    meant to block until the real binary has actually finished exiting)
#    return early, letting this function report "stopped" before the child
#    -- and whatever real shutdown work it's still doing, like telling a
#    sensor to stop -- has actually finished. The fix is in the trap body
#    itself: `trap "" INT TERM` as its last action turns off our own
#    signal handling entirely, once, right after forwarding the real
#    signal the first time -- so a second Ctrl+C can no longer interrupt
#    the waits below early. It doesn't (and can't, from a wrapper script)
#    stop a second Ctrl+C from also reaching the real binary directly, the
#    same way the first one does -- that part is just standard Unix
#    behavior common to virtually every CLI tool: press Ctrl+C once and
#    let it finish shutting down, don't press it again while it's still
#    printing shutdown messages.
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
    echo "start_lidar_service: $label exited unexpectedly (code $rc) after ${ran_s}s -- restarting in ${backoff}s" >&2

    sleep "$backoff"
    if [[ "$stopping" -eq 1 ]]; then
      break
    fi
    backoff=$(( backoff * 2 ))
    [[ "$backoff" -gt "$RESTART_BACKOFF_MAX_S" ]] && backoff="$RESTART_BACKOFF_MAX_S"
  done

  # Block until the child we just signalled (if any) has actually finished
  # its real shutdown (sensor stop command / file flush) -- the `wait`
  # above can return as soon as the signal interrupts it, before the child
  # has finished exiting.
  wait "$child_pid" 2>/dev/null
  echo "0" > "$pidfile"
}

PUBLISHER_PIDFILE="$(mktemp)"
QUERY_SERVER_PIDFILE="$(mktemp)"
PUBLISHER_SUPERVISOR_PID=""
QUERY_SERVER_SUPERVISOR_PID=""

cleanup() {
  # Same fix as inside supervise() -- see point 3 in its comment above.
  # Disabling our own INT/TERM handling here, once, means a second Ctrl+C
  # can't interrupt the `wait` below early either.
  trap '' INT TERM
  echo ""
  echo "start_lidar_service: stopping..."
  [[ -n "$PUBLISHER_SUPERVISOR_PID" ]]    && kill -INT "$PUBLISHER_SUPERVISOR_PID"    2>/dev/null
  [[ -n "$QUERY_SERVER_SUPERVISOR_PID" ]] && kill -INT "$QUERY_SERVER_SUPERVISOR_PID" 2>/dev/null
  wait "$PUBLISHER_SUPERVISOR_PID" "$QUERY_SERVER_SUPERVISOR_PID" 2>/dev/null
  rm -f "$PUBLISHER_PIDFILE" "$QUERY_SERVER_PIDFILE"
  echo "start_lidar_service: stopped."
}
trap cleanup INT TERM

echo "start_lidar_service: starting publisher+recorder (log: $LOG_PATH, port 5556) ..."
supervise "publisher+recorder" "$PUBLISHER_PIDFILE" \
    "$PUBLISHER_BIN" "$SICK_LAUNCH_FILE" "$SENSOR_IP" "$THIS_MACHINE_IP" \
    "$LOG_PATH" "$DB_PATH" "$PUBLISH_ENDPOINT" &
PUBLISHER_SUPERVISOR_PID=$!

# A short head start before the query server opens the same log/db files --
# purely so the recorder has created them if this is a brand new recording;
# ScanQueryServer/ScanReader require the files to already exist.
sleep 2

echo "start_lidar_service: starting query backup server (port 5560) ..."
supervise "query server" "$QUERY_SERVER_PIDFILE" \
    "$QUERY_SERVER_BIN" "$LOG_PATH" "$DB_PATH" "$QUERY_ENDPOINT" &
QUERY_SERVER_SUPERVISOR_PID=$!

echo "start_lidar_service: both running. live: tcp://$THIS_MACHINE_IP:5556  backup: tcp://$THIS_MACHINE_IP:5560"
echo "start_lidar_service: either one will auto-restart if it exits; Ctrl+C to stop both for real."

# Unlike the old `wait -n` here, this blocks until BOTH supervisors have
# been told to stop and have finished (via cleanup(), triggered by the trap
# above) -- a single child dying no longer ends the script, since its own
# supervisor now handles that. Only an explicit Ctrl+C/kill on this script
# ends it.
wait "$PUBLISHER_SUPERVISOR_PID" "$QUERY_SERVER_SUPERVISOR_PID"
