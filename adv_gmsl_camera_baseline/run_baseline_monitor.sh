#!/bin/bash

set -u

# ============================================================
# Configuration
# ============================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPIC="${TOPIC:-/camera/image_raw}"
RESULT_DIR="${RESULT_DIR:-${SCRIPT_DIR}/baseline_results}"

# ============================================================
# PIDs
# ============================================================

TEGRA_PID=""
HZ_PID=""
BW_PID=""
BASELINE_PID=""

# ============================================================
# Helpers
# ============================================================

log()
{
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"
}

stop_process()
{
    local pid="$1"
    local name="$2"

    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
        log "Stopping ${name} (PID=${pid})..."
        kill -INT "${pid}" 2>/dev/null || true
    fi
}

cleanup()
{
    echo
    log "Ctrl+C received, stopping baseline test..."
	trap - INT TERM
    # Stop ROS2 baseline launch first.
    stop_process "${BASELINE_PID}" "baseline_launch.py"

    # Stop topic monitoring.
    stop_process "${HZ_PID}" "ros2 topic hz"
    stop_process "${BW_PID}" "ros2 topic bw"

    # Stop tegrastats.
    stop_process "${TEGRA_PID}" "tegrastats"

    # Give processes some time to exit cleanly.
    sleep 1

    # Force kill anything still alive.
    if [[ -n "${BASELINE_PID}" ]] &&
       kill -0 "${BASELINE_PID}" 2>/dev/null; then
        kill -TERM "${BASELINE_PID}" 2>/dev/null || true
    fi

    if [[ -n "${HZ_PID}" ]] &&
       kill -0 "${HZ_PID}" 2>/dev/null; then
        kill -TERM "${HZ_PID}" 2>/dev/null || true
    fi

    if [[ -n "${BW_PID}" ]] &&
       kill -0 "${BW_PID}" 2>/dev/null; then
        kill -TERM "${BW_PID}" 2>/dev/null || true
    fi

    if [[ -n "${TEGRA_PID}" ]] &&
       kill -0 "${TEGRA_PID}" 2>/dev/null; then
        kill -TERM "${TEGRA_PID}" 2>/dev/null || true
    fi

    wait "${BASELINE_PID}" 2>/dev/null || true
    wait "${HZ_PID}" 2>/dev/null || true
    wait "${BW_PID}" 2>/dev/null || true
    wait "${TEGRA_PID}" 2>/dev/null || true

    log "Baseline test stopped."
    exit 0;
}

trap cleanup INT TERM EXIT

# ============================================================
# Prepare result directory
# ============================================================

mkdir -p "${RESULT_DIR}"

log "Result directory: ${RESULT_DIR}"
log "Topic: ${TOPIC}"

# ============================================================
# Start tegrastats
# ============================================================

log "Starting tegrastats..."

tegrastats \
    --interval 1000 \
    --logfile "${RESULT_DIR}/tegrastats.log" &

TEGRA_PID=$!

# ============================================================
# Start ros2 topic hz
# ============================================================

log "Starting ros2 topic hz..."

ros2 topic hz "${TOPIC}" --window 200 \
    > "${RESULT_DIR}/topic_hz.log" 2>&1 &

HZ_PID=$!

# ============================================================
# Start ros2 topic bw
# ============================================================

log "Starting ros2 topic bw..."

ros2 topic bw "${TOPIC}" \
    > "${RESULT_DIR}/topic_bw.log" 2>&1 &

BW_PID=$!

# ============================================================
# Wait
# ============================================================

echo
log "Monitoring started."
log "Run baseline_launch.py separately."
log "Press Ctrl+C to stop all monitoring processes."
echo

while true; do
    sleep 1
done
