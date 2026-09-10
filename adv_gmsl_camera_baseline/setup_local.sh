#!/bin/bash

log()
{
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"
}

if command -v nvpmodel >/dev/null 2>&1; then
    nvpmodel -m 0
else
    log "WARNING: nvpmodel not found, skipping."
fi

log "Enabling jetson_clocks..."

if command -v jetson_clocks >/dev/null 2>&1; then
    jetson_clocks
else
    log "WARNING: jetson_clocks not found, skipping."
fi

v4l2-ctl -d /dev/video2 -c sensor_mode=1,trig_pin=0xffff0007


RESULT_DIR="${RESULT_DIR:-${ISAAC_ROS_WS}/baseline_results}"
ENV_LOG="${RESULT_DIR}/environment.log"
{
    echo "===== Baseline Environment ====="
    echo "wall_time: $(date '+%Y-%m-%d %H:%M:%S %z')"
    echo

    echo "===== nvpmodel -q ====="
    if command -v nvpmodel >/dev/null 2>&1; then
        sudo nvpmodel -q 2>&1 || true
    else
        echo "nvpmodel: not found"
    fi

    echo

    echo "===== jetson_clocks --show ====="
    if command -v jetson_clocks >/dev/null 2>&1; then
        sudo jetson_clocks --show 2>&1 || true
    else
        echo "jetson_clocks: not found"
    fi

} > "${ENV_LOG}"

cat "${ENV_LOG}"

