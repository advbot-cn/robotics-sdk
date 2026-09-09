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

