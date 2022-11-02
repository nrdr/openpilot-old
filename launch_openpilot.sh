#!/usr/bin/bash

# Clear live parameters every 30 seconds for optimal performance.
while true; do rm -rf /data/params/d/LiveParameters; sleep 30; done &

export PASSIVE="0"
exec ./launch_chffrplus.sh

