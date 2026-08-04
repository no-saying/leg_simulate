#!/bin/bash
set -e
cd /home/yeht/COD-2026RoboMaster-Balance-Simulation_File/MJCF/control
make clean
make
cd ..
mkdir -p logs
LOG="logs/run_$(date +%Y%m%d_%H%M%S).log"
echo "$LOG" > logs/latest_run_log.txt
stdbuf -oL -eL ./controller 2>&1 | tee "$LOG" | python3 -u scripts/plot_fwd.py
