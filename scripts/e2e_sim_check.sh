#!/usr/bin/env bash
# Run the real MuJoCo digital twin through unified hand.launch.py use_sim:=true.
set -eo pipefail
cd "$(dirname "$0")/.."

source /opt/ros/lyrical/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"

LOG_DIR="${E2E_SIM_LOG_DIR:-$(mktemp -d /tmp/tuinadex_sim_e2e_XXXXXX)}"
ros2 launch tuinadex_hand hand.launch.py use_sim:=true \
  >"$LOG_DIR/hand_sim_launch.log" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

python3 scripts/sim_loop_probe.py \
  --position 0.2 --rate-hz 50 --active-sec 1.5 --settle-sec 2.0

echo "[E2E] PASS: unified sim launch closed loop completed"
echo "Launch log: $LOG_DIR/hand_sim_launch.log"
