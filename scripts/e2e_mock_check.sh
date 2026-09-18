#!/usr/bin/env bash
# End-to-end mock check: hand runtime + recorder + offline LeRobot conversion.
set -eo pipefail
cd "$(dirname "$0")/.."

source /opt/ros/lyrical/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

LEAP_HAND_PYTHON="${LEAP_HAND_PYTHON:-/home/bright/win_office/conda/envs/leap_hand/bin/python}"
SMOLVLA_PYTHON="${SMOLVLA_PYTHON:-/home/bright/win_office/conda/envs/smolvla/bin/python}"
WORK="$(mktemp -d /tmp/tuinadex_e2e_XXXXXX)"
KEEP_WORK="${E2E_KEEP_WORK:-0}"

cleanup() {
  local pids
  pids="$(jobs -p || true)"
  if [[ -n "$pids" ]]; then
    kill $pids 2>/dev/null || true
    wait $pids 2>/dev/null || true
  fi
  if [[ "$KEEP_WORK" != "1" ]]; then
    rm -rf "$WORK"
  fi
}
trap cleanup EXIT

ros2 run tuinadex_hand leap_hand_node --ros-args \
  -p mock_mode:=true -p control_rate_hz:=50.0 \
  >"$WORK/hand.log" 2>&1 &
sleep 1

ros2 run tuinadex_recorder tuinadex_recorder_node --ros-args \
  -p bag_root:="$WORK/bags" \
  >"$WORK/recorder.log" 2>&1 &
sleep 1
ros2 service call /recorder/start std_srvs/srv/Trigger >/dev/null

python3 scripts/mock_command_publisher.py \
  --position 0.2 --rate-hz 50 --duration-sec 3
sleep 2
ros2 service call /recorder/stop std_srvs/srv/Trigger >/dev/null

BAG="$(find "$WORK/bags" -maxdepth 1 -type d -name 'session_*' | sort | tail -1)"
if [[ -z "$BAG" ]]; then
  echo "E2E failed: no bag was created" >&2
  exit 1
fi

python3 - "$BAG" "$WORK/episode.npz" <<'PY'
import sys
from pathlib import Path

import numpy as np

ROOT = Path.cwd() / "dataset"
sys.path.insert(0, str(ROOT))
from tuinadex_dataset.adapter import convert_bag_to_episode

bag, output = sys.argv[1], sys.argv[2]
grid, states, actions, ok, errors = convert_bag_to_episode(bag)
if not ok:
    raise SystemExit("validation failed:\n  " + "\n  ".join(errors))
np.savez(output, grid=grid, states=states, actions=actions)
print(f"ROS-side conversion: {len(grid)} frames -> {output}")
PY

"$SMOLVLA_PYTHON" - "$WORK/episode.npz" "$WORK/dataset" <<'PY'
import sys
from pathlib import Path

import numpy as np

ROOT = Path.cwd() / "dataset"
sys.path.insert(0, str(ROOT))
from tuinadex_dataset.adapter import save_to_lerobot

episode, output = sys.argv[1], sys.argv[2]
data = np.load(episode)
result = save_to_lerobot(
    data["grid"], data["states"], data["actions"],
    repo_id=output, fps=50.0,
)
print(f"LeRobot dataset: {len(data['grid'])} frames -> {result}")
PY

if ! find "$WORK/dataset/data" -type f -name '*.parquet' -print -quit | grep -q .; then
  echo "E2E failed: expected LeRobot parquet was not found" >&2
  exit 1
fi

echo "[E2E] PASS: mock closed loop completed"
echo "Workspace: $WORK"
