#!/usr/bin/env bash
# Hub full-stack check with real EGL rendering and real H.264 export.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${TUINADEX_CONTROL_PYTHON:-$HOME/win_office/conda/envs/tuinadex-control/bin/python}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"

WORK="$(mktemp -d)"
LOG="$WORK/hub.log"
PID=""

cleanup() {
  if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
    kill -TERM "$PID" 2>/dev/null || true
    wait "$PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

cd "$ROOT"
"$PY" scripts/collect_teleop_data.py \
  --env sim --mode arm_hand --mock-teleop --hub --headless \
  --host 127.0.0.1 --port 0 --fps 50 --ui-fps 25 \
  --cameras overhead,wrist,side \
  --repo-id "$WORK/repo" > "$LOG" 2>&1 &
PID=$!

PORT=""
for _ in $(seq 1 60); do
  PORT="$(grep -m1 -o 'HUB_READY port=[0-9]*' "$LOG" 2>/dev/null | cut -d= -f2 || true)"
  [[ -n "$PORT" ]] && break
  sleep 0.5
done
if [[ -z "$PORT" ]]; then
  echo "[E2E] FAIL: hub did not report HUB_READY"
  tail -n 40 "$LOG"
  exit 1
fi
echo "[E2E] hub listening on $PORT"

BASE="http://127.0.0.1:$PORT"
curl -fsS -X POST "$BASE/api/action/label" \
  -H 'Content-Type: application/json' \
  -d '{"phase_label":"press","language_instruction":"按压大椎穴"}' \
  > /dev/null
curl -fsS -X POST "$BASE/api/action/start" \
  -H 'Content-Type: application/json' -d '{}' > /dev/null
sleep 1.2
curl -fsS -X POST "$BASE/api/action/stop" \
  -H 'Content-Type: application/json' -d '{}' > /dev/null
curl -fsS -X POST "$BASE/api/action/save" \
  -H 'Content-Type: application/json' -d '{}' > /dev/null

STATE=""
for _ in $(seq 1 120); do
  STATE="$(curl -fsS "$BASE/api/status" | "$PY" -c 'import sys,json;print(json.load(sys.stdin)["state"])')"
  [[ "$STATE" == "IDLE" ]] && break
  sleep 0.5
done
if [[ "$STATE" != "IDLE" ]]; then
  echo "[E2E] FAIL: state stuck at $STATE"
  tail -n 40 "$LOG"
  exit 1
fi

kill -TERM "$PID" 2>/dev/null || true
wait "$PID" 2>/dev/null || true
PID=""

"$PY" - "$WORK/repo" <<'PY'
import json
import sys
from pathlib import Path

repo = Path(sys.argv[1])
info_path = repo / "meta" / "info.json"
assert info_path.is_file(), f"missing {info_path}"
info = json.loads(info_path.read_text(encoding="utf-8"))
features = info["features"]

required = [
    "observation.state",
    "observation.effort",
    "observation.contact_forces",
    "action",
    "phase_index",
    "observation.images.overhead",
    "observation.images.wrist",
    "observation.images.side",
]
missing = [key for key in required if key not in features]
assert not missing, f"missing features: {missing}"
assert info["fps"] == 50, info["fps"]
assert info["total_episodes"] == 1, info["total_episodes"]
assert info["total_frames"] >= 20, info["total_frames"]
assert features["observation.state"]["shape"] == [22]
assert features["observation.contact_forces"]["shape"] == [4]

for camera in ("overhead", "wrist", "side"):
    videos = list(
        (repo / f"videos/observation.images.{camera}").rglob("*.mp4")
    )
    assert videos, f"no MP4 for {camera}"
    assert all(video.stat().st_size > 0 for video in videos)

assert list((repo / "data").rglob("*.parquet"))
phase_path = repo / "meta" / "phase_labels.json"
assert phase_path.is_file(), "missing phase labels"
phase_map = json.loads(phase_path.read_text(encoding="utf-8"))
assert "press" in phase_map.values(), phase_map
print(
    "[E2E] dataset OK: "
    f"frames={info['total_frames']} episodes={info['total_episodes']}"
)
PY

echo "[E2E] PASS"
