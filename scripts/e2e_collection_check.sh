#!/usr/bin/env bash
# ==============================================================================
# e2e_collection_check.sh — 多模态数据采集与 LeRobotDataset v3.0 自动化端到端回环自检
# ==============================================================================
set -euo pipefail

TMP_DIR=$(mktemp -d /tmp/tuinadex_e2e_v3_XXXX)
DATASET_DIR="${TMP_DIR}/tuina_v3_test"
trap 'rm -rf "${TMP_DIR}"' EXIT

echo "============================================================"
echo " [E2E COLLECTION] 开始多模态数据采集与 LeRobot v3.0 自动化回环自检..."
echo " 临时数据集路径: ${DATASET_DIR}"
echo "============================================================"

# 1. 在 tuinadex-vla 环境中运行采集脚本 (录制 1 试次，30 帧，含双目 head/wrist 视频)
~/win_office/conda/envs/tuinadex-vla/bin/python scripts/collect_teleop_data.py \
    --env sim \
    --standalone \
    --mode hand_only \
    --cameras head,wrist \
    --fps 50 \
    --mock-teleop \
    --episodes 1 \
    --frames-per-episode 30 \
    --repo-id "${DATASET_DIR}"

# 2. 检查生成的目录结构与元数据
if [ ! -f "${DATASET_DIR}/meta/info.json" ]; then
    echo "[FAIL] 未生成 meta/info.json" >&2
    exit 1
fi

echo "[INFO] 正在核验 LeRobotDataset v3.0 元数据与视频分块..."
~/win_office/conda/envs/tuinadex-vla/bin/python - <<EOF
import json
from pathlib import Path
from lerobot.datasets.lerobot_dataset import LeRobotDataset

root = Path("${DATASET_DIR}")
info_path = root / "meta" / "info.json"
with open(info_path) as f:
    meta = json.load(f)

assert meta["codebase_version"] == "v3.0", f"Unexpected version: {meta['codebase_version']}"
assert meta["total_frames"] == 30, f"Unexpected frames: {meta['total_frames']}"
assert "observation.images.head" in meta["features"], "Missing head camera feature"
assert "observation.images.wrist" in meta["features"], "Missing wrist camera feature"
assert meta["features"]["observation.images.head"]["dtype"] == "video", "head is not video"
assert meta["features"]["observation.images.wrist"]["dtype"] == "video", "wrist is not video"

# 检查视频 MP4 真实存在
videos = list((root / "videos").rglob("*.mp4"))
assert len(videos) >= 2, f"Expected at least 2 video files, got {len(videos)}"

# 原生加载 LeRobotDataset 并核验帧数据
ds = LeRobotDataset(root.name, root=root)
assert len(ds) == 30
frame0 = ds[0]
assert frame0["observation.state"].shape == (16,)
assert frame0["action"].shape == (16,)
assert "observation.images.head" in frame0
assert "observation.images.wrist" in frame0
print("Dataset validation passed! Frames: 30, Features verified.")
EOF

echo "============================================================"
echo " [E2E COLLECTION] PASS: LeRobotDataset v3.0 多模态数据采集全链路验证通过!"
echo "============================================================"
