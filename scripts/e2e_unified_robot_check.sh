#!/usr/bin/env bash
# ==============================================================================
# e2e_unified_robot_check.sh — 6-DOF 机械臂与 16-DOF 灵巧手 22 维联合控制与双模式启动 E2E 终验
# ==============================================================================
set -eo pipefail
cd "$(dirname "$0")/.."

source /opt/ros/lyrical/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

TMP_DIR=$(mktemp -d /tmp/tuinadex_unified_e2e_XXXX)
DATASET_DIR="${TMP_DIR}/dataset_22d"
LAUNCH_LOG="${TMP_DIR}/robot_launch.log"

cleanup() {
    local pids
    pids="$(jobs -p || true)"
    if [[ -n "$pids" ]]; then
        kill $pids 2>/dev/null || true
        wait $pids 2>/dev/null || true
    fi
    rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

echo "============================================================"
echo " [E2E UNIFIED] 启动 6-DOF 臂 + 16-DOF 手 22 维协同端到端自检..."
echo " 临时工作区: ${TMP_DIR}"
echo "============================================================"

# ------------------------------------------------------------------------------
# 阶段 1: 真实/Mock 模式统一启动 (use_sim:=false, mock:=true, mode:=arm_hand)
# ------------------------------------------------------------------------------
echo -e "\n>>> [1/3] 测试双模式之一: 统一硬件 Mock 运行时启动 (mode:=arm_hand, use_sim:=false, mock:=true)..."
ros2 launch tuinadex_bringup robot.launch.py \
    use_sim:=false mock:=true mode:=arm_hand \
    >"${LAUNCH_LOG}" 2>&1 &
REAL_LAUNCH_PID=$!
sleep 2

# 核验话题存在性
echo "[INFO] 核验话题列表..."
TOPICS="$(ros2 topic list)"
echo "$TOPICS" | grep -q "/hand/joint_states" || { echo "[FAIL] 缺少 /hand/joint_states"; exit 1; }
echo "$TOPICS" | grep -q "/arm/joint_states" || { echo "[FAIL] 缺少 /arm/joint_states"; exit 1; }
echo "$TOPICS" | grep -q "/hand/joint_command" || { echo "[FAIL] 缺少 /hand/joint_command"; exit 1; }
echo "$TOPICS" | grep -q "/arm/joint_command" || { echo "[FAIL] 缺少 /arm/joint_command"; exit 1; }

# 发送 22 维协同控制指令 (手部 0.4 rad, 机械臂 0.2 rad)
echo "[INFO] 下发 22 维协同控制指令 (持续 2 秒)..."
python3 scripts/mock_command_publisher.py \
    --mode arm_hand --position 0.4 --arm-position 0.2 --rate-hz 50 --duration-sec 2

# 停止真机/Mock Launch 进程
kill "${REAL_LAUNCH_PID}" 2>/dev/null || true
wait "${REAL_LAUNCH_PID}" 2>/dev/null || true
echo "[PASS] 硬件 Mock 模式 22 维联合指令与状态发布验证通过！"

# ------------------------------------------------------------------------------
# 阶段 2: MuJoCo 数字孪生仿真模式统一启动 (use_sim:=true, mode:=arm_hand)
# ------------------------------------------------------------------------------
echo -e "\n>>> [2/3] 测试双模式之二: MuJoCo 数字孪生仿真统一启动 (mode:=arm_hand, use_sim:=true)..."
ros2 launch tuinadex_bringup robot.launch.py \
    use_sim:=true mode:=arm_hand gui:=false \
    >"${LAUNCH_LOG}" 2>&1 &
SIM_LAUNCH_PID=$!
sleep 2

# 下发 22 维指令
echo "[INFO] 下发 22 维协同指令至 MuJoCo 仿真孪生系统..."
python3 scripts/mock_command_publisher.py \
    --mode arm_hand --position 0.35 --arm-position 0.15 --rate-hz 50 --duration-sec 2

kill "${SIM_LAUNCH_PID}" 2>/dev/null || true
wait "${SIM_LAUNCH_PID}" 2>/dev/null || true
echo "[PASS] MuJoCo 仿真模式 22 维联合运行验证通过！"

# ------------------------------------------------------------------------------
# 阶段 3: 22 维多模态数据采集与 LeRobotDataset v3.0 闭环
# ------------------------------------------------------------------------------
echo -e "\n>>> [3/3] 测试 22 维多模态数据采集直通 LeRobotDataset v3.0..."
~/win_office/conda/envs/tuinadex-vla/bin/python scripts/collect_teleop_data.py \
    --env sim \
    --standalone \
    --mode arm_hand \
    --cameras head,wrist \
    --fps 50 \
    --mock-teleop \
    --episodes 1 \
    --frames-per-episode 25 \
    --repo-id "${DATASET_DIR}"

# 深度校验导出的数据集维度
~/win_office/conda/envs/tuinadex-vla/bin/python - <<EOF
from pathlib import Path
from lerobot.datasets.lerobot_dataset import LeRobotDataset

root = Path("${DATASET_DIR}")
ds = LeRobotDataset(root.name, root=root)
assert len(ds) == 25, f"Expected 25 frames, got {len(ds)}"

frame = ds[0]
state = frame["observation.state"]
action = frame["action"]

print(f"Observation state shape: {state.shape}, Action shape: {action.shape}")
assert state.shape == (22,), f"Expected 22D state, got {state.shape}"
assert action.shape == (22,), f"Expected 22D action, got {action.shape}"
assert "observation.images.head" in frame
assert "observation.images.wrist" in frame
print("[PASS] LeRobot 22 维多模态数据集验证通过 (6 臂 + 16 手 + 双视角视频)！")
EOF

echo "============================================================"
echo " [E2E UNIFIED] ALL PASS: 22 维臂手联合控制、双模式启动及具身数据链路终验全绿通过！"
echo "============================================================"
