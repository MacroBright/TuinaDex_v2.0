#!/usr/bin/env bash
# ==============================================================================
# TuinaDex v2.0 - Environment Diagnostic & Health Check Tool
# Checks OS, ROS 2, DDS, Hardware Buses (CAN/USB), Conda, and Docker status.
# ==============================================================================

# ANSI Colors
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
BLUE="\033[0;34m"
BOLD="\033[1m"
NC="\033[0m"

echo -e "${BOLD}${BLUE}======================================================${NC}"
echo -e "${BOLD}${BLUE}       TuinaDex v2.0 - Environment Health Check       ${NC}"
echo -e "${BOLD}${BLUE}======================================================${NC}"

# 1. OS & Architecture
ARCH=$(uname -m)
KERNEL=$(uname -r)
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS_NAME="${NAME} ${VERSION}"
else
    OS_NAME="Unknown Linux"
fi
echo -e "\n${BOLD}[1] Operating System & Hardware Platform:${NC}"
echo -e "  - OS:           ${GREEN}${OS_NAME}${NC}"
echo -e "  - Architecture: ${GREEN}${ARCH}${NC}"
echo -e "  - Kernel:       ${GREEN}${KERNEL}${NC}"

# 2. ROS 2 Environment
echo -e "\n${BOLD}[2] ROS 2 & Middleware Status:${NC}"
if command -v ros2 >/dev/null 2>&1; then
    ROS_VER=$(ros2 --version 2>/dev/null || echo "Available")
    echo -e "  - ROS 2 CLI:    ${GREEN}Detected (${ROS_DISTRO:-unknown}) [${ROS_VER}]${NC}"
else
    if [ -d "/opt/ros" ]; then
        AVAILABLE_DISTROS=$(ls /opt/ros | tr '\n' ' ')
        echo -e "  - ROS 2:        ${YELLOW}Installed in /opt/ros (${AVAILABLE_DISTROS}), not sourced in current subshell${NC}"
    else
        echo -e "  - ROS 2:        ${RED}Not installed in /opt/ros${NC}"
    fi
fi

# Check DDS
if [ "${RMW_IMPLEMENTATION}" = "rmw_cyclonedds_cpp" ]; then
    echo -e "  - RMW Impl:     ${GREEN}${RMW_IMPLEMENTATION}${NC}"
else
    echo -e "  - RMW Impl:     ${YELLOW}${RMW_IMPLEMENTATION:-unset (default rmw_fastrtps_cpp)}${NC} -> Recommended: rmw_cyclonedds_cpp"
fi

if [ "${ROS_DOMAIN_ID}" = "42" ]; then
    echo -e "  - ROS_DOMAIN_ID:${GREEN}42 (Standard Baseline)${NC}"
else
    echo -e "  - ROS_DOMAIN_ID:${YELLOW}${ROS_DOMAIN_ID:-unset (0)}${NC} -> Baseline required: 42"
fi

# 3. Hardware Devices: USB-TTL & CAN
echo -e "\n${BOLD}[3] Physical Hardware Buses & Peripherals:${NC}"

# LEAP Hand USB-TTL
if [ -e "/dev/tuinadex_hand" ]; then
    echo -e "  - LEAP Hand:    ${GREEN}Detected via alias /dev/tuinadex_hand${NC}"
else
    USB_DEV_COUNT=$(ls /dev/ttyUSB* 2>/dev/null | wc -l)
    if [ "${USB_DEV_COUNT}" -gt 0 ]; then
        DEVS=$(ls /dev/ttyUSB* | tr '\n' ' ')
        echo -e "  - LEAP Hand:    ${YELLOW}Raw ports detected (${DEVS}), udev alias /dev/tuinadex_hand not yet assigned${NC}"
    else
        echo -e "  - LEAP Hand:    ${YELLOW}No /dev/ttyUSB* detected (OK if mock or disconnected)${NC}"
    fi
fi

# reBot-DevArm SocketCAN
if ip link show can0 >/dev/null 2>&1; then
    CAN_STATE=$(ip -details link show can0 | grep -o "state [A-Z]*" || echo "state UP")
    CAN_BITRATE=$(ip -details link show can0 | grep -o "bitrate [0-9]*" || echo "")
    echo -e "  - reBot CAN:    ${GREEN}can0 detected (${CAN_STATE}, ${CAN_BITRATE})${NC}"
else
    echo -e "  - reBot CAN:    ${YELLOW}can0 interface not active (run: setup SocketCAN or check USB2CAN)${NC}"
fi

# 4. Compute Accelerators (NVIDIA GPU / Ascend NPU)
echo -e "\n${BOLD}[4] Compute Accelerators:${NC}"
if command -v nvidia-smi >/dev/null 2>&1; then
    GPU_NAME=$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>/dev/null | head -n 1)
    echo -e "  - NVIDIA GPU:   ${GREEN}${GPU_NAME}${NC}"
elif command -v npu-smi >/dev/null 2>&1; then
    NPU_INFO=$(npu-smi info 2>/dev/null | grep -i "310B" | head -n 1)
    echo -e "  - Ascend NPU:   ${GREEN}${NPU_INFO:-Detected Ascend NPU}${NC}"
else
    echo -e "  - Accelerator:  ${YELLOW}No nvidia-smi or npu-smi detected (CPU-only mode)${NC}"
fi

# 5. Conda / Python Environments
echo -e "\n${BOLD}[5] Conda / Virtual Environments:${NC}"
CONDA_DIR="${HOME}/win_office/conda/envs"
for env_name in "leap_hand" "arm_robot" "smolvla"; do
    if [ -d "${CONDA_DIR}/${env_name}" ]; then
        PY_VER=$("${CONDA_DIR}/${env_name}/bin/python" --version 2>&1 || echo "error")
        echo -e "  - Conda ${env_name}: ${GREEN}Found (${PY_VER})${NC}"
    else
        echo -e "  - Conda ${env_name}: ${YELLOW}Not found in ${CONDA_DIR}/${NC}"
    fi
done

# 6. Docker Daemon
echo -e "\n${BOLD}[6] Containerization (Docker):${NC}"
if command -v docker >/dev/null 2>&1; then
    if docker info >/dev/null 2>&1; then
        DOCKER_VER=$(docker --version)
        echo -e "  - Docker:       ${GREEN}Daemon running [${DOCKER_VER}]${NC}"
    else
        echo -e "  - Docker:       ${YELLOW}Installed, but daemon not running or current user lacks permissions${NC}"
    fi
else
    echo -e "  - Docker:       ${RED}Docker not installed${NC}"
fi

echo -e "\n${BOLD}${BLUE}======================================================${NC}"
echo -e "${BOLD}${BLUE}               Health Check Completed                 ${NC}"
echo -e "${BOLD}${BLUE}======================================================${NC}"
exit 0
