#!/usr/bin/env bash
# ==============================================================================
# TuinaDex v2.0 - Orange Pi AI Pro 20T Initialization Script (TD_v2.0-ENV-EDGE)
# Configures Ubuntu 22.04 aarch64, Ascend NPU, ROS2 Humble, SocketCAN, and CycloneDDS.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# ANSI Colors
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
BLUE="\033[0;34m"
BOLD="\033[1m"
NC="\033[0m"

echo -e "${BOLD}${BLUE}======================================================${NC}"
echo -e "${BOLD}${BLUE} TuinaDex v2.0 - Orange Pi AI Pro Setup (ENV-EDGE)    ${NC}"
echo -e "${BOLD}${BLUE}======================================================${NC}"

ARCH=$(uname -m)
if [ "${ARCH}" != "aarch64" ]; then
    echo -e "${YELLOW}[Notice] Current CPU architecture is ${ARCH}. Target is aarch64 (Ascend 310B).${NC}"
    if [ "$1" != "--force" ] && [ "$1" != "--check-only" ]; then
        echo -e "Use '--force' if you intend to run setup steps on non-aarch64 for testing."
    fi
fi

# 1. Check Ascend NPU Driver & CANN
echo -e "\n${BOLD}[1/5] Checking Ascend 310B NPU & CANN Toolkit...${NC}"
if command -v npu-smi >/dev/null 2>&1; then
    NPU_STATUS=$(npu-smi info | grep -i "310B" || echo "Detected NPU")
    echo -e "  - ${GREEN}${NPU_STATUS}${NC}"
else
    echo -e "  - ${YELLOW}Warning: 'npu-smi' not found. Ensure official Ascend CANN 8.x drivers are installed.${NC}"
fi

# 2. Configure CycloneDDS Baseline
echo -e "\n${BOLD}[2/5] Configuring CycloneDDS Baseline...${NC}"
bash "${SCRIPT_DIR}/setup_cyclonedds.sh" --install
source "${SCRIPT_DIR}/setup_cyclonedds.sh"

# 3. Install SocketCAN Auto-start Service
echo -e "\n${BOLD}[3/5] Installing SocketCAN can0 Systemd Service...${NC}"
SERVICE_SRC="${SCRIPT_DIR}/tuinadex-can.service"
SERVICE_DEST="/etc/systemd/system/tuinadex-can.service"

if [ "$1" = "--install-all" ] || [ "$EUID" -eq 0 ]; then
    if [ "$EUID" -ne 0 ]; then
        echo -e "  - Installing ${SERVICE_DEST} via sudo..."
        sudo cp "${SERVICE_SRC}" "${SERVICE_DEST}"
        sudo systemctl daemon-reload
        sudo systemctl enable tuinadex-can.service
    else
        cp "${SERVICE_SRC}" "${SERVICE_DEST}"
        systemctl daemon-reload
        systemctl enable tuinadex-can.service
    fi
    echo -e "  - ${GREEN}tuinadex-can.service enabled for automatic 1Mbps bringup on boot.${NC}"
else
    echo -e "  - Service file available at: ${SERVICE_SRC}"
    echo -e "  - To enable on boot, run: ${YELLOW}sudo bash $0 --install-all${NC}"
fi

# 4. Install Udev Rules
echo -e "\n${BOLD}[4/5] Installing Udev Rules...${NC}"
RULES_SRC="${SCRIPT_DIR}/99-tuinadex-devices.rules"
RULES_DEST="/etc/udev/rules.d/99-tuinadex-devices.rules"

if [ "$1" = "--install-all" ] || [ "$EUID" -eq 0 ]; then
    if [ "$EUID" -ne 0 ]; then
        sudo cp "${RULES_SRC}" "${RULES_DEST}"
        sudo udevadm control --reload-rules && sudo udevadm trigger
    else
        cp "${RULES_SRC}" "${RULES_DEST}"
        udevadm control --reload-rules && udevadm trigger
    fi
    echo -e "  - ${GREEN}Udev rules installed and reloaded.${NC}"
else
    echo -e "  - Udev rules available at: ${RULES_SRC}"
fi

# 5. Core Dependencies Checklist (Humble)
echo -e "\n${BOLD}[5/5] ROS 2 Humble & Motor Dependencies Checklist...${NC}"
echo -e "  Required packages:"
echo -e "    - sudo apt install -y ros-humble-desktop ros-humble-rmw-cyclonedds-cpp ros-humble-pinocchio can-utils"
echo -e "    - python3 -m pip install --user motorbridge"

if command -v python3 >/dev/null 2>&1; then
    if python3 -c "import motorbridge" 2>/dev/null; then
        echo -e "  - ${GREEN}motorbridge Python library is installed.${NC}"
    else
        echo -e "  - ${YELLOW}motorbridge not found in current Python. Run: python3 -m pip install --user motorbridge${NC}"
    fi
fi

echo -e "\n${BOLD}${GREEN}Orange Pi initialization routine completed.${NC}"
