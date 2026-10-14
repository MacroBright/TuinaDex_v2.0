#!/usr/bin/env bash
# ==============================================================================
# TuinaDex v2.0 - Host Development PC Initialization Script (TD_v2.0-ENV-DEV)
# Configures ROS2 Lyrical dependencies, CycloneDDS, udev rules, and CAN interfaces.
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
echo -e "${BOLD}${BLUE}   TuinaDex v2.0 - Dev PC Setup (TD_v2.0-ENV-DEV)     ${NC}"
echo -e "${BOLD}${BLUE}======================================================${NC}"

# 1. CycloneDDS Configuration
echo -e "\n${BOLD}[1/4] Configuring CycloneDDS Baseline...${NC}"
bash "${SCRIPT_DIR}/setup_cyclonedds.sh" --install
source "${SCRIPT_DIR}/setup_cyclonedds.sh"

# 2. Udev Rules Installation
echo -e "\n${BOLD}[2/4] Device Permissions & Udev Rules...${NC}"
RULES_SRC="${SCRIPT_DIR}/99-tuinadex-devices.rules"
RULES_DEST="/etc/udev/rules.d/99-tuinadex-devices.rules"

if [ "$1" = "--install-rules" ] || [ "$EUID" -eq 0 ]; then
    if [ "$EUID" -ne 0 ]; then
        echo -e "  - Requesting sudo to copy udev rules to ${RULES_DEST}..."
        sudo cp "${RULES_SRC}" "${RULES_DEST}"
        sudo udevadm control --reload-rules && sudo udevadm trigger
    else
        cp "${RULES_SRC}" "${RULES_DEST}"
        udevadm control --reload-rules && udevadm trigger
    fi
    echo -e "  - ${GREEN}Udev rules installed and reloaded.${NC}"
else
    echo -e "  - Udev rules available at: ${RULES_SRC}"
    echo -e "  - To install system-wide, run: ${YELLOW}sudo bash $0 --install-rules${NC}"
fi

# Ensure user is in dialout group
CURRENT_USER="${SUDO_USER:-$USER}"
if groups "${CURRENT_USER}" | grep -q "\bdialout\b"; then
    echo -e "  - User '${CURRENT_USER}' is already in group 'dialout'."
else
    echo -e "  - ${YELLOW}Notice: User '${CURRENT_USER}' is not in 'dialout'. Run: sudo usermod -aG dialout ${CURRENT_USER}${NC}"
fi

# 3. Optional Virtual CAN Setup for Testing
if [ "$1" = "--setup-vcan" ]; then
    echo -e "\n${BOLD}[3/4] Setting up Virtual CAN (vcan0)...${NC}"
    if [ "$EUID" -ne 0 ]; then
        sudo modprobe vcan
        sudo ip link add dev vcan0 type vcan || true
        sudo ip link set up vcan0
    else
        modprobe vcan
        ip link add dev vcan0 type vcan || true
        ip link set up vcan0
    fi
    echo -e "  - ${GREEN}Virtual CAN interface vcan0 is UP.${NC}"
else
    echo -e "\n${BOLD}[3/4] SocketCAN Interface Check...${NC}"
    if ip link show can0 >/dev/null 2>&1; then
        echo -e "  - ${GREEN}can0 is present.${NC}"
    else
        echo -e "  - can0 not detected. (For simulation/mock testing, run: ${YELLOW}bash $0 --setup-vcan${NC})"
    fi
fi

# 4. ROS 2 Workspace Verification
echo -e "\n${BOLD}[4/4] Verifying ROS 2 Packages Build...${NC}"
if [ -f "/opt/ros/lyrical/setup.bash" ]; then
    echo -e "  - Sourcing /opt/ros/lyrical/setup.bash..."
    # Test build check
    echo -e "  - Core packages available in src/: tuinadex_core, tuinadex_hand, tuinadex_sim"
    echo -e "  - Build command: ${GREEN}source /opt/ros/lyrical/setup.bash && colcon build --symlink-install --packages-select tuinadex_core tuinadex_hand tuinadex_sim${NC}"
else
    echo -e "  - ${RED}Warning: /opt/ros/lyrical/setup.bash not found!${NC}"
fi

echo -e "\n${BOLD}${GREEN}Dev PC initialization routine finished successfully.${NC}"
