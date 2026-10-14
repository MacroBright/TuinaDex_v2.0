#!/usr/bin/env bash
# ==============================================================================
# TuinaDex v2.0 - Holistic Multi-Environment Health & Verification Suite
# Runs automated environment unit tests and diagnostic hardware/ROS check.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ANSI Colors
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
BLUE="\033[0;34m"
BOLD="\033[1m"
NC="\033[0m"

echo -e "${BOLD}${BLUE}======================================================${NC}"
echo -e "${BOLD}${BLUE}  TuinaDex v2.0 - Environment Holistic Verification   ${NC}"
echo -e "${BOLD}${BLUE}======================================================${NC}"

# 1. Run Automated Environment Tests
echo -e "\n${BOLD}[Step 1/2] Running automated environment test suite...${NC}"
PY_BIN="${HOME}/win_office/conda/envs/leap_hand/bin/python"
if [ ! -f "${PY_BIN}" ]; then
    PY_BIN="python3"
fi

if "${PY_BIN}" -m pytest "${REPO_ROOT}/tests/test_env_management.py" -v; then
    echo -e "${GREEN}[PASS] All environment scripts, rules, and Dockerfiles verified.${NC}"
else
    echo -e "${RED}[FAIL] Environment test suite failed!${NC}"
    exit 1
fi

# 2. Run Local Diagnostics
echo -e "\n${BOLD}[Step 2/2] Running system environment diagnostic report...${NC}"
bash "${REPO_ROOT}/scripts/env/check_env.sh"

echo -e "\n${BOLD}${GREEN}======================================================${NC}"
echo -e "${BOLD}${GREEN}  All TuinaDex v2.0 Environment Checks Passed (OK)    ${NC}"
echo -e "${BOLD}${GREEN}======================================================${NC}"
exit 0
