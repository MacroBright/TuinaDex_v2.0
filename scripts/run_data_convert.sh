#!/usr/bin/env bash
# ==============================================================================
# TuinaDex v2.0 - LeRobot Dataset Conversion Runner (TD_v2.0-ENV-DATA)
# Executes rosbag2 -> LeRobotDataset conversion inside isolated Docker container.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="tuinadex-data:v2.0"
DOCKERFILE_PATH="${REPO_ROOT}/docker/data.Dockerfile"

# ANSI Colors
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
BLUE="\033[0;34m"
BOLD="\033[1m"
NC="\033[0m"

# Option 1: Build Image
if [ "$1" = "--build" ]; then
    echo -e "${BOLD}${BLUE}[TD_v2.0-DATA] Building Docker image ${IMAGE_NAME}...${NC}"
    docker build -t "${IMAGE_NAME}" -f "${DOCKERFILE_PATH}" "${REPO_ROOT}"
    echo -e "${GREEN}[TD_v2.0-DATA] Build complete.${NC}"
    exit 0
fi

# Ensure Image Exists
if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    echo -e "${YELLOW}[TD_v2.0-DATA] Image ${IMAGE_NAME} not found. Building now...${NC}"
    docker build -t "${IMAGE_NAME}" -f "${DOCKERFILE_PATH}" "${REPO_ROOT}"
fi

# Ensure dataset directory exists
mkdir -p "${REPO_ROOT}/dataset"

echo -e "${BOLD}${BLUE}[TD_v2.0-DATA] Launching container with volume mounts...${NC}"
echo "  - Local dataset: ${REPO_ROOT}/dataset -> /workspace/dataset"
echo "  - Local source:  ${REPO_ROOT}/src -> /workspace/src"

if [ $# -gt 0 ]; then
    docker run --rm -it \
        -v "${REPO_ROOT}/dataset:/workspace/dataset" \
        -v "${REPO_ROOT}/src:/workspace/src" \
        -w /workspace \
        "${IMAGE_NAME}" "$@"
else
    docker run --rm -it \
        -v "${REPO_ROOT}/dataset:/workspace/dataset" \
        -v "${REPO_ROOT}/src:/workspace/src" \
        -w /workspace \
        "${IMAGE_NAME}" python3 -c "import lerobot; print('TuinaDex Data Converter Ready. LeRobot version:', lerobot.__version__)"
fi
