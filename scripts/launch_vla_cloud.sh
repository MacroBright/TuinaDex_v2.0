#!/usr/bin/env bash
# ==============================================================================
# TuinaDex v2.0 - Cloud VLA Inference & Training Service Runner (TD_v2.0-ENV-CLOUD)
# Launches high-compute GPU container with gRPC port 50051 mapping.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="tuinadex-vla:latest"
DOCKERFILE_PATH="${REPO_ROOT}/docker/vla_cloud.Dockerfile"
PORT=50051

# ANSI Colors
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
BLUE="\033[0;34m"
BOLD="\033[1m"
NC="\033[0m"

# Option 1: Build Image
if [ "$1" = "--build" ]; then
    echo -e "${BOLD}${BLUE}[TD_v2.0-CLOUD] Building Docker image ${IMAGE_NAME}...${NC}"
    docker build -t "${IMAGE_NAME}" -f "${DOCKERFILE_PATH}" "${REPO_ROOT}"
    echo -e "${GREEN}[TD_v2.0-CLOUD] Build complete.${NC}"
    exit 0
fi

# Ensure Image Exists
if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    echo -e "${YELLOW}[TD_v2.0-CLOUD] Image ${IMAGE_NAME} not found. Building now...${NC}"
    docker build -t "${IMAGE_NAME}" -f "${DOCKERFILE_PATH}" "${REPO_ROOT}"
fi

# Check GPU support flag
GPU_FLAG="--gpus all"
if ! command -v nvidia-smi >/dev/null 2>&1; then
    echo -e "${YELLOW}[TD_v2.0-CLOUD] Warning: nvidia-smi not detected. Falling back to CPU mode.${NC}"
    GPU_FLAG=""
fi

mkdir -p "${REPO_ROOT}/models"
mkdir -p "${REPO_ROOT}/dataset"

echo -e "${BOLD}${BLUE}[TD_v2.0-CLOUD] Launching VLA Service Container on port ${PORT}...${NC}"
echo "  - Models dir:  ${REPO_ROOT}/models -> /workspace/models"
echo "  - Dataset dir: ${REPO_ROOT}/dataset -> /workspace/dataset"
echo "  - Port:        ${PORT} (gRPC Action Chunking stream)"

if [ $# -gt 0 ]; then
    docker run --rm -it \
        ${GPU_FLAG} \
        -p "${PORT}:${PORT}" \
        -v "${REPO_ROOT}/models:/workspace/models" \
        -v "${REPO_ROOT}/dataset:/workspace/dataset" \
        -v "${REPO_ROOT}/src:/workspace/src" \
        -w /workspace \
        "${IMAGE_NAME}" "$@"
else
    docker run --rm -it \
        ${GPU_FLAG} \
        -p "${PORT}:${PORT}" \
        -v "${REPO_ROOT}/models:/workspace/models" \
        -v "${REPO_ROOT}/dataset:/workspace/dataset" \
        -v "${REPO_ROOT}/src:/workspace/src" \
        -w /workspace \
        "${IMAGE_NAME}"
fi
