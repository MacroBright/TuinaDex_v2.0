#!/usr/bin/env bash
# ==============================================================================
# TuinaDex v2.0 - Fast Environment & DDS Activator
# Usage: source scripts/activate_tuinadex.sh [control|vla]
# ==============================================================================

MODE="${1:-control}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Activate CycloneDDS Baseline
source "${SCRIPT_DIR}/env/setup_cyclonedds.sh" >/dev/null 2>&1

# 2. Activate Consolidated Conda Environment
case "${MODE}" in
    control|hw|sim)
        echo "[TD_v2.0] Activating Control Base: tuinadex-control (Python 3.10.20)..."
        conda activate tuinadex-control
        ;;
    vla|train|learning)
        echo "[TD_v2.0] Activating VLA & Learning Base: tuinadex-vla (Python 3.10.20 + CUDA)..."
        conda activate tuinadex-vla
        ;;
    *)
        echo "[TD_v2.0] Unknown mode: ${MODE}. Available options: 'control' or 'vla'"
        return 1 2>/dev/null || exit 1
        ;;
esac

echo "[TD_v2.0] Ready. (ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, RMW=${RMW_IMPLEMENTATION})"

