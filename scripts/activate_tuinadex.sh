#!/usr/bin/env bash
# ==============================================================================
# TuinaDex v2.0 - Fast Environment & DDS Activator
# Usage: source scripts/activate_tuinadex.sh [control|vla]
# ==============================================================================

MODE="${1:-control}"
_TD_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_TD_WORKSPACE_ROOT="$(cd "${_TD_SCRIPT_DIR}/.." && pwd)"

# 1. Activate CycloneDDS Baseline
source "${_TD_SCRIPT_DIR}/env/setup_cyclonedds.sh" >/dev/null 2>&1

# Ensure conda function is available in subshells
if ! declare -f conda >/dev/null 2>&1; then
    if [[ -f "/home/bright/miniconda3/etc/profile.d/conda.sh" ]]; then
        # shellcheck source=/dev/null
        source "/home/bright/miniconda3/etc/profile.d/conda.sh"
    elif [[ -n "${CONDA_EXE:-}" && -f "${CONDA_EXE%/bin/conda}/etc/profile.d/conda.sh" ]]; then
        # shellcheck source=/dev/null
        source "${CONDA_EXE%/bin/conda}/etc/profile.d/conda.sh"
    fi
fi

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

# 3. Source workspace overlay if built
if [[ -f "${_TD_WORKSPACE_ROOT}/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${_TD_WORKSPACE_ROOT}/install/setup.bash"
fi
unset _TD_SCRIPT_DIR _TD_WORKSPACE_ROOT

# 4. Native library precedence: ROS Lyrical ships Python 3.14 libeigenpy /
#    boost_python314. A Python 3.10 pinocchio must resolve its bundled cmeel
#    libs first, otherwise `import pinocchio` dies with an undefined symbol.
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    _td_cmeel_lib="${CONDA_PREFIX}/lib/python3.10/site-packages/cmeel.prefix/lib"
    if [[ -d "${_td_cmeel_lib}" ]]; then
        export LD_LIBRARY_PATH="${_td_cmeel_lib}:${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    fi
    unset _td_cmeel_lib
fi

echo "[TD_v2.0] Ready. (ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, RMW=${RMW_IMPLEMENTATION})"

