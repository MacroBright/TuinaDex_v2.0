# ==============================================================================
# TuinaDex v2.0 - Dataset Cleaning & Conversion Container (TD_v2.0-ENV-DATA)
# Standardizes Python 3.10, PyTorch, LeRobot, rosbags, and ffmpeg
# Specification: docs/superpowers/specs/2026-10-14-environment-management-architecture-design.md
# ==============================================================================
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install base utilities and Python 3.10
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    git \
    curl \
    ffmpeg \
    build-essential \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install PyTorch CPU baseline (deterministic for conversion)
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
    torch==2.2.2 \
    torchvision==0.17.2 \
    --index-url https://download.pytorch.org/whl/cpu

# Install LeRobot & dataset conversion dependencies
RUN pip3 install --no-cache-dir \
    lerobot==0.4.4 \
    rosbags \
    safetensors \
    scipy \
    "numpy<2.0.0" \
    tqdm \
    huggingface_hub

WORKDIR /workspace

# Default command: display LeRobot version and usage
CMD ["python3", "-c", "import lerobot; print('TuinaDex Data Converter - LeRobot v' + lerobot.__version__)"]
