# ==============================================================================
# TuinaDex v2.0 - Cloud VLA Inference & Training Container (TD_v2.0-ENV-CLOUD)
# CUDA 12.4 + PyTorch 2.4 + LeRobot + gRPC/Zenoh Action Chunking Server
# Specification: docs/superpowers/specs/2026-10-14-environment-management-architecture-design.md
# ==============================================================================
FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install base packages
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

# Install PyTorch with CUDA 12.4 support
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
    torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/cu124

# Install LeRobot, Transformers, gRPC, and Action Chunking dependencies
RUN pip3 install --no-cache-dir \
    lerobot==0.4.4 \
    transformers \
    accelerate \
    grpcio \
    grpcio-tools \
    protobuf \
    safetensors \
    scipy \
    "numpy<2.0.0" \
    tqdm \
    huggingface_hub

WORKDIR /workspace

# gRPC streaming port for Action Chunking
EXPOSE 50051

# Default CMD: print GPU and PyTorch status
CMD ["python3", "-c", "import torch; print('TuinaDex VLA Cloud Service Ready | CUDA:', torch.cuda.is_available(), '| Devices:', torch.cuda.device_count())"]
