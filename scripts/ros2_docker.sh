#!/usr/bin/env bash
# Run TuinaDex ROS2 commands in the official Ubuntu 26.04 / ROS2 Lyrical image.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKER_ARGS=(
  run --rm --network=host --ipc=host
  --user "$(id -u):$(id -g)"
  --env HOME=/tmp
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
  --volume "$ROOT:/workspace"
  --workdir /workspace
)

if [[ -e /dev/ttyUSB0 ]]; then
  DOCKER_ARGS+=(--device=/dev/ttyUSB0)
  DIALOUT_GID="$(getent group dialout | cut -d: -f3)"
  if [[ -n "$DIALOUT_GID" ]]; then
    DOCKER_ARGS+=(--group-add "$DIALOUT_GID")
  fi
fi

if (( "$#" == 0 )); then
  set -- bash
fi

exec docker "${DOCKER_ARGS[@]}" ros:lyrical bash -lc \
  'source /opt/ros/lyrical/setup.bash; exec "$@"' _ "$@"
