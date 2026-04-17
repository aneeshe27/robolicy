#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${IMAGE_TAG:-robolicy/pickplace-noetic:latest}"
DOCKER_FLAGS=(--rm -i)

if [[ -t 0 && -t 1 ]]; then
  DOCKER_FLAGS=(--rm -it)
fi

exec docker run "${DOCKER_FLAGS[@]}" \
  --network host \
  --user "$(id -u):$(id -g)" \
  --volume "${ROOT_DIR}:/workspace" \
  --volume "${ROOT_DIR}/external/pick-and-place/pick_and_place:/opt/robolicy_ws/src/pick_and_place" \
  --volume "${ROOT_DIR}/external/panda_simulator:/opt/robolicy_ws/src/panda_simulator" \
  --workdir /workspace \
  --env HOME=/tmp/robolicy-home \
  --env OPENPI_SERVER_HOST="${OPENPI_SERVER_HOST:-127.0.0.1}" \
  --env OPENPI_SERVER_PORT="${OPENPI_SERVER_PORT:-8000}" \
  --env OPENPI_PROMPT="${OPENPI_PROMPT:-pick up the red cube and place it in the red bin}" \
  --env OPENPI_MAX_STEPS="${OPENPI_MAX_STEPS:-150}" \
  --env OPENPI_OPEN_LOOP_HORIZON="${OPENPI_OPEN_LOOP_HORIZON:-8}" \
  --env OPENPI_MAX_ABS_JOINT_VELOCITY="${OPENPI_MAX_ABS_JOINT_VELOCITY:-0.35}" \
  "${IMAGE_TAG}" \
  bash -lc 'source /opt/ros/noetic/setup.bash && source /opt/robolicy_ws/devel/setup.bash && /workspace/scripts/run_openpi_pickplace_zero_shot.sh "$@"' _ "$@"
