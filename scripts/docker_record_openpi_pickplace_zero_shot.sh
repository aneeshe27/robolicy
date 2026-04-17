#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${IMAGE_TAG:-robolicy/pickplace-noetic:latest}"
OUTPUT_PATH="${1:-${ROOT_DIR}/outputs/pickplace_openpi_zero_shot.mp4}"
WIDTH_VALUE="${WIDTH:-1280}"
HEIGHT_VALUE="${HEIGHT:-720}"
DOCKER_FLAGS=(--rm -i)

if [[ -t 0 && -t 1 ]]; then
  DOCKER_FLAGS=(--rm -it)
fi

if [[ "${OUTPUT_PATH}" != /* ]]; then
  OUTPUT_PATH="${ROOT_DIR}/${OUTPUT_PATH}"
fi

CONTAINER_OUTPUT="/workspace${OUTPUT_PATH#${ROOT_DIR}}"

exec docker run "${DOCKER_FLAGS[@]}" \
  --network host \
  --user "$(id -u):$(id -g)" \
  --volume "${ROOT_DIR}:/workspace" \
  --volume "${ROOT_DIR}/external/pick-and-place/pick_and_place:/opt/robolicy_ws/src/pick_and_place" \
  --volume "${ROOT_DIR}/external/panda_simulator:/opt/robolicy_ws/src/panda_simulator" \
  --workdir /workspace \
  --env HOME=/tmp/robolicy-home \
  --env WIDTH="${WIDTH_VALUE}" \
  --env HEIGHT="${HEIGHT_VALUE}" \
  --env FPS="${FPS:-30}" \
  --env DURATION="${DURATION:-16}" \
  --env BOOT_WAIT="${BOOT_WAIT:-40}" \
  --env CROP_X="${CROP_X:-0}" \
  --env CROP_Y="${CROP_Y:-0}" \
  --env CROP_W="${CROP_W:-${WIDTH_VALUE}}" \
  --env CROP_H="${CROP_H:-${HEIGHT_VALUE}}" \
  --env STACK_SCRIPT="/workspace/scripts/run_openpi_pickplace_zero_shot.sh" \
  --env OPENPI_SERVER_HOST="${OPENPI_SERVER_HOST:-127.0.0.1}" \
  --env OPENPI_SERVER_PORT="${OPENPI_SERVER_PORT:-8000}" \
  --env OPENPI_PROMPT="${OPENPI_PROMPT:-pick up the red cube and place it in the red bin}" \
  --env OPENPI_MAX_STEPS="${OPENPI_MAX_STEPS:-150}" \
  --env OPENPI_OPEN_LOOP_HORIZON="${OPENPI_OPEN_LOOP_HORIZON:-8}" \
  --env OPENPI_MAX_ABS_JOINT_VELOCITY="${OPENPI_MAX_ABS_JOINT_VELOCITY:-0.35}" \
  --env OPENPI_GAZEBO_GUI=true \
  --env OPENPI_GAZEBO_HEADLESS=false \
  "${IMAGE_TAG}" \
  /workspace/scripts/record_external_pickplace.sh "${CONTAINER_OUTPUT}"
