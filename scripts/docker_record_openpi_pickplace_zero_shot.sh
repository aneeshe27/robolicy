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

# DURATION=0 means "record until the pi0 bridge rollout exits", which pairs
# naturally with OPENPI_MAX_STEPS. If the caller sets DURATION>0, the
# recorder stops after that many seconds even if the rollout is still going.
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
  --env FPS="${FPS:-20}" \
  --env DURATION="${DURATION:-0}" \
  --env BOOT_WAIT="${BOOT_WAIT:-3}" \
  --env TILE_WIDTH="${TILE_WIDTH:-640}" \
  --env TILE_HEIGHT="${TILE_HEIGHT:-480}" \
  --env RECORDER_STARTUP_TIMEOUT="${RECORDER_STARTUP_TIMEOUT:-90}" \
  --env EXTERIOR_TOPIC="${EXTERIOR_TOPIC:-/camera/color/image_raw}" \
  --env WRIST_TOPIC="${WRIST_TOPIC:-/wrist_camera/color/image_raw}" \
  --env STACK_SCRIPT="/workspace/scripts/run_openpi_pickplace_zero_shot.sh" \
  --env RECORDER_SCRIPT="/workspace/scripts/record_openpi_rollout_topics.py" \
  --env OPENPI_SERVER_HOST="${OPENPI_SERVER_HOST:-127.0.0.1}" \
  --env OPENPI_SERVER_PORT="${OPENPI_SERVER_PORT:-8765}" \
  --env OPENPI_PROMPT="${OPENPI_PROMPT:-pick up the red cube and place it in the red bin}" \
  --env OPENPI_MAX_STEPS="${OPENPI_MAX_STEPS:-150}" \
  --env OPENPI_OPEN_LOOP_HORIZON="${OPENPI_OPEN_LOOP_HORIZON:-8}" \
  --env OPENPI_ACTION_MODE="${OPENPI_ACTION_MODE:-velocity}" \
  --env OPENPI_MAX_ABS_JOINT_VELOCITY="${OPENPI_MAX_ABS_JOINT_VELOCITY:-1.2}" \
  --env OPENPI_GRIPPER_MODE="${OPENPI_GRIPPER_MODE:-continuous}" \
  "${IMAGE_TAG}" \
  /workspace/scripts/record_openpi_pickplace_topics.sh "${CONTAINER_OUTPUT}"
