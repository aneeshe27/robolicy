#!/usr/bin/env bash
# Docker wrapper: runs the scripted ground-truth pick-and-place inside the
# robolicy/pickplace-noetic container and records the exterior + wrist
# cameras into an mp4. No VLA involved.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${IMAGE_TAG:-robolicy/pickplace-noetic:latest}"
OUTPUT_PATH="${1:-${ROOT_DIR}/outputs/pickplace_scripted.mp4}"
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

# DURATION=0 lets the recorder stop when the scripted rollout finishes.
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
  --env RECORDER_STARTUP_TIMEOUT="${RECORDER_STARTUP_TIMEOUT:-120}" \
  --env EXTERIOR_TOPIC="${EXTERIOR_TOPIC:-/camera/color/image_raw}" \
  --env WRIST_TOPIC="${WRIST_TOPIC:-/wrist_camera/color/image_raw}" \
  --env STACK_SCRIPT="/workspace/scripts/run_pickplace_scripted.sh" \
  --env RECORDER_SCRIPT="/workspace/scripts/record_openpi_rollout_topics.py" \
  --env OPENPI_PROMPT="${OPENPI_PROMPT:-scripted: pick red cube, place in red bin}" \
  --env PICKPLACE_CUBE_MODEL="${PICKPLACE_CUBE_MODEL:-block_red_target}" \
  --env PICKPLACE_BIN_MODEL="${PICKPLACE_BIN_MODEL:-bin_red}" \
  --env PICKPLACE_CUBE_HEIGHT="${PICKPLACE_CUBE_HEIGHT:-0.03}" \
  --env PICKPLACE_FINGERTIP_Z_EXTRA="${PICKPLACE_FINGERTIP_Z_EXTRA:-0.045}" \
  --env PICKPLACE_GRASP_DEPTH="${PICKPLACE_GRASP_DEPTH:-0.015}" \
  --env PICKPLACE_PRE_PICK_HEIGHT="${PICKPLACE_PRE_PICK_HEIGHT:-0.25}" \
  --env PICKPLACE_PLACE_HOVER_HEIGHT="${PICKPLACE_PLACE_HOVER_HEIGHT:-0.30}" \
  --env PICKPLACE_RELEASE_HEIGHT="${PICKPLACE_RELEASE_HEIGHT:-0.24}" \
  --env PICKPLACE_TRAVEL_HEIGHT="${PICKPLACE_TRAVEL_HEIGHT:-0.55}" \
  --env PICKPLACE_JOINT_SPEED="${PICKPLACE_JOINT_SPEED:-0.08}" \
  --env PICKPLACE_GRASP_WIDTH="${PICKPLACE_GRASP_WIDTH:-0.0}" \
  --env PICKPLACE_GRASP_FORCE="${PICKPLACE_GRASP_FORCE:-80.0}" \
  --env PICKPLACE_GRASP_EPS_IN="${PICKPLACE_GRASP_EPS_IN:-0.005}" \
  --env PICKPLACE_GRASP_EPS_OUT="${PICKPLACE_GRASP_EPS_OUT:-0.06}" \
  --env PICKPLACE_GRASP_SETTLE="${PICKPLACE_GRASP_SETTLE:-1.5}" \
  --env PICKPLACE_ATTEMPTS="${PICKPLACE_ATTEMPTS:-1}" \
  "${IMAGE_TAG}" \
  /workspace/scripts/record_openpi_pickplace_topics.sh "${CONTAINER_OUTPUT}"
