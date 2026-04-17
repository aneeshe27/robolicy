#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${IMAGE_TAG:-robolicy/pickplace-noetic:latest}"
OUTPUT_PATH="${1:-${ROOT_DIR}/outputs/pickplace_demo.mp4}"
DOCKER_FLAGS=(--rm -i)

if [[ -t 0 && -t 1 ]]; then
  DOCKER_FLAGS=(--rm -it)
fi

if [[ "${OUTPUT_PATH}" != /* ]]; then
  OUTPUT_PATH="${ROOT_DIR}/${OUTPUT_PATH}"
fi

CONTAINER_OUTPUT="/workspace${OUTPUT_PATH#${ROOT_DIR}}"

exec docker run "${DOCKER_FLAGS[@]}" \
  --user "$(id -u):$(id -g)" \
  --volume "${ROOT_DIR}:/workspace" \
  --volume "${ROOT_DIR}/external/pick-and-place/pick_and_place:/opt/robolicy_ws/src/pick_and_place" \
  --volume "${ROOT_DIR}/external/panda_simulator:/opt/robolicy_ws/src/panda_simulator" \
  --workdir /workspace \
  --env HOME=/tmp/robolicy-home \
  --env WIDTH="${WIDTH:-1280}" \
  --env HEIGHT="${HEIGHT:-720}" \
  --env FPS="${FPS:-30}" \
  --env DURATION="${DURATION:-16}" \
  --env BOOT_WAIT="${BOOT_WAIT:-45}" \
  --env LIBGL_ALWAYS_SOFTWARE=1 \
  "${IMAGE_TAG}" \
  /workspace/scripts/record_external_pickplace.sh "${CONTAINER_OUTPUT}"
