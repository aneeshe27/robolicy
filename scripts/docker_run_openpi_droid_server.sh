#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${IMAGE_TAG:-robolicy/openpi-server:latest}"
CACHE_DIR="${OPENPI_CACHE_DIR:-${ROOT_DIR}/.cache/openpi}"
PORT="${OPENPI_SERVER_PORT:-8000}"
SERVER_ARGS="${SERVER_ARGS:---env DROID --port ${PORT}}"
DOCKER_FLAGS=(--rm -i)

if [[ -t 0 && -t 1 ]]; then
  DOCKER_FLAGS=(--rm -it)
fi

mkdir -p "${CACHE_DIR}"

exec docker run "${DOCKER_FLAGS[@]}" \
  --network host \
  --gpus "${OPENPI_GPUS:-all}" \
  --ipc host \
  --volume "${ROOT_DIR}/external/openpi:/app" \
  --volume "${CACHE_DIR}:/openpi_assets" \
  --workdir /app \
  --env OPENPI_DATA_HOME=/openpi_assets \
  --env SERVER_ARGS="${SERVER_ARGS}" \
  "${IMAGE_TAG}"
