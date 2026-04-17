#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${IMAGE_TAG:-robolicy/openpi-server:latest}"

"${ROOT_DIR}/scripts/apply_external_demo_overlays.sh"

docker build \
  -f "${ROOT_DIR}/external/openpi/scripts/docker/serve_policy.Dockerfile" \
  -t "${IMAGE_TAG}" \
  "${ROOT_DIR}/external/openpi"
