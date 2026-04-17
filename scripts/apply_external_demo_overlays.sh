#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

copy_if_repo_exists() {
  local src="$1"
  local dst="$2"

  mkdir -p "$(dirname "${dst}")"
  cp "${src}" "${dst}"
}

if [[ -d "${ROOT_DIR}/external/pick-and-place" ]]; then
  copy_if_repo_exists \
    "${ROOT_DIR}/overlays/pick_and_place/launch/panda_world.launch" \
    "${ROOT_DIR}/external/pick-and-place/pick_and_place/launch/panda_world.launch"
  copy_if_repo_exists \
    "${ROOT_DIR}/overlays/pick_and_place/worlds/pick_and_place_openpi.world" \
    "${ROOT_DIR}/external/pick-and-place/pick_and_place/worlds/pick_and_place_openpi.world"
  copy_if_repo_exists \
    "${ROOT_DIR}/overlays/pick_and_place/robots/panda_openpi_cameras.urdf.xacro" \
    "${ROOT_DIR}/external/pick-and-place/pick_and_place/robots/panda_openpi_cameras.urdf.xacro"
fi

if [[ -d "${ROOT_DIR}/external/openpi" ]]; then
  copy_if_repo_exists \
    "${ROOT_DIR}/overlays/openpi/serve_policy.Dockerfile" \
    "${ROOT_DIR}/external/openpi/scripts/docker/serve_policy.Dockerfile"
fi

echo "External overlays applied."
