#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXTERNAL_DIR="${ROOT_DIR}/external"

clone_if_missing() {
  local repo_url="$1"
  local dest_dir="$2"
  local commit="$3"

  if [[ -d "${dest_dir}/.git" ]]; then
    echo "Keeping existing repo: ${dest_dir}"
    return
  fi

  rm -rf "${dest_dir}"
  git clone "${repo_url}" "${dest_dir}"
  git -C "${dest_dir}" checkout "${commit}"
}

mkdir -p "${EXTERNAL_DIR}"

clone_if_missing \
  "https://github.com/justagist/panda_simulator.git" \
  "${EXTERNAL_DIR}/panda_simulator" \
  "6cec91d"

clone_if_missing \
  "https://github.com/elena-ecn/pick-and-place.git" \
  "${EXTERNAL_DIR}/pick-and-place" \
  "a4c3b22"

clone_if_missing \
  "https://github.com/Physical-Intelligence/openpi.git" \
  "${EXTERNAL_DIR}/openpi" \
  "650c5b0"

cd "${ROOT_DIR}"
./scripts/apply_external_demo_overlays.sh

echo "External repos are ready under ${EXTERNAL_DIR}"
