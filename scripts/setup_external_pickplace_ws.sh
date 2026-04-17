#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${1:-${ROOT_DIR}/external/noetic_ws}"
SRC_DIR="${WS_DIR}/src"
RUN_DEPS=0

if [[ "${1:-}" == "--deps" ]]; then
  WS_DIR="${ROOT_DIR}/external/noetic_ws"
  RUN_DEPS=1
elif [[ "${2:-}" == "--deps" ]]; then
  RUN_DEPS=1
fi

mkdir -p "${SRC_DIR}"

ln -sfn "${ROOT_DIR}/external/panda_simulator" "${SRC_DIR}/panda_simulator"
ln -sfn "${ROOT_DIR}/external/pick-and-place/pick_and_place" "${SRC_DIR}/pick_and_place"

echo "Workspace linked at ${WS_DIR}"
echo "  - panda_simulator -> ${SRC_DIR}/panda_simulator"
echo "  - pick_and_place -> ${SRC_DIR}/pick_and_place"

if [[ "${RUN_DEPS}" -eq 0 ]]; then
  exit 0
fi

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ROS Noetic was not found at /opt/ros/noetic/setup.bash." >&2
  echo "The workspace links were created, but dependency setup was skipped." >&2
  exit 1
fi

if ! command -v wstool >/dev/null 2>&1; then
  echo "wstool is not installed. Install ROS Noetic workspace tools first." >&2
  exit 1
fi

if ! command -v rosdep >/dev/null 2>&1; then
  echo "rosdep is not installed. Install ROS Noetic workspace tools first." >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash

pushd "${SRC_DIR}" >/dev/null
if [[ ! -f .rosinstall ]]; then
  wstool init .
fi
wstool merge panda_simulator/dependencies.rosinstall
wstool up
rosdep install -y --from-paths . --ignore-src --rosdistro noetic --skip-keys python-sip
popd >/dev/null

echo "Dependencies prepared for ${WS_DIR}"
