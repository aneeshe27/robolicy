#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${1:-${ROOT_DIR}/external/noetic_ws}"

"${ROOT_DIR}/scripts/setup_external_pickplace_ws.sh" "${WS_DIR}" --deps

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ROS Noetic was not found at /opt/ros/noetic/setup.bash." >&2
  exit 1
fi

if ! command -v catkin >/dev/null 2>&1; then
  echo "catkin is not installed. Install catkin tools first." >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
cd "${WS_DIR}"
catkin build
