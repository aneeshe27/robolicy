#!/usr/bin/env bash
set -euo pipefail

export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

WORLD_WAIT="${WORLD_WAIT:-18}"
PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
  wait >/dev/null 2>&1 || true
}
trap cleanup EXIT

roslaunch pick_and_place panda_world.launch \
  world:="${OPENPI_WORLD:-/workspace/external/pick-and-place/pick_and_place/worlds/pick_and_place_openpi.world}" \
  gui:="${OPENPI_GAZEBO_GUI:-false}" \
  headless:="${OPENPI_GAZEBO_HEADLESS:-true}" &
PIDS+=($!)
sleep "${WORLD_WAIT}"

python3 /workspace/scripts/openpi_pickplace_zero_shot.py \
  --server-host "${OPENPI_SERVER_HOST:-127.0.0.1}" \
  --server-port "${OPENPI_SERVER_PORT:-8000}" \
  --prompt "${OPENPI_PROMPT:-pick up the red cube and place it in the red bin}" \
  --steps "${OPENPI_MAX_STEPS:-150}" \
  --open-loop-horizon "${OPENPI_OPEN_LOOP_HORIZON:-8}" \
  --max-abs-joint-velocity "${OPENPI_MAX_ABS_JOINT_VELOCITY:-0.35}" \
  "$@"
