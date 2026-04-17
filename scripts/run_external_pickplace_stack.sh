#!/usr/bin/env bash
set -euo pipefail

export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

WORLD_WAIT="${WORLD_WAIT:-18}"
MOVEIT_WAIT="${MOVEIT_WAIT:-12}"
DETECTOR_WAIT="${DETECTOR_WAIT:-4}"

PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
  wait >/dev/null 2>&1 || true
}
trap cleanup EXIT

roslaunch pick_and_place panda_world.launch &
PIDS+=($!)
sleep "${WORLD_WAIT}"

roslaunch panda_sim_moveit sim_move_group.launch &
PIDS+=($!)
sleep "${MOVEIT_WAIT}"

rosrun pick_and_place object_detector.py &
PIDS+=($!)
sleep "${DETECTOR_WAIT}"

rosrun pick_and_place pick_and_place_state_machine.py &
PIDS+=($!)

wait
