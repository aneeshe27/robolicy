#!/usr/bin/env bash
# Launch Gazebo + MoveIt and then run the scripted (ground-truth) pick-and-place.
# No VLA, no policy server. Just classical IK + Cartesian planning via
# ``panda_robot.PandaArm.move_to_cartesian_pose``.
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
  world:="${PICKPLACE_WORLD:-/workspace/external/pick-and-place/pick_and_place/worlds/pick_and_place_openpi.world}" \
  gui:="${PICKPLACE_GAZEBO_GUI:-false}" \
  headless:="${PICKPLACE_GAZEBO_HEADLESS:-true}" &
PIDS+=($!)
sleep "${WORLD_WAIT}"

# No MoveIt: we use PandaArm.inverse_kinematics (KDL) + move_to_joint_position
# with use_moveit=False, which drives the panda_simulator JointTrajectory
# action server directly.
python3 /workspace/scripts/pickplace_scripted_groundtruth.py \
  --cube-model "${PICKPLACE_CUBE_MODEL:-block_red_target}" \
  --bin-model "${PICKPLACE_BIN_MODEL:-bin_red}" \
  --cube-height "${PICKPLACE_CUBE_HEIGHT:-0.03}" \
  --fingertip-z-extra "${PICKPLACE_FINGERTIP_Z_EXTRA:-0.045}" \
  --grasp-depth "${PICKPLACE_GRASP_DEPTH:-0.015}" \
  --pre-pick-height "${PICKPLACE_PRE_PICK_HEIGHT:-0.25}" \
  --place-hover-height "${PICKPLACE_PLACE_HOVER_HEIGHT:-0.30}" \
  --release-height "${PICKPLACE_RELEASE_HEIGHT:-0.24}" \
  --travel-height "${PICKPLACE_TRAVEL_HEIGHT:-0.55}" \
  --joint-speed "${PICKPLACE_JOINT_SPEED:-0.08}" \
  --grasp-width "${PICKPLACE_GRASP_WIDTH:-0.0}" \
  --grasp-force "${PICKPLACE_GRASP_FORCE:-80.0}" \
  --grasp-epsilon-inner "${PICKPLACE_GRASP_EPS_IN:-0.005}" \
  --grasp-epsilon-outer "${PICKPLACE_GRASP_EPS_OUT:-0.06}" \
  --grasp-settle-time "${PICKPLACE_GRASP_SETTLE:-1.5}" \
  --attempts "${PICKPLACE_ATTEMPTS:-1}" \
  "$@"
