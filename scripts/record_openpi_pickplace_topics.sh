#!/usr/bin/env bash
# Headless recorder for the openpi pick-and-place rollout that captures the
# exterior + wrist ROS image topics (not the gzclient GUI) into an mp4.
#
# Xvfb is still started because Gazebo's camera sensors need an OpenGL
# context, but gzclient is not launched and x11grab is not used. The video
# comes straight from the ROS topics the bridge is already consuming.
set -euo pipefail

OUTPUT_PATH="${1:-/workspace/outputs/pickplace_openpi_zero_shot.mp4}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-20}"
DURATION="${DURATION:-0}"
DISPLAY_NUM="${DISPLAY_NUM:-:99}"
BOOT_WAIT="${BOOT_WAIT:-3}"
STACK_LOG="${STACK_LOG:-/workspace/outputs/pickplace_record_stack.log}"
XVFB_LOG="${XVFB_LOG:-/tmp/pickplace_xvfb.log}"
RECORDER_STARTUP_TIMEOUT="${RECORDER_STARTUP_TIMEOUT:-90}"
TILE_WIDTH="${TILE_WIDTH:-640}"
TILE_HEIGHT="${TILE_HEIGHT:-480}"
STACK_SCRIPT="${STACK_SCRIPT:-/workspace/scripts/run_openpi_pickplace_zero_shot.sh}"
RECORDER_SCRIPT="${RECORDER_SCRIPT:-/workspace/scripts/record_openpi_rollout_topics.py}"
EXTERIOR_TOPIC="${EXTERIOR_TOPIC:-/camera/color/image_raw}"
WRIST_TOPIC="${WRIST_TOPIC:-/wrist_camera/color/image_raw}"
OPENPI_PROMPT_BANNER="${OPENPI_PROMPT:-pick up the red cube and place it in the red bin}"

# Force headless Gazebo; we no longer capture the GUI.
export OPENPI_GAZEBO_GUI="false"
export OPENPI_GAZEBO_HEADLESS="true"

mkdir -p "$(dirname "${OUTPUT_PATH}")"

RECORDER_PID=""
STACK_PID=""
XVFB_PID=""

cleanup() {
  local exit_code=$?
  if [[ -n "${RECORDER_PID}" ]]; then
    kill -TERM "${RECORDER_PID}" >/dev/null 2>&1 || true
    wait "${RECORDER_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${STACK_PID}" ]]; then
    kill -TERM "${STACK_PID}" >/dev/null 2>&1 || true
    wait "${STACK_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${XVFB_PID}" ]]; then
    kill -TERM "${XVFB_PID}" >/dev/null 2>&1 || true
    wait "${XVFB_PID}" >/dev/null 2>&1 || true
  fi
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

Xvfb "${DISPLAY_NUM}" -screen 0 "${WIDTH}x${HEIGHT}x24" -ac +extension GLX +render -noreset \
  >"${XVFB_LOG}" 2>&1 &
XVFB_PID=$!
sleep 2

export DISPLAY="${DISPLAY_NUM}"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1

# Launch Gazebo + pi0 bridge in the background. The bridge's lifetime is
# controlled by OPENPI_MAX_STEPS (set by the caller / docker wrapper).
mkdir -p "$(dirname "${STACK_LOG}")"
"${STACK_SCRIPT}" >"${STACK_LOG}" 2>&1 &
STACK_PID=$!

# The recorder itself blocks until the exterior camera topic is publishing,
# so BOOT_WAIT only needs to give Xvfb and roslaunch a brief head start.
sleep "${BOOT_WAIT}"

RECORDER_ARGS=(
  --output "${OUTPUT_PATH}"
  --exterior-topic "${EXTERIOR_TOPIC}"
  --wrist-topic "${WRIST_TOPIC}"
  --fps "${FPS}"
  --tile-width "${TILE_WIDTH}"
  --tile-height "${TILE_HEIGHT}"
  --startup-timeout "${RECORDER_STARTUP_TIMEOUT}"
  --prompt "${OPENPI_PROMPT_BANNER}"
)
if [[ "${DURATION}" != "0" ]]; then
  RECORDER_ARGS+=(--duration "${DURATION}")
fi

python3 "${RECORDER_SCRIPT}" "${RECORDER_ARGS[@]}" &
RECORDER_PID=$!

if [[ "${DURATION}" != "0" ]]; then
  # Fixed-length recording: wait for the recorder to exit.
  wait "${RECORDER_PID}"
  RECORDER_EXIT=$?
  RECORDER_PID=""
  if [[ ${RECORDER_EXIT} -ne 0 ]]; then
    echo "Recorder exited with status ${RECORDER_EXIT}" >&2
    exit "${RECORDER_EXIT}"
  fi
else
  # Open-ended recording: stop when the rollout stack exits.
  wait "${STACK_PID}"
  STACK_EXIT=$?
  STACK_PID=""
  kill -TERM "${RECORDER_PID}" >/dev/null 2>&1 || true
  wait "${RECORDER_PID}" >/dev/null 2>&1 || true
  RECORDER_PID=""
  if [[ ${STACK_EXIT} -ne 0 ]]; then
    echo "Rollout stack exited with status ${STACK_EXIT}" >&2
    exit "${STACK_EXIT}"
  fi
fi

echo "Recorded ${OUTPUT_PATH}"
