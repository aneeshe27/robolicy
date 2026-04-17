#!/usr/bin/env bash
set -euo pipefail

OUTPUT_PATH="${1:-/workspace/outputs/pickplace_demo.mp4}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
DURATION="${DURATION:-16}"
DISPLAY_NUM="${DISPLAY_NUM:-:99}"
BOOT_WAIT="${BOOT_WAIT:-45}"
CROP_X="${CROP_X:-272}"
CROP_Y="${CROP_Y:-79}"
CROP_W="${CROP_W:-560}"
CROP_H="${CROP_H:-315}"
STACK_SCRIPT="${STACK_SCRIPT:-/workspace/scripts/run_external_pickplace_stack.sh}"

mkdir -p "$(dirname "${OUTPUT_PATH}")"

cleanup() {
  local exit_code=$?
  kill "${FFMPEG_PID:-}" >/dev/null 2>&1 || true
  kill "${STACK_PID:-}" >/dev/null 2>&1 || true
  kill "${XVFB_PID:-}" >/dev/null 2>&1 || true
  wait "${FFMPEG_PID:-}" >/dev/null 2>&1 || true
  wait "${STACK_PID:-}" >/dev/null 2>&1 || true
  wait "${XVFB_PID:-}" >/dev/null 2>&1 || true
  exit "${exit_code}"
}
trap cleanup EXIT

Xvfb "${DISPLAY_NUM}" -screen 0 "${WIDTH}x${HEIGHT}x24" -ac +extension GLX +render -noreset >/tmp/pickplace_xvfb.log 2>&1 &
XVFB_PID=$!
sleep 2

export DISPLAY="${DISPLAY_NUM}"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1

"${STACK_SCRIPT}" >/tmp/pickplace_stack.log 2>&1 &
STACK_PID=$!

sleep "${BOOT_WAIT}"

ffmpeg -y \
  -video_size "${WIDTH}x${HEIGHT}" \
  -framerate "${FPS}" \
  -f x11grab \
  -i "${DISPLAY_NUM}.0" \
  -t "${DURATION}" \
  -vf "crop=${CROP_W}:${CROP_H}:${CROP_X}:${CROP_Y},scale=${WIDTH}:${HEIGHT}:flags=lanczos" \
  -pix_fmt yuv420p \
  "${OUTPUT_PATH}" >/tmp/pickplace_ffmpeg.log 2>&1 &
FFMPEG_PID=$!

wait "${FFMPEG_PID}"

echo "Recorded ${OUTPUT_PATH}"
