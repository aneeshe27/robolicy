#!/usr/bin/env python3
"""Record the Panda exterior + wrist camera topics to an mp4 video.

This bypasses the flaky ``gzclient``-on-``Xvfb`` rendering path used by the
old x11grab recorder. Instead we subscribe to the same ROS image topics the
openpi bridge already consumes and write them (side by side) through
``ffmpeg``. The resulting video shows exactly what pi0 is conditioning on.
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def _placeholder(width, height, label):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    cv2.putText(
        frame,
        label,
        ((width - tw) // 2, (height + th) // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (180, 180, 180),
        2,
    )
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="Destination mp4 path.")
    parser.add_argument("--exterior-topic", default="/camera/color/image_raw")
    parser.add_argument("--wrist-topic", default="/wrist_camera/color/image_raw")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="If >0, stop after this many seconds. Otherwise record until SIGTERM.",
    )
    parser.add_argument("--tile-width", type=int, default=640)
    parser.add_argument("--tile-height", type=int, default=480)
    parser.add_argument("--prompt", default="")
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for the first exterior frame before giving up.",
    )
    args = parser.parse_args()

    bridge = CvBridge()
    lock = threading.Lock()
    state = {
        "ext": None,
        "wrist": None,
        "frames_written": 0,
        "ext_received": 0,
        "wrist_received": 0,
    }

    def _ext_cb(msg):
        try:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logwarn_throttle(5.0, f"exterior cv_bridge error: {exc}")
            return
        with lock:
            state["ext"] = img
            state["ext_received"] += 1

    def _wrist_cb(msg):
        try:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logwarn_throttle(5.0, f"wrist cv_bridge error: {exc}")
            return
        with lock:
            state["wrist"] = img
            state["wrist_received"] += 1

    rospy.init_node("openpi_rollout_recorder", anonymous=True, disable_signals=True)
    rospy.Subscriber(args.exterior_topic, Image, _ext_cb, queue_size=1)
    rospy.Subscriber(args.wrist_topic, Image, _wrist_cb, queue_size=1)

    rospy.loginfo(
        "openpi rollout recorder waiting on %s and %s",
        args.exterior_topic,
        args.wrist_topic,
    )

    # Wait for the first exterior frame; wrist is allowed to lag.
    wait_deadline = time.time() + args.startup_timeout
    while not rospy.is_shutdown():
        with lock:
            if state["ext"] is not None:
                break
        if time.time() > wait_deadline:
            rospy.logerr(
                "Timed out after %.1fs waiting for exterior frames on %s",
                args.startup_timeout,
                args.exterior_topic,
            )
            sys.exit(2)
        time.sleep(0.25)

    tile_w = args.tile_width
    tile_h = args.tile_height
    banner_h = 44
    full_w = tile_w * 2
    full_h = tile_h + banner_h

    output_path = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    ffmpeg_cmd = [
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-loglevel",
        "warning",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "bgr24",
        "-s",
        f"{full_w}x{full_h}",
        "-r",
        str(args.fps),
        "-i",
        "pipe:0",
        "-pix_fmt",
        "yuv420p",
        "-c:v",
        "libx264",
        "-preset",
        "veryfast",
        "-movflags",
        "+faststart",
        output_path,
    ]
    ffmpeg = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

    stop_requested = {"flag": False}

    def _handle_signal(signum, _frame):
        rospy.loginfo("Recorder got signal %s, stopping.", signum)
        stop_requested["flag"] = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    wrist_placeholder = _placeholder(tile_w, tile_h, "wrist: no frame yet")

    start = time.time()
    frame_interval = 1.0 / args.fps
    next_tick = start

    try:
        while not rospy.is_shutdown() and not stop_requested["flag"]:
            now = time.time()
            if args.duration > 0 and (now - start) >= args.duration:
                break
            if now < next_tick:
                time.sleep(min(frame_interval / 4.0, next_tick - now))
                continue
            next_tick += frame_interval

            with lock:
                ext = state["ext"]
                wrist = state["wrist"]

            if ext is None:
                continue

            if ext.shape[1] != tile_w or ext.shape[0] != tile_h:
                left = cv2.resize(ext, (tile_w, tile_h))
            else:
                left = ext.copy()

            if wrist is None:
                right = wrist_placeholder.copy()
            else:
                if wrist.shape[1] != tile_w or wrist.shape[0] != tile_h:
                    right = cv2.resize(wrist, (tile_w, tile_h))
                else:
                    right = wrist.copy()

            cv2.putText(
                left,
                "exterior",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (40, 255, 40),
                2,
            )
            cv2.putText(
                right,
                "wrist",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (40, 255, 40),
                2,
            )

            top = np.concatenate([left, right], axis=1)
            banner = np.zeros((banner_h, full_w, 3), dtype=np.uint8)
            elapsed = now - start
            msg = (
                f"t={elapsed:6.1f}s  prompt: {args.prompt}"
                if args.prompt
                else f"t={elapsed:6.1f}s"
            )
            cv2.putText(
                banner,
                msg,
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                1,
            )

            frame = np.concatenate([top, banner], axis=0)
            try:
                ffmpeg.stdin.write(frame.tobytes())
            except BrokenPipeError:
                rospy.logerr("ffmpeg pipe closed unexpectedly.")
                break
            state["frames_written"] += 1
    finally:
        try:
            if ffmpeg.stdin:
                ffmpeg.stdin.close()
        except Exception:
            pass
        try:
            ffmpeg.wait(timeout=15)
        except subprocess.TimeoutExpired:
            ffmpeg.kill()
            ffmpeg.wait(timeout=5)

    rospy.loginfo(
        "Wrote %d frames to %s (exterior_msgs=%d wrist_msgs=%d)",
        state["frames_written"],
        output_path,
        state["ext_received"],
        state["wrist_received"],
    )


if __name__ == "__main__":
    main()
