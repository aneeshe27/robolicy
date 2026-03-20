#!/usr/bin/env python3
"""Run a toy DeepMind Acrobot rollout with fixed clockwise torque.

This script is designed for SSH/headless use:
- it steps dm_control acrobot/swingup with a fixed action,
- renders RGB frames directly from MuJoCo,
- writes an MP4 you can watch locally or over remote tooling.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from dm_control import suite


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--steps",
        type=int,
        default=600,
        help="Number of control steps to run.",
    )
    parser.add_argument(
        "--action",
        type=float,
        default=-1.0,
        help="Constant action value (negative ~= clockwise from default camera).",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=640,
        help="Frame width in pixels.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help="Frame height in pixels.",
    )
    parser.add_argument(
        "--camera-id",
        type=int,
        default=0,
        help="Camera ID for MuJoCo render.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Output video frames per second.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("outputs/toy_acrobot_cw.mp4"),
        help="Path to write MP4 rollout.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    env = suite.load(domain_name="acrobot", task_name="swingup")
    timestep = env.reset()

    # dm_control acrobot has one continuous actuator.
    action_spec = env.action_spec()
    action_value = float(np.clip(args.action, action_spec.minimum[0], action_spec.maximum[0]))
    action = np.array([action_value], dtype=np.float32)

    frames: list[np.ndarray] = []
    total_reward = 0.0

    # Include the initial frame.
    frames.append(
        env.physics.render(
            height=args.height,
            width=args.width,
            camera_id=args.camera_id,
        )
    )

    for _ in range(args.steps):
        timestep = env.step(action)
        reward = 0.0 if timestep.reward is None else float(timestep.reward)
        total_reward += reward
        frames.append(
            env.physics.render(
                height=args.height,
                width=args.width,
                camera_id=args.camera_id,
            )
        )
        if timestep.last():
            break

    imageio.mimsave(args.output, frames, fps=args.fps)

    print(f"Saved rollout video: {args.output.resolve()}")
    print(f"Applied fixed action: {action_value:.3f}")
    print(f"Frames: {len(frames)}, cumulative_reward: {total_reward:.4f}")


if __name__ == "__main__":
    main()
