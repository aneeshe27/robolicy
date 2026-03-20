#!/usr/bin/env python3
"""Run policy evaluation on dm_control acrobot and emit structured diagnostics.

V1 behavior:
- loads editable policy artifact (`policy.yaml`),
- executes fixed primitives over dm_control acrobot/swingup,
- records a rollout MP4 with a timestamped filename by default.
"""

from __future__ import annotations

import argparse
import ast
import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

import imageio.v2 as imageio
import numpy as np
import yaml
from dm_control.rl import control
from dm_control.suite import acrobot as acrobot_module

from predicates import compute_derived
from primitives import PRIMITIVES


def _load_acrobot(seed: int, gear: int = 6) -> control.Environment:
    """Load dm_control acrobot with a custom actuator gear ratio."""
    xml_raw, assets = acrobot_module.get_model_and_assets()
    xml_str = xml_raw.decode("utf-8") if isinstance(xml_raw, bytes) else xml_raw
    xml_str = xml_str.replace('gear="2"', f'gear="{gear}"')
    physics = acrobot_module.Physics.from_xml_string(xml_str, assets)
    task = acrobot_module.Balance(sparse=False, random=seed)
    return control.Environment(physics, task, time_limit=40)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, default=Path("policy.yaml"))
    parser.add_argument("--steps", type=int, default=600)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--camera-id", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--gear", type=int, default=9)
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--video-dir", type=Path, default=Path("outputs"))
    return parser.parse_args()


def _safe_eval_condition(expr: str, derived: Dict[str, Any], memory: Dict[str, Any]) -> bool:
    allowed_nodes = (
        ast.Expression,
        ast.BoolOp,
        ast.UnaryOp,
        ast.BinOp,
        ast.Compare,
        ast.Name,
        ast.Load,
        ast.Constant,
        ast.And,
        ast.Or,
        ast.Not,
        ast.Eq,
        ast.NotEq,
        ast.Lt,
        ast.LtE,
        ast.Gt,
        ast.GtE,
        ast.Is,
        ast.IsNot,
        ast.Add,
        ast.Sub,
        ast.Mult,
        ast.Div,
        ast.Mod,
    )
    tree = ast.parse(expr, mode="eval")
    if not all(isinstance(node, allowed_nodes) for node in ast.walk(tree)):
        raise ValueError(f"Unsupported expression in policy condition: {expr!r}")

    names = {}
    names.update(derived)
    names.update(memory)
    value = eval(compile(tree, "<policy-condition>", "eval"), {"__builtins__": {}}, names)
    return bool(value)


def _pick_next_mode(
    mode_name: str,
    policy: Dict[str, Any],
    derived: Dict[str, Any],
    memory: Dict[str, Any],
    mode_step_count: int,
) -> str:
    mode_cfg = policy["modes"][mode_name]
    dwell_steps = int(mode_cfg.get("dwell_steps", 1))
    if mode_step_count < dwell_steps:
        return mode_name

    for rule in mode_cfg.get("switch_if", []):
        cond = str(rule.get("if", "")).strip()
        target = str(rule.get("then", "")).strip()
        if not cond or not target:
            continue
        if target not in policy["modes"]:
            continue
        if _safe_eval_condition(cond, derived=derived, memory=memory):
            return target
    return mode_name


def _build_result(
    success: bool,
    total_reward: float,
    max_height_fraction: float,
    min_target_distance: float,
    time_near_top: int,
    max_consecutive_stable_steps: int,
    final_consecutive_stable_steps: int,
    capture_attempts: int,
    successful_capture_steps: int,
    usage: Dict[str, int],
    event_trace: list[Dict[str, Any]],
    video_path: str | None,
) -> Dict[str, Any]:
    overspeed_events = sum(1 for e in event_trace if e["event"] == "overspeed")
    capture_failed = sum(1 for e in event_trace if e["event"] == "capture_failed")
    failure_mode = "success" if success else "insufficient stabilization"
    if capture_failed > 0 and overspeed_events > 0:
        failure_mode = "repeated top-entry overshoot"
    elif min_target_distance < 0.25 and max_consecutive_stable_steps < 60:
        failure_mode = "reached_top_but_could_not_hold"

    return {
        "task": "Acrobot swing-up and capture",
        "result": {
            "success": success,
            "return": round(total_reward, 4),
            "max_height_fraction": round(max_height_fraction, 4),
            "min_target_distance": round(min_target_distance, 4),
            "time_near_top": time_near_top,
            "max_consecutive_stable_steps": max_consecutive_stable_steps,
            "final_consecutive_stable_steps": final_consecutive_stable_steps,
            "capture_attempts": capture_attempts,
            "successful_capture_steps": successful_capture_steps,
        },
        "controller_usage": usage,
        "diagnostics": {
            "failure_mode": failure_mode,
            "overspeed_events": overspeed_events,
            "capture_failed_events": capture_failed,
            "success_criteria": {
                "max_consecutive_stable_steps_required": 50,
                "final_consecutive_stable_steps_required": 20,
                "stable_condition": (
                    "target_distance < 0.30 and speed_norm < 1.0 and near_top"
                ),
            },
        },
        "event_trace_tail": event_trace[-20:],
        "video_path": video_path,
    }


def main() -> None:
    args = parse_args()
    policy = yaml.safe_load(args.policy.read_text())
    env = _load_acrobot(seed=args.seed, gear=args.gear)

    mode = str(policy["initial_mode"])
    mode_step_count = 0
    thresholds = dict(policy.get("thresholds", {}))
    memory = dict(policy.get("memory", {}))
    memory["_runtime_physics"] = env.physics
    memory["_mpc_seed"] = args.seed
    memory["_episode_steps_limit"] = args.steps
    usage = {name: 0 for name in PRIMITIVES.keys()}

    timestep = env.reset()
    total_reward = 0.0
    max_height_fraction = 0.0
    min_target_distance = float("inf")
    time_near_top = 0
    capture_attempts = 0
    successful_capture_steps = 0
    consecutive_stable_steps = 0
    max_consecutive_stable_steps = 0
    event_trace: list[Dict[str, Any]] = []

    frames: list[np.ndarray] = []
    video_path: str | None = None
    if not args.no_video:
        frames.append(env.physics.render(height=args.height, width=args.width, camera_id=args.camera_id))

    for t in range(args.steps):
        memory["_episode_step_idx"] = t
        obs = {k: v for k, v in timestep.observation.items()}
        derived = compute_derived(obs=obs, memory=memory, thresholds=thresholds)

        primitive_name = str(policy["modes"][mode]["primitive"])
        primitive = PRIMITIVES[primitive_name]
        out = primitive({"obs": obs, "derived": derived, "memory": memory})
        action = out["action"]
        usage[primitive_name] = usage.get(primitive_name, 0) + 1

        if primitive_name == "capture_top":
            capture_attempts += 1
            if out["status"] == "done":
                successful_capture_steps += 1
            if out["status"] == "failed":
                event_trace.append({"t": t, "event": "capture_failed", "primitive": primitive_name})
                memory["capture_fail_count"] = int(memory.get("capture_fail_count", 0)) + 1

        if bool(derived["overspeed"]):
            event_trace.append({"t": t, "event": "overspeed", "speed": float(derived["speed_norm"])})
        if bool(derived["near_top"]):
            time_near_top += 1
            event_trace.append(
                {
                    "t": t,
                    "event": "entered_near_top",
                    "speed": float(derived["speed_norm"]),
                    "primitive": primitive_name,
                }
            )
        stable_now = (
            bool(derived["near_top"])
            and derived.get("target_distance") is not None
            and float(derived["target_distance"]) < 0.30
            and float(derived["speed_norm"]) < 1.0
        )
        if stable_now:
            consecutive_stable_steps += 1
            max_consecutive_stable_steps = max(max_consecutive_stable_steps, consecutive_stable_steps)
        else:
            consecutive_stable_steps = 0

        top_angle = float(derived["top_angle"])
        target_distance = derived.get("target_distance")
        if target_distance is None:
            height_frac = float((1.0 - np.cos(top_angle)) / 2.0)
        else:
            td = float(target_distance)
            min_target_distance = min(min_target_distance, td)
            # Monotone visualization metric: closer target => higher fraction.
            height_frac = float(max(0.0, 1.0 - td / 2.0))
        max_height_fraction = max(max_height_fraction, height_frac)

        timestep = env.step(action)
        total_reward += 0.0 if timestep.reward is None else float(timestep.reward)

        if not args.no_video:
            frames.append(
                env.physics.render(height=args.height, width=args.width, camera_id=args.camera_id)
            )

        memory["prev_top_angle"] = top_angle
        memory["last_mode"] = mode
        if "recent_failures" in memory and not isinstance(memory["recent_failures"], list):
            memory["recent_failures"] = []

        next_mode = _pick_next_mode(
            mode_name=mode,
            policy=policy,
            derived=derived,
            memory=memory,
            mode_step_count=mode_step_count,
        )
        if next_mode != mode:
            mode = next_mode
            mode_step_count = 0
        else:
            mode_step_count += 1

        if timestep.last():
            break

    success = max_consecutive_stable_steps >= 50 and consecutive_stable_steps >= 20

    if not args.no_video:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.video_dir.mkdir(parents=True, exist_ok=True)
        video_file = args.video_dir / f"eval_rollout_{timestamp}.mp4"
        imageio.mimsave(video_file, frames, fps=args.fps)
        video_path = str(video_file.resolve())

    packet = _build_result(
        success=success,
        total_reward=total_reward,
        max_height_fraction=max_height_fraction,
        min_target_distance=(0.0 if min_target_distance == float("inf") else min_target_distance),
        time_near_top=time_near_top,
        max_consecutive_stable_steps=max_consecutive_stable_steps,
        final_consecutive_stable_steps=consecutive_stable_steps,
        capture_attempts=capture_attempts,
        successful_capture_steps=successful_capture_steps,
        usage=usage,
        event_trace=event_trace,
        video_path=video_path,
    )
    print(json.dumps(packet, indent=2))


if __name__ == "__main__":
    main()

