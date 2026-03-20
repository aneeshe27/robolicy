"""Fixed derived features and predicates for Acrobot V1."""

from __future__ import annotations

from typing import Any, Dict

import numpy as np


DEFAULT_THRESHOLDS = {
    "near_top_angle_rad": 0.45,
    "near_top_distance": 0.35,
    "overspeed_abs": 2.2,
    "energy_target": 1.75,
    "low_energy_margin": 0.40,
}


def _wrap_to_pi(x: float) -> float:
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def _parse_dm_acrobot_obs(obs: Dict[str, Any]) -> tuple[float, float, float, float]:
    """Return theta1, theta2, dtheta1, dtheta2 from dm_control observation dict."""
    orientations = np.asarray(obs["orientations"], dtype=np.float64).reshape(-1)
    velocity = np.asarray(obs["velocity"], dtype=np.float64).reshape(-1)
    if orientations.shape[0] < 4 or velocity.shape[0] < 2:
        raise ValueError("Unexpected dm_control acrobot observation shape")

    # dm_control Acrobot emits [upper_horizontal, lower_horizontal, upper_vertical, lower_vertical].
    h1, h2, v1, v2 = orientations[:4]
    dtheta1, dtheta2 = velocity[:2]
    theta1 = float(np.arctan2(h1, v1))
    theta2 = float(np.arctan2(h2, v2))
    return theta1, theta2, float(dtheta1), float(dtheta2)


def compute_derived(
    obs: Dict[str, Any],
    memory: Dict[str, Any] | None = None,
    thresholds: Dict[str, float] | None = None,
) -> Dict[str, Any]:
    cfg = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        cfg.update(thresholds)

    theta1, theta2, dtheta1, dtheta2 = _parse_dm_acrobot_obs(obs)
    # With dm_control orientation convention, 0 means link points upward.
    top_angle = theta2
    top_angle_error = _wrap_to_pi(top_angle)
    speed_norm = float(np.sqrt(dtheta1 * dtheta1 + dtheta2 * dtheta2))

    # V1 proxy energy: simple potential+kinetic surrogate, monotone with swing amplitude.
    potential = 1.0 - float(np.cos(top_angle))
    kinetic = 0.5 * float(dtheta1 * dtheta1 + dtheta2 * dtheta2)
    energy = potential + kinetic
    energy_gap = float(cfg["energy_target"] - energy)

    prev_top_angle = None
    if memory:
        prev_top_angle = memory.get("prev_top_angle")
    crossing_direction = 0
    if prev_top_angle is not None:
        delta = _wrap_to_pi(top_angle - float(prev_top_angle))
        crossing_direction = 1 if delta > 0.0 else (-1 if delta < 0.0 else 0)

    near_top = abs(top_angle_error) < float(cfg["near_top_angle_rad"])
    overspeed = speed_norm > float(cfg["overspeed_abs"])
    low_energy = energy < float(cfg["energy_target"] - cfg["low_energy_margin"])

    target_distance = None
    if memory:
        physics = memory.get("_runtime_physics")
        if physics is not None:
            target_distance = float(physics.to_target())
            near_top = target_distance < float(cfg["near_top_distance"])

    return {
        "theta1": theta1,
        "theta2": theta2,
        "joint_velocities": (dtheta1, dtheta2),
        "top_angle": top_angle,
        "top_angle_error": top_angle_error,
        "speed_norm": speed_norm,
        "energy": energy,
        "energy_gap": energy_gap,
        "target_distance": target_distance,
        "near_top": near_top,
        "overspeed": overspeed,
        "low_energy": low_energy,
        "crossing_direction": crossing_direction,
    }

