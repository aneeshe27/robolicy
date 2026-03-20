"""Fixed primitive library for Acrobot policy maintenance V1.

Controller approach (gear=9, elbow-only, no external forces):
  - Energy-shaping + tip-steering + kick for global swing-up.
  - Collocated PFL capture: exact elbow linearization with high-gain
    tip-angle PD (kp=200) + secondary q2 centering. The large
    effective gain (Minv11*GEAR ≈ 950) keeps control within limits.
"""

from __future__ import annotations

from typing import Any, Dict, Literal, TypedDict

import mujoco
import numpy as np

PrimitiveStatus = Literal["running", "done", "failed"]


class PrimitiveInput(TypedDict):
    obs: Dict[str, Any]
    derived: Dict[str, Any]
    memory: Dict[str, Any]


class PrimitiveOutput(TypedDict):
    action: np.ndarray
    status: PrimitiveStatus
    confidence: float
    diagnostics: Dict[str, Any]


class PrimitiveContract(TypedDict):
    name: str
    description: str
    intended_region: str
    expected_effects: list[str]
    known_failure_modes: list[str]


PRIMITIVE_CONTRACTS: Dict[str, PrimitiveContract] = {
    "pump_clockwise": {
        "name": "pump_clockwise",
        "description": "Fixed clockwise torque.",
        "intended_region": "far from upright",
        "expected_effects": ["increase swing amplitude"],
        "known_failure_modes": ["overspeed near top"],
    },
    "pump_counterclockwise": {
        "name": "pump_counterclockwise",
        "description": "Fixed counterclockwise torque.",
        "intended_region": "far from upright",
        "expected_effects": ["increase swing amplitude"],
        "known_failure_modes": ["overspeed near top"],
    },
    "damp_velocity": {
        "name": "damp_velocity",
        "description": "Opposes elbow velocity to remove energy.",
        "intended_region": "near upright, excessive velocity",
        "expected_effects": ["reduce speed"],
        "known_failure_modes": ["may drift from target"],
    },
    "capture_top": {
        "name": "capture_top",
        "description": "Collocated PFL: elbow linearization + high-gain tip tracking.",
        "intended_region": "near upright",
        "expected_effects": ["hold at target"],
        "known_failure_modes": ["coupling insufficient for large shoulder angles"],
    },
    "solve_swingup": {
        "name": "solve_swingup",
        "description": "Energy-shaping + steering swing-up.",
        "intended_region": "global",
        "expected_effects": ["reach and maintain near upright"],
        "known_failure_modes": ["residual oscillation from underactuation"],
    },
}

_GEAR = 9.0
_upright_energy_cache: dict[int, float] = {}


def _wrap(x: float) -> float:
    return float((x + np.pi) % (2.0 * np.pi) - np.pi)


def _get_upright_energy(physics: Any) -> float:
    key = id(physics)
    if key in _upright_energy_cache:
        return _upright_energy_cache[key]
    state = physics.get_state().copy()
    physics.data.qpos[:] = 0.0
    physics.data.qvel[:] = 0.0
    physics.data.ctrl[:] = 0.0
    physics.forward()
    e_star = float(physics.data.energy[0])
    physics.set_state(state)
    physics.forward()
    _upright_energy_cache[key] = e_star
    return e_star


def _clip(u: float) -> np.ndarray:
    return np.array([float(np.clip(u, -1.0, 1.0))], dtype=np.float32)


def _capture_action(physics: Any) -> float:
    """Collocated PFL capture with high-gain tip tracking.

    Exactly linearizes the elbow (collocated joint), then uses the virtual
    input to drive the tip angle q1+q2 → 0 while centering the elbow.
    The collocated structure keeps control well within actuator limits
    (effective gain Minv11*GEAR ≈ 950) even with aggressive PD gains.
    """
    q1 = _wrap(float(physics.data.qpos[0]))
    q2 = _wrap(float(physics.data.qpos[1]))
    dq1 = float(physics.data.qvel[0])
    dq2 = float(physics.data.qvel[1])

    M = np.zeros((physics.model.nv, physics.model.nv))
    mujoco.mj_fullM(physics.model.ptr, M, physics.data.qM)
    bias = physics.data.qfrc_bias.copy()

    tip = q1 + q2
    dtip = dq1 + dq2

    kp_tip = 200.0
    kd_tip = 40.0
    kp_q2 = 10.0
    kd_q2 = 3.0

    v = -kp_tip * tip - kd_tip * dtip - kp_q2 * q2 - kd_q2 * dq2

    det_M = M[0, 0] * M[1, 1] - M[0, 1] ** 2
    Minv10 = -M[1, 0] / det_M
    Minv11 = M[0, 0] / det_M

    u = (v + Minv10 * bias[0] + Minv11 * bias[1]) / (Minv11 * _GEAR)

    return float(u)


def pump_clockwise(inp: PrimitiveInput) -> PrimitiveOutput:
    return {"action": _clip(-0.95), "status": "running", "confidence": 0.7,
            "diagnostics": {"expected_effect": "energy_cw"}}


def pump_counterclockwise(inp: PrimitiveInput) -> PrimitiveOutput:
    return {"action": _clip(0.95), "status": "running", "confidence": 0.7,
            "diagnostics": {"expected_effect": "energy_ccw"}}


def damp_velocity(inp: PrimitiveInput) -> PrimitiveOutput:
    physics = inp["memory"].get("_runtime_physics")
    if physics is not None:
        dq2 = float(physics.data.qvel[1])
        raw = -float(np.sign(dq2))
    else:
        dq1, dq2 = inp["derived"].get("joint_velocities", (0.0, 0.0))
        v = float(dq1 + dq2)
        raw = -1.0 * float(np.sign(v)) if abs(v) > 1.5 else -0.5 * v
    sn = float(inp["derived"]["speed_norm"])
    done = sn < 0.5
    return {"action": _clip(raw), "status": "done" if done else "running",
            "confidence": 0.75, "diagnostics": {"expected_effect": "damp"}}


_capture_log_count = 0

def capture_top(inp: PrimitiveInput) -> PrimitiveOutput:
    global _capture_log_count
    physics = inp["memory"].get("_runtime_physics")
    td = inp["derived"].get("target_distance")
    sn = float(inp["derived"]["speed_norm"])
    near = bool(inp["derived"].get("near_top", False))

    if physics is not None:
        raw = _capture_action(physics)
        action = _clip(raw)
        if _capture_log_count < 200:
            import sys
            q1w = _wrap(float(physics.data.qpos[0]))
            q2w = _wrap(float(physics.data.qpos[1]))
            dq1 = float(physics.data.qvel[0])
            dq2 = float(physics.data.qvel[1])
            print(f"CAP t={inp['memory'].get('_episode_step_idx',0):4d}  "
                  f"q1={q1w:+.4f} q2={q2w:+.4f} dq1={dq1:+.3f} dq2={dq2:+.3f}  "
                  f"raw_u={raw:+.4f} clipped={float(action[0]):+.4f}  "
                  f"td={float(td) if td is not None else -1:.4f} sn={sn:.3f}",
                  file=sys.stderr)
            _capture_log_count += 1
    else:
        te = float(inp["derived"].get("top_angle_error", 0.0))
        dq1, dq2 = inp["derived"].get("joint_velocities", (0.0, 0.0))
        action = _clip(-(3.0 * te + 1.0 * float(dq1 + dq2)))

    done = near and td is not None and float(td) < 0.10 and sn < 0.35
    return {"action": action, "status": "done" if done else "running",
            "confidence": 0.9 if done else 0.6,
            "diagnostics": {"expected_effect": "stabilize"}}


def solve_swingup(inp: PrimitiveInput) -> PrimitiveOutput:
    td = inp["derived"].get("target_distance")
    sn = float(inp["derived"]["speed_norm"])
    near = bool(inp["derived"].get("near_top", False))
    physics = inp["memory"].get("_runtime_physics")

    if physics is None:
        te = float(inp["derived"].get("top_angle_error", 0.0))
        dq1, dq2 = inp["derived"].get("joint_velocities", (0.0, 0.0))
        action = _clip(-(2.0 * te + 0.8 * float(dq1 + dq2)))
        regime = "fallback"
    else:
        e = float(physics.data.energy[0] + physics.data.energy[1])
        e_star = _get_upright_energy(physics)
        q1 = float(physics.data.qpos[0])
        q2 = float(physics.data.qpos[1])
        dq2 = float(physics.data.qvel[1])
        dq1 = float(physics.data.qvel[0])
        e_err = e - e_star

        if abs(dq1) + abs(dq2) < 0.3 and abs(e_err) > 3.0:
            action = _clip(1.0)
            regime = "kick"
        else:
            gain = min(1.0, abs(e_err) / 10.0) * 0.5
            u_energy = -gain * e_err * dq2
            u_steer = -2.0 * float(np.sin(q1 + q2))
            action = _clip(u_energy + u_steer)
            regime = "energy"

    done = near and td is not None and float(td) < 0.15 and sn < 0.4
    return {"action": action, "status": "done" if done else "running",
            "confidence": 0.9 if near else 0.8,
            "diagnostics": {"regime": regime}}


PRIMITIVES = {
    "pump_clockwise": pump_clockwise,
    "pump_counterclockwise": pump_counterclockwise,
    "damp_velocity": damp_velocity,
    "capture_top": capture_top,
    "solve_swingup": solve_swingup,
}
