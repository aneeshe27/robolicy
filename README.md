# Acrobot V1 — Policy-Maintenance POC

A V1 proof of concept for the **Acrobot** task in a MuJoCo / Gym-style environment. This project tests whether an LLM can act as a **policy maintenance layer** over a fixed library of strong controller primitives—not whether it can invent a better low-level controller from scratch.

---

## Core thesis

1. A **nominal** Acrobot policy works in the base environment.
2. A **mild dynamics shift** causes that policy to fail.
3. The LLM is **only** allowed to make **surgical edits** to the **high-level policy**.
4. After each edit, the policy runs in the terminal; the terminal returns **structured diagnostics**.
5. The LLM adjusts the policy again **based only on that output**.

**The LLM must not** rewrite the whole system, change simulator internals, or modify primitive implementations. Its job is **policy repair**.

---

## Execution model

This project is intended to work well with a tool like Claude Code or another code-editing agent.

| Step | What happens |
|------|----------------|
| 1 | Groundwork and Acrobot setup are created **once**. |
| 2 | A **separate** agent chat is opened for **iterative repair**. |
| 3 | That agent may edit **only** the policy artifact. |
| 4 | After each policy edit, it **must** run the policy evaluation in the terminal. |
| 5 | It reads the structured output and makes the **smallest plausible** next edit. |
| 6 | Repeat until success or until the primitive library is deemed insufficient. |

**Goals of this setup:**

- Keep context small  
- Make edits surgical  
- Avoid rewriting the full policy every time  
- Mirror **real policy maintenance** rather than prompt-heavy planning  

---

## Scope for V1

For V1, it is acceptable that:

- Primitives are fairly strong  
- Predicates are hand-designed  
- The policy language is constrained  
- The LLM only edits **high-level control logic**  

Later versions can weaken predicates, require predicate invention, or reduce primitive strength. **V1** should optimize for a **clean, working repair loop**.

---

## Main design boundary

Split the system into:

| Fixed (not LLM-owned) | Single editable artifact |
|------------------------|---------------------------|
| Primitives | **Policy** (modes, switches, thresholds, retries) |
| Runtime / evaluator | |
| Diagnostics | |

### LLM owns

- Mode definitions  
- Switching logic  
- Predicate thresholds  
- Retry / fallback logic  
- Bounded memory logic  

### Runtime owns

- Primitive execution  
- State tracking  
- Predicate evaluation  
- Diagnostics  
- Validation  
- Safety checks  
- Rejection of invalid policy edits  

---

## Primitive library (V1)

A small, fixed library with strong semantics.

| Primitive | Role in hierarchy |
|-----------|-------------------|
| `pump_clockwise` | Build energy in one phase |
| `pump_counterclockwise` | Build energy in the opposite phase |
| `damp_velocity` | Damp when overshooting |
| `capture_top` | Stabilize near upright |

### Primitive contracts

Each primitive exposes a contract readable by the LLM:

- **name**  
- **description**  
- **intended_region**  
- **expected_effects**  
- **known_failure_modes**  

Example:

```json
{
  "name": "capture_top",
  "description": "Locally stabilizes the acrobot near upright.",
  "intended_region": "near upright, moderate velocity",
  "expected_effects": [
    "reduce top angle error",
    "reduce angular velocity"
  ],
  "known_failure_modes": [
    "fails if entered with too much speed",
    "fails if energy is too low"
  ]
}
```

The LLM should reason over these contracts, **not** over primitive implementation internals.

---

## Primitive interface

### Common input

Every primitive consumes the same runtime input object (each may use only a subset):

```text
PrimitiveInput = {
    "obs": current_state_observation,
    "derived": {
        "energy": ...,
        "energy_gap": ...,
        "near_top": ...,
        "overspeed": ...,
        "crossing_direction": ...,
    },
    "memory": {
        "recent_failures": ...,
        "last_mode": ...,
        "capture_fail_count": ...,
    }
}
```

**Principles:**

- Current observation/state  
- A few derived features  
- Optional bounded memory  
- Fixed internal parameters chosen ahead of time  

**V1:** Do **not** let the LLM pass arbitrary numeric values directly into primitive internals.

### Output

Preferred shape:

```text
PrimitiveOutput = {
    "action": torque_or_control_signal,
    "status": "running" | "done" | "failed",
    "confidence": float,
    "diagnostics": {
        "expected_effect": ...,
        "entered_valid_region": ...,
        "saturation": ...,
    }
}
```

Simpler V1 alternative:

```json
{
  "action": "...",
  "valid_now": true,
  "tag": "pumping"
}
```

(`tag` may be e.g. `"pumping" | "capture" | "damping"`.) For Acrobot, **action** remains essential; extra fields support runtime summaries and post-failure repair.

---

## Policy language

The LLM does **not** call primitives every timestep. It edits a **high-level policy program** the runtime executes.

The runtime:

1. Observes state  
2. Evaluates predicates  
3. Chooses the active primitive per the policy  
4. Runs that primitive for a **bounded horizon** or until a switch condition fires  

### V1: small typed mode language (example)

```text
MODE PUMP_CW:
    primitive = pump_clockwise
    switch_if:
        if near_top and overspeed: DAMP
        if near_top and not overspeed: CAPTURE

MODE PUMP_CCW:
    primitive = pump_counterclockwise
    switch_if:
        if near_top and overspeed: DAMP
        if near_top and not overspeed: CAPTURE

MODE DAMP:
    primitive = damp_velocity
    switch_if:
        if near_top and not overspeed: CAPTURE
        if low_energy: PUMP_CW

MODE CAPTURE:
    primitive = capture_top
    switch_if:
        if capture_failed_recently: DAMP
```

---

## What the LLM may vs. may not change

| Allowed | Not allowed |
|---------|-------------|
| Which primitive each mode uses | Primitive implementation internals |
| Switching conditions | Simulator code |
| Predicate thresholds | Arbitrary torque directly |
| Dwell times | Evaluator or diagnostics logic |
| Retry logic, fallback order | Unconstrained Python outside the policy surface |
| Small bounded memory logic | Unrelated files |

This project is specifically about **policy maintenance by surgical edits**.

---

## Policy-maintenance workflow

The iterative repair agent should:

1. Edit the **policy only**  
2. Run the **evaluation script** in the terminal  
3. Read **structured diagnostic** output  
4. Apply the **smallest plausible** next policy edit  
5. Repeat  

Avoid re-emitting or rewriting the **entire** policy when a small patch suffices.

---

## Post-failure packet for the LLM

After a failed rollout, the LLM should **not** rely mainly on raw reward or raw video. It should receive a **structured failure packet** including:

- Current policy program  
- Primitive catalog and contracts  
- Rollout summary  
- Structured failure diagnosis  
- Short event trace  
- Allowed edit types  
- Optional extra diagnostics on request  

Example:

```json
{
  "task": "Acrobot swing-up and capture",
  "attempt_id": 1,
  "perturbation": {
    "link2_mass_scale": 1.25
  },
  "current_policy": "...policy text here...",

  "result": {
    "success": false,
    "return": -500,
    "max_height_fraction": 0.91,
    "time_near_top": 12,
    "capture_attempts": 4,
    "successful_capture_steps": 0
  },

  "controller_usage": {
    "pump_clockwise": 120,
    "pump_counterclockwise": 95,
    "damp_velocity": 0,
    "capture_top": 18
  },

  "diagnostics": {
    "failure_mode": "repeated top-entry overshoot",
    "avg_top_entry_speed": 2.8,
    "entered_capture_region": 4,
    "capture_engaged_outside_valid_region": 2,
    "overspeed_events": 5,
    "low_energy_events": 0,
    "rapid_mode_switches": 3
  },

  "event_trace": [
    {
      "t": 140,
      "event": "entered_near_top",
      "speed": 3.1,
      "primitive": "capture_top"
    },
    {
      "t": 142,
      "event": "capture_failed",
      "speed": 2.7,
      "primitive": "capture_top"
    }
  ],

  "allowed_edits": [
    "change mode transitions",
    "change predicate thresholds",
    "insert damp before capture",
    "add retry logic"
  ]
}
```

This packet is the **primary** information source for policy repair.

---

## Implementation preference

To support surgical edits, the policy should live in a **small dedicated artifact**, e.g.:

- `policy.yaml`  
- `policy.json`  
- A tiny DSL file  

Avoid making the editable surface a **large Python file** when possible.

### Suggested codebase split

| File | Role |
|------|------|
| `primitives.py` | Fixed primitives |
| `predicates.py` | Fixed (V1) |
| `policy.yaml` (or similar) | **Editable** policy |
| `run_eval.py` | Fixed evaluation entrypoint |
| `diagnostics.py` | Fixed diagnostics |
| `verify_policy.py` | Fixed policy validation |

---

## Summary

**V1** validates a closed loop: **dynamics shift → failure → structured diagnostics → surgical policy edit → re-evaluate**, with the LLM constrained to the policy artifact and reasoning from contracts and diagnostics—not from rewriting the simulator or primitives.
