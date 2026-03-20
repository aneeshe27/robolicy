#!/usr/bin/env python3
"""Compute LQR gains for acrobot upright via MuJoCo linearization + DARE."""

import numpy as np
from scipy import linalg
from dm_control.suite import acrobot as acrobot_module
from dm_control.rl import control

GEAR = 9
DT = 0.01  # dm_control default timestep for acrobot


def load_acrobot(gear: int = GEAR) -> control.Environment:
    xml_raw, assets = acrobot_module.get_model_and_assets()
    xml_str = xml_raw.decode("utf-8") if isinstance(xml_raw, bytes) else xml_raw
    xml_str = xml_str.replace('gear="2"', f'gear="{gear}"')
    physics = acrobot_module.Physics.from_xml_string(xml_str, assets)
    task = acrobot_module.Balance(sparse=False, random=0)
    return control.Environment(physics, task, time_limit=10)


def linearize(env: control.Environment, eps_q=1e-4, eps_v=1e-4, eps_u=1e-4):
    """Finite-difference linearization at the upright equilibrium (q=0, dq=0, u=0)."""
    physics = env.physics
    nq = physics.model.nq  # 2
    nv = physics.model.nv  # 2
    nu = physics.model.nu  # 1

    def step_from(q, v, u):
        state = physics.get_state().copy()
        physics.data.qpos[:] = q
        physics.data.qvel[:] = v
        physics.data.ctrl[:] = u
        physics.forward()
        physics.step()
        q_next = physics.data.qpos[:].copy()
        v_next = physics.data.qvel[:].copy()
        physics.set_state(state)
        physics.forward()
        return np.concatenate([q_next, v_next])

    q0 = np.zeros(nq)
    v0 = np.zeros(nv)
    u0 = np.zeros(nu)
    x0 = step_from(q0, v0, u0)

    A = np.zeros((nq + nv, nq + nv))
    for i in range(nq):
        dq = np.zeros(nq)
        dq[i] = eps_q
        xp = step_from(q0 + dq, v0, u0)
        xm = step_from(q0 - dq, v0, u0)
        A[:, i] = (xp - xm) / (2 * eps_q)
    for i in range(nv):
        dv = np.zeros(nv)
        dv[i] = eps_v
        xp = step_from(q0, v0 + dv, u0)
        xm = step_from(q0, v0 - dv, u0)
        A[:, nq + i] = (xp - xm) / (2 * eps_v)

    B = np.zeros((nq + nv, nu))
    for i in range(nu):
        du = np.zeros(nu)
        du[i] = eps_u
        xp = step_from(q0, v0, u0 + du)
        xm = step_from(q0, v0, u0 - du)
        B[:, i] = (xp - xm) / (2 * eps_u)

    return A, B


def solve_dare(A, B, Q, R):
    P = linalg.solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    eigvals = np.linalg.eigvals(A - B @ K)
    return K, P, eigvals


def main():
    env = load_acrobot(gear=GEAR)
    A, B = linearize(env)
    print("=== Linearized system (gear={}) ===".format(GEAR))
    print("A =")
    print(np.array2string(A, precision=6, suppress_small=True))
    print("B =")
    print(np.array2string(B, precision=6, suppress_small=True))

    print("\nOpen-loop eigenvalues:", np.linalg.eigvals(A))

    test_configs = [
        ("Original: Q=diag(10,10,1,1), R=0.01", np.diag([10, 10, 1, 1]), np.array([[0.01]])),
        ("Heavy-R: Q=diag(10,10,1,1), R=1", np.diag([10, 10, 1, 1]), np.array([[1.0]])),
        ("Heavy-R: Q=diag(10,10,1,1), R=10", np.diag([10, 10, 1, 1]), np.array([[10.0]])),
        ("Heavy-R: Q=diag(10,10,1,1), R=100", np.diag([10, 10, 1, 1]), np.array([[100.0]])),
        ("Heavy-R: Q=diag(10,10,1,1), R=1000", np.diag([10, 10, 1, 1]), np.array([[1000.0]])),
        ("Heavy-R: Q=diag(10,10,1,1), R=10000", np.diag([10, 10, 1, 1]), np.array([[10000.0]])),
        ("Weak-Q: Q=diag(1,1,0.1,0.1), R=1", np.diag([1, 1, 0.1, 0.1]), np.array([[1.0]])),
        ("Weak-Q: Q=diag(1,1,0.1,0.1), R=100", np.diag([1, 1, 0.1, 0.1]), np.array([[100.0]])),
    ]

    # Typical capture-entry state
    x_typical = np.array([0.4, -0.5, -1.3, 0.2])

    for label, Q, R in test_configs:
        K, P, eigvals = solve_dare(A, B, Q, R)
        u_typical = float((K @ x_typical)[0])
        u_clipped = float(np.clip(u_typical, -1, 1))
        saturated = abs(u_typical) > 1.0

        print(f"\n--- {label} ---")
        print(f"  K = [{K[0,0]:.3f}, {K[0,1]:.3f}, {K[0,2]:.3f}, {K[0,3]:.3f}]")
        print(f"  CL eigenvalues: {np.abs(eigvals)}")
        print(f"  Max |eig|: {max(abs(eigvals)):.6f} {'(STABLE)' if max(abs(eigvals)) < 1 else '(UNSTABLE)'}")
        print(f"  u at typical state {x_typical}: raw={u_typical:.3f}, clipped={u_clipped:.3f}, saturated={saturated}")

        # Find the max |x| at which controller doesn't saturate (along K direction)
        k_norm = np.linalg.norm(K)
        max_state_norm = 1.0 / k_norm
        print(f"  Max |x| for non-saturation: {max_state_norm:.4f}")


if __name__ == "__main__":
    main()
