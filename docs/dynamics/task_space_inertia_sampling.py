"""
Sample the task space inertia Λ(q) of the MuJoCo model at the TCP over random collision free poses and
check the Cartesian damping of the cartesian_impedance_controller (impedance mode) against it.

Per Cartesian axis: the range of Λ_ii(q) and the damping ratio of the constant cartesian_damping. Then the
damping ratio of the closed loop (linearized, smallest over its modes) per pose for three damping laws:
the constant D, D per axis from Λ_ii (simplified) and D from the full Λ (generalized eigenproblem). The
adaptive laws are also evaluated with a wrong armature in the controller model. Last, the damping the joints
feel (largest eigenvalue of Jᵀ D J) in regular and near singular poses. Needs a sourced workspace
(robotarm_mujoco).

    python3 docs/dynamics/task_space_inertia_sampling.py [--tool none] [--zeta 0.7]
"""

import argparse
import os

import mujoco
import numpy as np
from robotarm_mujoco.dynamics import full_mass_matrix, load_model
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
SIM_PARAMS = os.path.join(REPO, 'src', 'robotarm_mujoco', 'config', 'sim_params.yaml')
CONTROLLERS = os.path.join(REPO, 'src', 'robotarm_bringup', 'config', 'controllers.yaml')
CONTROLLER = 'cartesian_impedance_controller'
AXES = ['x', 'y', 'z', 'rx', 'ry', 'rz']
UNITS = ['kg'] * 3 + ['kg·m²'] * 3


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--tool', default='none',
                        help='file name in robotarm_description/urdf/tools/ without .xacro')
    parser.add_argument('--params', default=SIM_PARAMS, help='sim parameter yaml (default: source tree)')
    parser.add_argument('--controllers', default=CONTROLLERS, help='controllers.yaml with the gains')
    parser.add_argument('--site', default='tcp', help='MuJoCo site of the TCP')
    parser.add_argument('--samples', type=int, default=2000, help='number of collision free random poses')
    parser.add_argument('--zeta', type=float, default=0.7, help='target damping ratio of the adaptive laws')
    parser.add_argument('--armature-error', type=float, nargs='*', default=[0.5, 2.0],
                        help='factors on the armature in the controller model (the plant keeps the nominal one)')
    parser.add_argument('--sigma-regular', type=float, default=0.05,
                        help='poses with a smallest singular value of J above this count as regular (the jacobian '
                             'inverse damps below sqrt(2)·λ)')
    parser.add_argument('--seed', type=int, default=0)
    return parser.parse_args()


def load_impedance_gains(path):
    with open(path) as f:
        p = yaml.safe_load(f)[CONTROLLER]['ros__parameters']
    return (np.array(p['cartesian_stiffness']), np.array(p['cartesian_damping']),
            np.array(p['joint_damping']), float(p['lambda']))


def task_inertia(J, M, lam):
    """Λ = (J M⁻¹ Jᵀ + λ² I)⁻¹, regularized like the damped jacobian inverse."""
    return np.linalg.inv(J @ np.linalg.solve(M, J.T) + lam ** 2 * np.eye(6))


def damping_diagonal(Lam, K, zeta):
    """Simplified: D_i = 2 ζ sqrt(K_i Λ_ii), coupling of Λ ignored."""
    return np.diag(2 * zeta * np.sqrt(K * np.diag(Lam)))


def damping_full(Lam, K, zeta):
    """
    Full: double diagonalization. With Λ = L Lᵀ and L⁻¹ K L⁻ᵀ = V diag(k0) Vᵀ, Q = Vᵀ Lᵀ gives
    Λ = Qᵀ Q, K = Qᵀ diag(k0) Q, and D = 2 Qᵀ diag(ζ sqrt(k0)) Q.
    """
    L = np.linalg.cholesky(Lam)
    Li = np.linalg.inv(L)
    k0, V = np.linalg.eigh(Li @ np.diag(K) @ Li.T)
    Q = V.T @ L.T
    return 2 * Q.T @ np.diag(zeta * np.sqrt(np.clip(k0, 0, None))) @ Q


def modal_zeta(M, J, K, D_cart, D_joint):
    """
    Modes of the linearized closed loop in joint space, M q̈ + (Jᵀ D J + D_joint) q̇ + Jᵀ K J q = 0.
    Returns the smallest damping ratio over the modes (real eigenvalues, no oscillation, count as ζ = 1)
    and the slowest decay time constant 1/|Re λ|.
    """
    n = M.shape[0]
    Minv = np.linalg.inv(M)
    A = np.block([[np.zeros((n, n)), np.eye(n)],
                  [-Minv @ J.T @ np.diag(K) @ J, -Minv @ (J.T @ D_cart @ J + np.diag(D_joint))]])
    ev = np.linalg.eigvals(A)
    tau_slow = 1.0 / max(np.abs(ev.real).min(), 1e-12)
    osc = ev[np.abs(ev.imag) > 1e-9 * np.abs(ev).max()]
    zeta = min(1.0, (-osc.real / np.abs(osc)).min()) if osc.size else 1.0
    return zeta, tau_slow


def pct(x, p):
    return np.percentile(x, p, axis=0)


def main():
    args = parse_args()
    m = load_model(args.tool, args.params)
    d = mujoco.MjData(m)
    site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, args.site)
    lo, hi = m.jnt_range[:, 0], m.jnt_range[:, 1]
    K, D_const, D_joint, lam = load_impedance_gains(args.controllers)
    D_const = np.diag(D_const)
    arm = np.diag(m.dof_armature)
    zeta = np.full(6, args.zeta)

    # damping laws: name -> D(Λ_model). Λ_model uses the controller's armature, the plant the nominal one.
    laws = {'joint_damping only': lambda Lam: np.zeros((6, 6)),
            'constant D': lambda Lam: D_const,
            'simplified, Λ_ii': lambda Lam: damping_diagonal(Lam, K, zeta),
            'simplified, 1/(Λ⁻¹)_ii': lambda Lam: damping_diagonal(np.diag(1 / np.diag(np.linalg.inv(Lam))), K, zeta),
            'full, generalized eig': lambda Lam: damping_full(Lam, K, zeta)}
    cases = [(name, 1.0) for name in laws] + [(name, f) for name in list(laws)[2:] for f in args.armature_error]

    rng = np.random.default_rng(args.seed)
    jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    lam_diag, lam_noarm, sigma_min, tcp_dist = [], [], [], []
    zeta_modal = {c: ([], []) for c in cases}   # (cartesian damping only, with joint damping)
    joint_damping_max = {name: [] for name in list(laws)[1:]}   # largest eigenvalue of Jᵀ D J, nominal armature
    rejected = 0
    while len(lam_diag) < args.samples:
        q = rng.uniform(lo, hi)
        M = full_mass_matrix(m, d, q)
        if d.ncon > 0:          # self collision or below the floor
            rejected += 1
            continue
        mujoco.mj_jacSite(m, d, jacp, jacr, site)
        J = np.vstack([jacp, jacr])
        Lam = task_inertia(J, M, lam)
        lam_diag.append(np.diag(Lam))
        lam_noarm.append(np.diag(task_inertia(J, M - arm, lam)))
        sigma_min.append(np.linalg.svd(J, compute_uv=False)[-1])
        tcp_dist.append(np.linalg.norm(d.site_xpos[site]))
        for name, f in cases:
            D = laws[name](task_inertia(J, M + (f - 1) * arm, lam))
            zeta_modal[(name, f)][0].append(modal_zeta(M, J, K, D, np.zeros(m.nv)))
            zeta_modal[(name, f)][1].append(modal_zeta(M, J, K, D, D_joint))
            if f == 1.0 and name in joint_damping_max:
                joint_damping_max[name].append(np.linalg.eigvalsh(J.T @ D @ J).max())
    lam_diag, lam_noarm = np.array(lam_diag), np.array(lam_noarm)
    sigma_min, tcp_dist = np.array(sigma_min), np.array(tcp_dist)
    modal = {c: tuple(np.array(v) for v in zeta_modal[c]) for c in cases}   # each (samples, 2): ζ, τ_slow

    M_zero = full_mass_matrix(m, d, np.clip(np.zeros(m.nv), lo, hi))
    mujoco.mj_jacSite(m, d, jacp, jacr, site)
    lam_zero = np.diag(task_inertia(np.vstack([jacp, jacr]), M_zero, lam))

    regular = sigma_min >= args.sigma_regular
    print(f'tool: {args.tool}, site: {args.site}, {args.samples} collision free random poses '
          f'({rejected} colliding rejected), λ = {lam:g}')
    print(f'TCP distance from the base: {tcp_dist.min():.2f} – {tcp_dist.max():.2f} m, σ_min(J) p5 '
          f'{pct(sigma_min, 5):.3f}, median {np.median(sigma_min):.3f}, below the jinv threshold '
          f'sqrt(2)·λ: {np.mean(sigma_min < np.sqrt(2) * lam):.0%}, regular (σ_min ≥ {args.sigma_regular:g}): '
          f'{regular.mean():.0%}')
    print(f'Λ_ii at the zero pose: ' + ', '.join(f'{a} {v:.3g}' for a, v in zip(AXES, lam_zero)))

    for title, sel in ((f'regular poses, σ_min ≥ {args.sigma_regular:g}', regular), ('all poses', slice(None))):
        print(f'\n### {title}\n')
        L, L0 = lam_diag[sel], lam_noarm[sel]
        p5, med, p95 = pct(L, 5), np.median(L, 0), pct(L, 95)
        share = 1 - np.median(L0 / L, 0)
        print('| axis | unit | min | p5 | median | p95 | max | p95/p5 | armature share (median) |')
        print('|---|---|---|---|---|---|---|---|---|')
        for i, a in enumerate(AXES):
            print(f'| {a} | {UNITS[i]} | {L[:, i].min():.3g} | {p5[i]:.3g} | {med[i]:.3g} '
                  f'| {p95[i]:.3g} | {L[:, i].max():.3g} | {p95[i] / p5[i]:.1f} | {share[i]:.0%} |')

        print(f'\n| axis | K | D now | ζ now p5 – p95 | f_n p5 – p95 [Hz] | D for ζ={args.zeta:g} p5 – p95 |')
        print('|---|---|---|---|---|---|')
        for i, a in enumerate(AXES):
            z = D_const[i, i] / (2 * np.sqrt(K[i] * L[:, i]))
            fn = np.sqrt(K[i] / L[:, i]) / (2 * np.pi)
            need = 2 * args.zeta * np.sqrt(K[i] * L[:, i])
            print(f'| {a} | {K[i]:g} | {D_const[i, i]:g} | {pct(z, 5):.2f} – {pct(z, 95):.2f} '
                  f'| {pct(fn, 5):.1f} – {pct(fn, 95):.1f} | {pct(need, 5):.3g} – {pct(need, 95):.3g} |')

        for col, label in ((0, 'cartesian damping only'), (1, 'with joint_damping')):
            print(f'\nclosed loop, {label}: smallest modal ζ per pose (1 = no oscillating mode) '
                  f'and slowest decay time constant\n')
            print('| damping law | armature in model | ζ p5 | ζ median | ζ p95 | τ_slow median [ms] | τ_slow p95 [ms] |')
            print('|---|---|---|---|---|---|---|')
            for name, f in cases:
                z, tau = modal[(name, f)][col][sel].T
                if name == 'joint_damping only' and col == 0:
                    continue
                arm_str = f'×{f:g}' if name in list(laws)[2:] else '–'
                print(f'| {name} | {arm_str} | {pct(z, 5):.2f} | {np.median(z):.2f} | {pct(z, 95):.2f} '
                      f'| {np.median(tau) * 1e3:.0f} | {pct(tau, 95) * 1e3:.0f} |')

    singular = sigma_min < np.sqrt(2) * lam
    print(f'\n### damping felt by the joints: largest eigenvalue of Jᵀ D J [Nm·s/rad]\n')
    print(f'| damping law | regular: median | p95 | max | near singular (σ_min < {np.sqrt(2) * lam:.3g}): median '
          f'| p95 | max |')
    print('|---|---|---|---|---|---|---|')
    for name, v in joint_damping_max.items():
        v = np.array(v)
        r, s_ = v[regular], v[singular]
        print(f'| {name} | {np.median(r):.3g} | {pct(r, 95):.3g} | {r.max():.3g} '
              f'| {np.median(s_):.3g} | {pct(s_, 95):.3g} | {s_.max():.3g} |')

if __name__ == '__main__':
    main()
