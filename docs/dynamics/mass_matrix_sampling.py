"""
Sample the joint space mass matrix of the MuJoCo model over random poses and check the joint damping
of the cartesian_impedance_controller against it.

Per joint: the range of M_ii(q) (armature included), the damping ratio of the current hold spring and
joint damping, and the damping a given damping ratio needs. Needs a sourced workspace (robotarm_mujoco).

    python3 docs/dynamics/mass_matrix_sampling.py [--tool none] [--armature axis5=7.2e-4 ...]
"""

import argparse
import os

import mujoco
import numpy as np
from robotarm_mujoco.dynamics import full_mass_matrix, joint_names, load_gains, load_model

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
SIM_PARAMS = os.path.join(REPO, 'src', 'robotarm_mujoco', 'config', 'sim_params.yaml')
CONTROLLERS = os.path.join(REPO, 'src', 'robotarm_bringup', 'config', 'controllers.yaml')


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--tool', default='none',
                        help='file name in robotarm_description/urdf/tools/ without .xacro')
    parser.add_argument('--params', default=SIM_PARAMS, help='sim parameter yaml (default: source tree)')
    parser.add_argument('--controllers', default=CONTROLLERS, help='controllers.yaml with the gains')
    parser.add_argument('--armature', nargs='*', default=[], metavar='JOINT=VALUE',
                        help='kg·m², overrides the URDF armature of a joint')
    parser.add_argument('--samples', type=int, default=2000, help='number of random poses')
    parser.add_argument('--zeta', type=float, nargs='+', default=[0.7, 1.0],
                        help='damping ratios to compute the needed damping for')
    parser.add_argument('--rate', type=float, default=500.0, help='Hz, controller update rate')
    parser.add_argument('--seed', type=int, default=0)
    return parser.parse_args()


def rng_str(lo, hi, fmt):
    return f'{lo:{fmt}} – {hi:{fmt}}' if not np.isclose(lo, hi, rtol=1e-3) else f'{lo:{fmt}}'


def main():
    args = parse_args()
    armature = dict((name, float(value)) for name, _, value in (a.partition('=') for a in args.armature))
    m = load_model(args.tool, args.params, armature)
    d = mujoco.MjData(m)
    names = joint_names(m)
    lo, hi = m.jnt_range[:, 0], m.jnt_range[:, 1]
    D, K = load_gains(args.controllers)
    dt = 1.0 / args.rate

    rng = np.random.default_rng(args.seed)
    diag = np.zeros((args.samples, m.nv))
    coupling = np.zeros(args.samples)
    for k in range(args.samples):
        M = full_mass_matrix(m, d, rng.uniform(lo, hi))
        diag[k] = np.diag(M)
        s = np.sqrt(diag[k])
        coupling[k] = np.abs(M / np.outer(s, s) - np.eye(m.nv)).max()
    m_min, m_med, m_max = diag.min(0), np.median(diag, 0), diag.max(0)
    m_zero = np.diag(full_mass_matrix(m, d, np.clip(np.zeros(m.nv), lo, hi)))

    print(f'tool: {args.tool}, {args.samples} random poses, uniform within the joint limits\n')
    print('| joint | armature | M_ii at zero | M_ii min | median | max | max/min | armature share (median) |')
    print('|---|---|---|---|---|---|---|---|')
    for i, n in enumerate(names):
        a = m.dof_armature[i]
        print(f'| {n} | {a:.3g} | {m_zero[i]:.3g} | {m_min[i]:.3g} | {m_med[i]:.3g} | {m_max[i]:.3g} '
              f'| {m_max[i] / m_min[i]:.2f} | {a / m_med[i]:.0%} |')
    print(f'\nnormalized off-diagonal coupling max|M_ij|/sqrt(M_ii M_jj): '
          f'median {np.median(coupling):.2f}, max {coupling.max():.2f}\n')

    zeta_cols = ''.join(f' D for ζ={z:g} |' for z in args.zeta)
    print(f'| joint | K hold | D now | ζ now | f_n hold [Hz] | M/D release [ms] | D·dt/M_min |{zeta_cols}')
    print('|---|---|---|---|---|---|---|' + '---|' * len(args.zeta))
    for i, n in enumerate(names):
        decay = rng_str(m_min[i] / D[i] * 1e3, m_max[i] / D[i] * 1e3, '.0f') if D[i] > 0 else '–'
        step = f'{D[i] * dt / m_min[i]:.3f}'
        if K[i] > 0:
            zeta = rng_str(D[i] / (2 * np.sqrt(K[i] * m_max[i])), D[i] / (2 * np.sqrt(K[i] * m_min[i])), '.2f')
            fn = rng_str(np.sqrt(K[i] / m_max[i]) / (2 * np.pi), np.sqrt(K[i] / m_min[i]) / (2 * np.pi), '.1f')
            need = ''.join(f' {rng_str(2 * z * np.sqrt(K[i] * m_min[i]), 2 * z * np.sqrt(K[i] * m_max[i]), ".3g")} |'
                           for z in args.zeta)
        else:
            zeta, fn, need = '–', '–', ' – |' * len(args.zeta)
        print(f'| {n} | {K[i]:g} | {D[i]:g} | {zeta} | {fn} | {decay} | {step} |{need}')


if __name__ == '__main__':
    main()
