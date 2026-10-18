# Joint inertia and damping ballpark

Snapshot of the joint space mass matrix `M(q)` of the MuJoCo model, used to pick the `joint_damping` and
`hold_stiffness` of the `cartesian_impedance_controller` from a target damping ratio instead of by trial
and error. Sampled 2026-10-11 from [`mass_matrix_sampling.py`](mass_matrix_sampling.py), which uses the
model helpers in `robotarm_mujoco.dynamics`. The task space counterpart (Cartesian inertia `Λ(q)` at the TCP, for
the Cartesian damping) is [task_space_inertia.md](task_space_inertia.md).

## Method

Per joint the controller is close to a mass-spring-damper around the hold anchor:

```
M_ii(q)·q̈ + D·q̇ + K·(q − q_anchor) = τ_ext

ω_n = sqrt(K / M_ii)               natural frequency of the hold
ζ   = D / (2·sqrt(K·M_ii))         damping ratio
D   = 2·ζ·sqrt(K·M_ii)             damping for a target ζ
T   = M_ii / D                     decay time constant with damping only (hold slid, arm released)
```

`M(q)` comes from `mujoco.mj_fullM`, so it already contains the reflected rotor inertia (`armature`) that
`sim_params.yaml` adds to each joint. The same matrix can be built from `RobotarmRbd::recursive_newton_euler`
with `q̇ = 0`, gravity off and `q̈ = e_i` (column `i`). The rbd adds the URDF `<armature>` of each joint as
`armature_i·q̈_i`, so the columns already contain it on the diagonal. Note: the URDF values are placeholders (0)
until measured, the numbers below use the `sim_params.yaml` estimates.

Poses are sampled uniformly within the joint limits (2000 poses, seed 0). Some of them would self collide, so
the min/max spread is a bit wider than in real use.

### Armature used

| joint | rotor inertia | ratio | armature | source |
|---|---|---|---|---|
| axis1 | 1.8e-4 kg·m² | 6 | 6.5e-3 kg·m² | `sim_params.yaml`, LA8308 rotor **estimate** |
| axis2 | 1.8e-4 kg·m² | 30 | 0.162 kg·m² | `sim_params.yaml`, LA8308 rotor **estimate** |
| axis3 | 1.8e-4 kg·m² | 11.45 | 2.4e-2 kg·m² | `sim_params.yaml`, LA8308 rotor **estimate** |
| axis4 | 1.8e-4 kg·m² | 4 | 2.9e-3 kg·m² | `sim_params.yaml`, LA8308 rotor **estimate** |
| axis5 | 2.0–2.5e-5 kg·m² | 4 | **7.2e-4** kg·m² (6.4–8.0e-4) | `sim_params.yaml`, wrist motor **estimate**, see below |
| axis6 | 2.0–2.5e-5 kg·m² | 4 | **7.2e-4** kg·m² (6.4–8.0e-4) | `sim_params.yaml`, wrist motor **estimate**, see below |

**Wrist differential:** both wrist motors drive both joints
([DifferentialTransmission.cpp](../../src/moteus_interface/src/DifferentialTransmission.cpp)):
`θ_a = N·(q5 + q6)`, `θ_b = N·(q5 − q6)` at the rotors. The rotor kinetic energy is

```
½·J·N²·[(q̇5 + q̇6)² + (q̇5 − q̇6)²] = ½·(2·J·N²)·(q̇5² + q̇6²)
```

so each wrist joint sees **2·J·N²**, with no cross term. It is twice what a single motor per joint would
give. With `J = 2.25e-5 kg·m²` (middle of 20–25 kg·mm²) and `N = 4`, that is `7.2e-4 kg·m²`. This assumes the
1:4 sits between each motor and the differential, and the differential itself is 1:1 (`ratio` 1.0 in the
`wrist_transmission`). `sim_params.yaml` sets it as `armature` directly, since `rotor_inertia` + `gear_ratio`
would only give half of it.

## Results, no tool

| joint | armature | M_ii at zero | M_ii min | median | max | max/min | armature share (median) |
|---|---|---|---|---|---|---|---|
| axis1 | 0.00648 | 0.973 | 0.0782 | 0.357 | 0.97 | 12.41 | 2% |
| axis2 | 0.162 | 0.975 | 0.476 | 0.788 | 0.977 | 2.05 | 21% |
| axis3 | 0.0236 | 0.152 | 0.114 | 0.139 | 0.154 | 1.36 | 17% |
| axis4 | 0.00288 | 0.00764 | 0.00764 | 0.0113 | 0.0128 | 1.67 | 26% |
| axis5 | 0.00072 | 0.0063 | 0.0063 | 0.0063 | 0.0063 | 1.00 | 11% |
| axis6 | 0.00072 | 0.000746 | 0.000746 | 0.000746 | 0.000746 | 1.00 | 97% |

M_ii in kg·m². Normalized off-diagonal coupling `max|M_ij| / sqrt(M_ii·M_jj)`: median 0.54, max 0.71.

Against the gains in `controllers.yaml` at the time of sampling
(`joint_damping: [4.0, 6.0, 2.0, 0.4, 0.08, 0.002]`, `hold_stiffness: [100, 100, 60, 20, 2, 0]`):

| joint | K hold | D now | ζ now | f_n hold [Hz] | M/D release [ms] | D·dt/M_min | D for ζ=0.7 | D for ζ=1 |
|---|---|---|---|---|---|---|---|---|
| axis1 | 100 | 4 | 0.20 – 0.72 | 1.6 – 5.7 | 20 – 242 | 0.102 | 3.91 – 13.8 | 5.59 – 19.7 |
| axis2 | 100 | 6 | 0.30 – 0.43 | 1.6 – 2.3 | 79 – 163 | 0.025 | 9.66 – 13.8 | 13.8 – 19.8 |
| axis3 | 60 | 2 | 0.33 – 0.38 | 3.1 – 3.7 | 57 – 77 | 0.035 | 3.65 – 4.26 | 5.22 – 6.09 |
| axis4 | 20 | 0.4 | 0.40 – 0.51 | 6.3 – 8.1 | 19 – 32 | 0.105 | 0.547 – 0.707 | 0.782 – 1.01 |
| axis5 | 2 | 0.08 | 0.36 | 2.8 | 79 | 0.025 | 0.157 | 0.225 |
| axis6 | 0 | 0.002 | – | – | 373 | 0.005 | – | – |

K in Nm/rad, D in Nm·s/rad, `dt` = 2 ms (500 Hz). Ranges go from the lightest to the heaviest pose.

## Results, `axis_triad` tool

| joint | armature | M_ii at zero | M_ii min | median | max | max/min | armature share (median) |
|---|---|---|---|---|---|---|---|
| axis1 | 0.00648 | 1 | 0.0784 | 0.363 | 0.998 | 12.73 | 2% |
| axis2 | 0.162 | 1 | 0.476 | 0.803 | 1 | 2.11 | 20% |
| axis3 | 0.0236 | 0.161 | 0.115 | 0.146 | 0.163 | 1.41 | 16% |
| axis4 | 0.00288 | 0.00768 | 0.00768 | 0.0123 | 0.0141 | 1.84 | 24% |
| axis5 | 0.00072 | 0.00768 | 0.00768 | 0.00768 | 0.00768 | 1.00 | 9% |
| axis6 | 0.00072 | 0.000785 | 0.000785 | 0.000785 | 0.000785 | 1.00 | 92% |

| joint | K hold | D now | ζ now | f_n hold [Hz] | M/D release [ms] | D·dt/M_min | D for ζ=0.7 | D for ζ=1 |
|---|---|---|---|---|---|---|---|---|
| axis1 | 100 | 4 | 0.20 – 0.71 | 1.6 – 5.7 | 20 – 250 | 0.102 | 3.92 – 14 | 5.6 – 20 |
| axis2 | 100 | 6 | 0.30 – 0.43 | 1.6 – 2.3 | 79 – 167 | 0.025 | 9.66 – 14 | 13.8 – 20 |
| axis3 | 60 | 2 | 0.32 – 0.38 | 3.1 – 3.6 | 58 – 82 | 0.035 | 3.68 – 4.38 | 5.26 – 6.26 |
| axis4 | 20 | 0.4 | 0.38 – 0.51 | 6.0 – 8.1 | 19 – 35 | 0.104 | 0.549 – 0.744 | 0.784 – 1.06 |
| axis5 | 2 | 0.08 | 0.32 | 2.6 | 96 | 0.021 | 0.173 | 0.248 |
| axis6 | 0 | 0.002 | – | – | 393 | 0.005 | – | – |

The triad tool changes the arm joints by a few percent. Only the wrist joints change noticeably, axis5 by +22%.

## Takeaways

- **The current damping is about half of critical.** ζ is 0.3–0.5 on every joint with a hold spring, so the hold
  overshoots and rings a few times when it catches. ζ ≈ 0.7 at the median pose needs roughly 1.5–2× the current
  values: about `[8, 12, 4, 0.65, 0.16]`.
- **Only axis1 varies with the pose, by 12×.** Its inertia depends on the reach (axis2/axis3), from 0.08 kg·m²
  folded to 0.97 kg·m² stretched out. A constant D gives ζ from 0.2 to 0.72. Computing `D_i = 2ζ·sqrt(K_i·M_ii(q))`
  online keeps ζ constant. The other joints vary by at most 2×, so constants are fine for them.
- **Rotor inertia matters most on axis2 and the wrist.** It is about 20% of axis2's inertia (ratio 30) and 25% of
  axis4's. On axis6 it is almost all of it (97%), because the link itself is tiny and the differential doubles
  the reflected inertia. The LA8308 value is an estimate. Measuring it moves the axis2 numbers the most.
- **axis6 is effectively undamped.** With D = 0.002 and M = 7.5e-4, the release decay is ~370 ms. That is only fine
  if you want the roll joint to coast freely.
- **The joints are strongly coupled** (off-diagonal up to 0.7 of the diagonal). The per joint view is a ballpark.
  A Cartesian impedance should shape the damping with the task space inertia `Λ = (J·M⁻¹·Jᵀ)⁻¹`, not per joint.
- **The 500 Hz loop is not the limit at these gains.** `D·dt/M_min` is at most 0.1, so doubling D stays well
  within the stable range. What will limit stiffness on the hardware is the delay, the velocity noise and the
  gearbox compliance and backlash. Find that limit by increasing K until the joint chatters and backing off ~3×.

## Push test in the sim

`ros2 run robotarm_mujoco push_test` measures the response in the running sim instead of predicting it:
it pushes a body with a defined force, releases it and fits overshoot, ζ and settling time per joint,
next to the values expected from the tables above. Usage and how to read it:
[robotarm_mujoco README](../../src/robotarm_mujoco/README.md#push-test).

Results 2026-10-11, soft gains (`hold_stiffness: [20, 50, 30, 4, 1, 1]`,
`joint_damping: [5.5, 12.5, 4.0, 0.42, 0.16, 0.055]`):

- Every joint that responded on its own settled without overshoot (ζ ≥ 1) in about 0.25–0.4 s,
  consistent with the expected ζ of 0.8–1.0.
- axis1 with a sideways wrist push: measured ζ 0.71 against 0.71 expected from `M(q)`.
- axis5, 0.5 N at a 10 cm lever (0.049 Nm): 0.0488 rad deflection, exactly `τ / K` with K = 1 Nm/rad,
  returned without overshoot in 0.38 s.

## Rerun the sampling

Needs the sourced workspace (`robotarm_mujoco` is imported). By default it reads `sim_params.yaml` and
`controllers.yaml` from the source tree, not the installed copies:

```bash
source install/setup.bash
python3 docs/dynamics/mass_matrix_sampling.py --tool none
python3 docs/dynamics/mass_matrix_sampling.py --tool axis_triad
```

`--armature axis5=8e-4 ...` overrides a joint's armature for what-if runs. Other options: `--samples`,
`--zeta 0.5 0.7 1.0`, `--rate`, `--seed`, `--params`, `--controllers`.
