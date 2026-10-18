# Task space inertia and Cartesian damping ballpark

Snapshot of the task space inertia `Λ(q)` at the TCP of the MuJoCo model, used to decide on mass-adaptive
damping for the `cartesian_impedance_controller`
([NEXT_STEPS.md, part 1](../cartesian_impedance/NEXT_STEPS.md#1-mass-adaptive-damping)). Sampled 2026-10-14
from [`task_space_inertia_sampling.py`](task_space_inertia_sampling.py), with the gains of the
`cartesian_impedance_controller` in `controllers.yaml` at that time:

```
cartesian_stiffness: [200, 200, 200, 5, 5, 5]          # N/m, Nm/rad
cartesian_damping:   [28, 28, 28, 0.22, 0.22, 0.22]    # N·s/m, Nm·s/rad
joint_damping:       [5.5, 12.5, 4.0, 0.42, 0.16, 0.055]
lambda: 0.01
```

**These gains are placeholders**, copied from the gc mode estimate and not tested. Results that depend on them
(how well the current `cartesian_damping` fits, how much `joint_damping` dominates) describe these placeholders,
not a tuned controller. The spread of `Λ`, the armature sensitivity, the comparison of the damping laws and the
behavior near singularities do not depend on the gains (the stiffness only enters through the ratio of
translation to rotation, see [Other stiffnesses](#other-stiffnesses)).

The joint space counterpart is the [README](README.md) in this folder.

## Method

Per Cartesian axis `i` (base frame, `[x, y, z, rx, ry, rz]`) the controller is close to a mass-spring-damper:

```
Λ   = (J·M⁻¹·Jᵀ + λ²·I)⁻¹      task space inertia, regularized
ζ_i = D_i / (2·sqrt(K_i·Λ_ii))  damping ratio, coupling of Λ ignored
f_n = sqrt(K_i / Λ_ii) / 2π     natural frequency
```

`Λ_ii` is the inertia felt when accelerating along axis `i` **with the other five axes held**. That matches a
6D impedance, which holds the orientation while you push the position and the other way round. `M(q)` comes from
`mujoco.mj_fullM` (armature included), `J` from `mj_jacSite` at the `tcp` site. The `λ²·I` term reuses the
controller's `lambda` as a plain regularization. The impedance law itself only uses `Jᵀ`, never the damped
`J⁺` of `robotarm_rbd`, so `lambda` has no effect in impedance mode today.

The per axis view ignores the coupling of `Λ` (off-diagonal terms). For the full picture the script also builds
the linearized closed loop in joint space and takes its eigenvalues:

```
M·q̈ + (Jᵀ·D·J + D_joint)·q̇ + Jᵀ·K·J·q = 0
```

From the modes it reports the **smallest modal ζ** (the worst-ringing mode, 1 = no mode oscillates) and the
**slowest decay time constant** `τ_slow = 1/|Re λ|` (how long the slowest mode takes to settle). Linearized
around the pose: no spring clamp, no friction, perfect gravity compensation, no velocity noise or delay.

Damping laws compared, all with one ζ for every axis:

| law | D |
|---|---|
| constant D | `cartesian_damping` from the yaml |
| simplified, `Λ_ii` | `D_i = 2·ζ·sqrt(K_i·Λ_ii(q))`, diagonal |
| simplified, `1/(Λ⁻¹)_ii` | same, with the inertia felt when the other axes are free, cheaper (no 6×6 inverse) |
| full, generalized eig | `D = 2·Qᵀ·diag(ζ·sqrt(k0))·Q` with `Λ = QᵀQ`, `K = Qᵀ·diag(k0)·Q` (double diagonalization), full 6×6 |

The adaptive laws are also evaluated with the armature in the **controller's** model scaled by ×0.5 and ×2, while
the plant keeps the nominal armature. That is the case "rotor inertias wrong in the controller".

Poses are sampled uniformly within the joint limits. Poses with any contact (self collision, below the floor) are
rejected and redrawn (158 of 2158). 2000 poses, seed 0, target ζ = 0.7.

**Regular poses:** `σ_min(J) ≥ 0.05` (57 % of the samples). **Near singular:** `σ_min(J) < sqrt(2)·λ = 0.014`
(15 %), where the damped jacobian inverse of `robotarm_rbd` starts damping. Uniform sampling hits singular poses
far more often than real use, so the regular subset is the one to read for the workspace.

## Results, no tool, regular poses

| axis | unit | min | p5 | median | p95 | max | p95/p5 | armature share (median) |
|---|---|---|---|---|---|---|---|---|
| x | kg | 1.93 | 2.63 | 4.62 | 13.8 | 33.9 | 5.2 | 18% |
| y | kg | 1.91 | 2.64 | 4.53 | 13.7 | 38.8 | 5.2 | 19% |
| z | kg | 1.87 | 1.97 | 4.88 | 14 | 29.4 | 7.1 | 30% |
| rx | kg·m² | 0.000824 | 0.00961 | 0.0754 | 0.241 | 0.548 | 25.1 | 32% |
| ry | kg·m² | 0.00103 | 0.00993 | 0.0751 | 0.227 | 0.542 | 22.9 | 32% |
| rz | kg·m² | 0.000752 | 0.0127 | 0.074 | 0.216 | 0.728 | 17.0 | 28% |

Armature share: how much `Λ_ii` drops when the armature is removed from `M`.

| axis | K | D now | ζ now p5 – p95 | f_n p5 – p95 [Hz] | D for ζ=0.7 p5 – p95 |
|---|---|---|---|---|---|
| x | 200 | 28 | 0.27 – 0.61 | 0.6 – 1.4 | 32.1 – 73.5 |
| y | 200 | 28 | 0.27 – 0.61 | 0.6 – 1.4 | 32.2 – 73.4 |
| z | 200 | 28 | 0.26 – 0.70 | 0.6 – 1.6 | 27.8 – 74 |
| rx | 5 | 0.22 | 0.10 – 0.50 | 0.7 – 3.6 | 0.307 – 1.54 |
| ry | 5 | 0.22 | 0.10 – 0.49 | 0.7 – 3.6 | 0.312 – 1.49 |
| rz | 5 | 0.22 | 0.11 – 0.44 | 0.8 – 3.2 | 0.353 – 1.45 |

Closed loop, **Cartesian damping only** (`joint_damping` = 0):

| damping law | armature in model | ζ p5 | ζ median | ζ p95 | τ_slow median [ms] | τ_slow p95 [ms] |
|---|---|---|---|---|---|---|
| constant D | – | 0.10 | 0.18 | 0.23 | 1407 | 4305 |
| simplified, `Λ_ii` | ×1 | 0.36 | 0.43 | 0.48 | 613 | 1167 |
| simplified, `Λ_ii` | ×0.5 | 0.33 | 0.39 | 0.45 | 662 | 1298 |
| simplified, `Λ_ii` | ×2 | 0.42 | 0.48 | 0.54 | 544 | 989 |
| simplified, `1/(Λ⁻¹)_ii` | ×1 | 0.06 | 0.10 | 0.16 | 2510 | 7609 |
| full, generalized eig | ×1 | 0.70 | 0.70 | 0.70 | 377 | 657 |
| full, generalized eig | ×0.5 | 0.50 | 0.51 | 0.51 | 410 | 732 |
| full, generalized eig | ×2 | 0.71 | 0.72 | 0.75 | 324 | 566 |

Simplified `Λ_ii` with the target raised to ζ = 1.1 (`--zeta 1.1`, 1000 poses): smallest modal ζ 0.57 / 0.66 /
0.74 (p5 / median / p95), τ_slow median 406 ms.

Closed loop, **with `joint_damping`** (the placeholder values above):

| damping law | armature in model | ζ p5 | ζ median | ζ p95 | τ_slow median [ms] | τ_slow p95 [ms] |
|---|---|---|---|---|---|---|
| `joint_damping` only | – | 0.33 | 0.34 | 0.38 | 2644 | 10577 |
| constant D | – | 0.99 | 1.00 | 1.00 | 2749 | 10675 |
| simplified, `Λ_ii` | ×1 | 1.00 | 1.00 | 1.00 | 2875 | 10874 |
| full, generalized eig | ×1 | 0.93 | 0.97 | 1.00 | 3000 | 11185 |

## Results, no tool, all poses (near singularities included)

| axis | unit | min | p5 | median | p95 | max | p95/p5 |
|---|---|---|---|---|---|---|---|
| x | kg | 1.93 | 2.77 | 6.16 | 281 | 7990 | 101 |
| y | kg | 1.91 | 2.89 | 6.25 | 366 | 7950 | 127 |
| z | kg | 1.86 | 2.04 | 6.06 | 258 | 9590 | 126 |
| rx | kg·m² | 0.000751 | 0.0139 | 0.108 | 5.11 | 1840 | 367 |
| ry | kg·m² | 0.000902 | 0.0132 | 0.109 | 5.01 | 968 | 380 |
| rz | kg·m² | 0.000752 | 0.0162 | 0.101 | 5.12 | 327 | 315 |

At the **zero pose** (stretched, wrist singular) `Λ_ii` is x 1370 kg, y 6560 kg, z 5.85 kg, rx 0.00068, ry 0.119,
rz 3450 kg·m². Smallest modal ζ, Cartesian damping only: constant D 0.01 – 0.22 (p5 – p95), simplified 0.33 –
0.49, full 0.65 – 0.70.

## Damping felt by the joints near singularities

`Λ` blows up near a singularity because `J` loses rank: one singular value σ → 0, `J·M⁻¹·Jᵀ` gets an eigenvalue
∝ σ², and its inverse grows ∝ 1/σ². `M` stays positive definite and bounded everywhere, it only sets the scale.
Physically: stretched out, no joint motion moves the TCP radially, so the arm feels infinitely heavy in that
direction.

What reaches the motors is not `D` but the joint space damping `Jᵀ·D·J` (`τ = Jᵀ·D·J·q̇`). Largest eigenvalue,
Nm·s/rad, target ζ = 0.7:

| damping law | regular: median | p95 | max | near singular: median | p95 | max |
|---|---|---|---|---|---|---|
| constant D | 9.6 | 15.5 | 18.4 | 14.8 | 18.8 | 19.3 |
| simplified, `Λ_ii` | 15.4 | 29 | 49.6 | 112 | 481 | 875 |
| simplified, `1/(Λ⁻¹)_ii` | 5.57 | 9.53 | 13.7 | 8.88 | 14.2 | 17.1 |
| full, generalized eig | 8.2 | 11.3 | 12.3 | 11.1 | 13.4 | 13.7 |

The full law limits itself: in the singular direction `sqrt(Λ)` grows like 1/σ, the two `J` around `D` shrink it by
σ², so `Jᵀ·D·J` there goes like σ → 0. Its `D` is aligned with the eigenvectors of `Λ`, so the cancellation is
exact. The simplified `Λ_ii` law puts the blown-up inertia on the diagonal of every axis that has a component in
the singular direction. Its diagonal `D` is not aligned with that direction, so `J` does not cancel it, and up to
~50× the normal damping leaks into joint directions that can move. The simplified law would need a clamp or a
much larger `λ`, the full law needs neither.

## Other stiffnesses

Smallest modal ζ in regular poses, Cartesian damping only, target ζ = 0.7, `--controllers` with a modified
`cartesian_stiffness`:

| K translation / rotation | simplified `Λ_ii` p5 / median / p95 | full p5 / median / p95 |
|---|---|---|
| 200 / 5 (placeholder) | 0.36 / 0.43 / 0.48 | 0.70 / 0.70 / 0.70 |
| 500 / 20 | 0.38 / 0.44 / 0.50 | 0.70 / 0.70 / 0.70 |
| 1000 / 5 | 0.39 / 0.45 / 0.52 | 0.70 / 0.70 / 0.70 |
| 100 / 30 | 0.47 / 0.55 / 0.64 | 0.70 / 0.70 / 0.70 |

The simplified law stays in a narrow band, but its level depends on the ratio of translational to rotational
stiffness, so its ζ setting has to be retuned when the stiffnesses change. The full law hits the target for any
stiffness.

## Runtime

Benchmark against the installed `robotarm_rbd_core` (Release, `-O3`), AMD Ryzen 7 5800H, 200 000 cycles over
1000 random poses. The benchmark is a throwaway program, not in the repo.

| work per cycle | mean | p99 |
|---|---|---|
| today's impedance rbd work (2 jacobians, 2 transforms, 2 RNEA) | 3.6 µs | 4.3 µs |
| `M(q)` alone (6 RNEA with `q̈ = e_i`, armature included by the RNEA) | 3.8 µs | 4.8 µs |
| simplified (`M`, `J`, `Λ`, diagonal `D`) | 5.3 µs | 7.1 µs |
| full, Cholesky of `Λ` + `SelfAdjointEigenSolver` of `L⁻¹·K·L⁻ᵀ` | 7.2 µs | 9.1 µs |
| full, `GeneralizedSelfAdjointEigenSolver(K, Λ)` | 7.4 µs | 10.1 µs |

Against 2000 µs per cycle at 500 Hz and the controller manager's warning at 100 µs mean, the full law is
negligible. A Raspberry Pi class CPU would be roughly 3 – 5× slower, still far below the limits. Single maxima of
30 – 160 µs were OS scheduling on a desktop kernel without isolated cores, not the computation.

## Takeaways

- **The pose dependence is large enough to matter.** In regular poses `Λ_ii` spreads 5 – 7× in translation and
  17 – 25× in rotation (p5 – p95), so any constant D spreads ζ by 2.3 – 2.7× and 4 – 5×. Tuned for the heavy
  poses, the light ones end up at ζ ≈ 1.6 – 1.9 (translation) and ≈ 2.9 – 3.5 (rotation), heavily overdamped and
  sluggish. This holds for any gains.
- **The arm is heavier than a guess suggests, mostly in rotation.** The median is 4.6 kg and 0.075 kg·m²,
  because turning the TCP in place also moves the arm. For ζ ≈ 0.7 at the median pose with the placeholder
  stiffness, a constant D would be about `[45, 45, 45, 0.85, 0.85, 0.85]`.
- **The armature uncertainty barely matters, and acts mostly as an offset.** Halving or doubling every rotor
  inertia moves the simplified law's ζ by about ±10 %, the full law's to 0.51 (×0.5, almost the same in every
  pose) or 0.72 (×2). The pose dependence comes from the link masses and the geometry. If unsure, estimate the
  armature high rather than low: too high costs almost nothing. Wrong link inertias (CAD) scale ζ with the square
  root of the error.
- **The full law is the better choice**, not only the more exact one: it hits the target ζ in every pose and for
  every stiffness, keeps the joint space damping bounded near singularities on its own, and costs ~7 µs per cycle.
  With one ζ for all axes, `D` is a matrix square root of `K` and `Λ` and changes smoothly with the pose.
- **The simplified law** has the right shape at a lower level (0.36 – 0.48 for a target of 0.7), the level depends
  on the stiffness ratio, and near singularities it puts up to ~50× the normal damping on the joints unless
  clamped. The `1/(Λ⁻¹)_ii` variant is useless (0.06 – 0.16).
- **`joint_damping` acts in parallel and can dominate.** With the placeholder values every law is overdamped
  (smallest ζ ≈ 1) and the Cartesian D hardly matters. The slowest mode is mostly an elbow motion (axis2 against
  axis3, the wrist compensating) that moves the TCP very little. The Cartesian spring barely restores it, while
  `joint_damping` on axis2/axis3 resists it fully, so it creeps back with a time constant of ~2.6 s (median, up to
  ~10 s). A 10× stiffer rotation only makes it 1.5× faster. When tuning, keep `joint_damping` small in impedance
  mode and let the Cartesian D carry the damping, otherwise the adaptive damping has no visible effect.

## Rerun the sampling

Needs the sourced workspace (`robotarm_mujoco` is imported). By default it reads `sim_params.yaml` and
`controllers.yaml` from the source tree:

```bash
source install/setup.bash
python3 docs/dynamics/task_space_inertia_sampling.py --tool none
python3 docs/dynamics/task_space_inertia_sampling.py --zeta 1.1 --armature-error 0.5 2
```

Other options: `--samples`, `--sigma-regular` (threshold for the regular poses), `--site`, `--seed`, `--params`,
`--controllers` (e.g. a copy with other stiffnesses). About 4 s for 2000 poses.
