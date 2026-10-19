# Cartesian impedance control: theory notes

Learning notes from working out the control law of the `cartesian_impedance_controller`
([CartesianImpedanceController.cpp](../../src/robotarm_controllers/src/CartesianImpedanceController.cpp)).
Each section ends with the misconception it cleared up, since those were the parts worth writing down.

Conventions used throughout:

- poses, twists and wrenches in the **base frame**, rows ordered `[linear; angular]` like the rbd Jacobian
- `q`, `q̇` measured, `q_d`, `q̇_d`, `q̈_d` reference from the upstream controller
- error is **current − target** (mechanics convention), see [section 2](#2-the-pose-error)

---

## Contents

- [Design decisions](#design-decisions)
- [1. From a wrench to joint torques](#1-from-a-wrench-to-joint-torques)
- [2. The pose error](#2-the-pose-error)
- [3. Velocities and damping](#3-velocities-and-damping)
- [4. Feedforward](#4-feedforward)
- [5. The closed loop](#5-the-closed-loop)
- [6. Why no inertia shaping](#6-why-no-inertia-shaping)
- [7. Accuracy and future extensions](#7-accuracy-and-future-extensions)
- [8. Damping from a damping ratio](#8-damping-from-a-damping-ratio)
- [Open items](#open-items)

---

## Design decisions

| Topic | Decision |
|---|---|
| Control law | impedance **without inertia shaping** (no F/T sensor) |
| Modes | `gc`: gravity compensation, joint damping, breakaway hold. `impedance`: gravity compensation, joint damping, cartesian spring/damper, no hold |
| Translational only | no separate mode, a soft rotation is a low `K_rot` (may be 0) |
| Reference | joint space `q_d`, `q̇_d`, `q̈_d` through controller chaining (JointStream, CartesianJog, later JTC/MoveIt) |
| No upstream | NaN in a reference → hold the activation pose, NaN velocity/acceleration → 0 |
| Feedforward | `RNEA(q, q̇, q̈_d)`, measured `q̇` for Coriolis, fall back to `q̇_d` if the noise is too much |
| Damping | on the velocity error `ẋ − ẋ_d`, never on `ẋ` alone. `D(q)` from one damping ratio `ζ`, the same in every mode and pose ([section 8](#8-damping-from-a-damping-ratio)) |
| Safety | NaN check, torque rate limit (`torque_rise_time`), torque clamp, later error saturation and joint limit protection |

---

## 1. From a wrench to joint torques

The goal is a TCP that behaves like a mass-spring-damper attached to a target pose. Springs and dampers produce a
**wrench** `F = [Fx, Fy, Fz, Mx, My, Mz]ᵀ`, the motors produce joint torques `τ`.

**Virtual work.** The power the motors put in equals the power at the TCP:

```
τᵀ·q̇ = Fᵀ·ẋ = Fᵀ·J·q̇ = (Fᵀ·J)·q̇      for every q̇
→ τᵀ = Fᵀ·J
→ τ  = Jᵀ·F                            ((A·B)ᵀ = Bᵀ·Aᵀ)
```

`J` moves velocities from the joints to the TCP, `Jᵀ` moves forces back. The dimensions force the transpose: for the
translational part only, `J_v` is 3×6 and only `J_vᵀ·F` (6×3 · 3×1) gives six torques.

**Per joint** `τ_i = j_i · F`, the dot product of column `i` (how joint `i` moves the TCP) with the force. A joint that
moves the TCP perpendicular to `F` gets no torque, the bearings carry that part.

**Singular values** show both mappings in one picture, `J = U·Σ·Vᵀ`:

```
velocities:  q̇ = J⁻¹·ẋ = V·Σ⁻¹·Uᵀ·ẋ     scales with 1/σ
forces:      τ = Jᵀ·F  = V·Σ·Uᵀ·F       scales with σ
```

Same directions, reciprocal scaling (kinetostatic duality). A pose behaves like a gearbox: fast and weak in one
direction, slow and strong in another. Straight legs hold a heavy load with no effort but cannot push further up.

> **Aha: forces are not velocities run backwards.**
> At the stretched arm, jogging straight down needs huge joint velocities (`1/σ → ∞`), which is why the jog
> controller needs damped least squares. The same direction as a force gives **no torque at all** (`σ → 0`): the
> lever arms of axis2, 3 and 5 are parallel to the force, the structure carries it. The arm does fold eventually,
> but because the straight pose is an unstable equilibrium (a pencil pressed onto its tip), not because of large
> torques. Consequence: `τ = Jᵀ·F` needs no inversion and never blows up at a singularity, the spring just loses
> its effect in the singular direction.

---

## 2. The pose error

Current and target pose from forward kinematics, both in the base frame:

```
T   = FK(q)     → p,   R
T_d = FK(q_d)   → p_d, R_d
```

**Position:** `e_p = p − p_d` in m.

**Orientation:** neither `R − R_d` (not a rotation) nor a difference of RPY angles (wrap around, coupled axes, gimbal
lock) works. Instead:

```
R_err = R·R_dᵀ                       rotation from target to current, axis in the base frame
e_o   = θ·k                          axis-angle (log map), θ ∈ [0, π], in rad
```

```cpp
Eigen::AngleAxisd aa(R * R_d.transpose());
e_o = aa.angle() * aa.axis();
```

`e = [e_p; e_o]`, so `K = diag(K_p, K_p, K_p, K_o, K_o, K_o)` with `K_p` in N/m and `K_o` in Nm/rad.

**Sign conventions.** Both give the same force, only the bookkeeping differs:

```
control:    e = x_d − x,   u = +K·e        (CartesianJog drift correction: x_ref · x_curᵀ)
mechanics:  e = x − x_d,   F = −K·e        (this controller: R · R_dᵀ)
```

The mechanics convention keeps `Λ·ë + D·ė + K·e = F_ext` with all positive coefficients, like the impedance
literature. `R_d·Rᵀ` is the inverse rotation and gives exactly `−e_o`. `R_dᵀ·R` is **not** the negative, it is the
same rotation expressed in the target frame.

**The 180° flip.** Past 180° the log map takes the short way: `190° about +k` becomes `170° about −k`. The magnitude
becomes `360° − θ` and the axis flips, so the spring moment jumps from `+K_o·π` to `−K_o·π` and the spring now pulls
forward to complete the turn.

> **Aha: the log map swallows the wrap because a rotation matrix has one value per orientation.**
> `yaw = 179°` vs `−179°` is a 358° error with RPY and the spring would turn the long way around. The matrices are
> almost identical, so `R·R_dᵀ` is a 2° rotation.

> **Aha: don't unwrap the 180° flip, make it harmless.**
> Tracking the angle beyond 180° adds state with no clear meaning (after 350° the "right" way back is ambiguous)
> and would pull 350° back at full force. Winding only exists in joint space. Instead: saturate the error
> (`|e_o| ≤ e_o_max`, the jump becomes `2·K_o·e_o_max`), add joint limit protection, and abort on a persistently
> large error. The wrist motors only have 4–5 Nm, so turning the wrist past 180° by hand is realistic.

---

## 3. Velocities and damping

A spring alone stores energy and oscillates. A damper `F = −D·ė` dissipates `ėᵀ·D·ė ≥ 0`, energy can only decrease.

The velocity error is **computed, not differentiated**:

```
ẋ   = J(q)·q̇          current twist [v; ω]
ẋ_d = J(q_d)·q̇_d      target twist, J at the reference pose
ė   = ẋ − ẋ_d         both parts: [v − v_d; ω − ω_d]
```

For rotation `ω − ω_d` is not the exact time derivative of `e_o = θ·k` (the axis can turn), only for small errors.
It is still the right quantity: the power of a moment is `Mᵀ·ω`. Calling it `dx_err` in code avoids the confusion.

**Damp the error, not the velocity.** With a reference moving at constant `v_d`, in steady state:

```
damping on ẋ:          K·e = D·v_d   →  e = D·v_d / K       (50 · 0.1 / 500 = 1 cm lag)
damping on ẋ − ẋ_d:    K·e = 0       →  e = 0
```

What remains is every force the feedforward does not know (friction, model error): `e ≈ F_unmodeled / K`.

**Joint damping stays in impedance mode.** At a singularity there are joint motions that do not move the TCP
(axis4 and axis6 counter-rotating at the wrist singularity). `e` and `ẋ` do not change, so the cartesian spring and
damper do nothing. Gravity compensation keeps the axes from falling but does not hold them, the null space motion is
neutrally stable. Only `D_j` brakes it.

**Release behavior** after pulling the TCP away: `D = 0` oscillates until friction stops it, small `D` oscillates
with decaying amplitude, critical damping returns fastest without overshoot, large `D` creeps back with time constant
`≈ D/K`. In discrete time `D` cannot grow without bound either, the limit is roughly `D·dt / mass`, which hits the
light wrist rotation first.

> **Aha: the steady state lag does not depend on the inertia.**
> At constant velocity nothing accelerates, so only spring against damper counts: `K·e = D·v`. The inertia only
> decides how quickly the lag builds up.

---

## 4. Feedforward

The spring only reacts to deviations. Feedforward supplies the torques the **intended** motion needs, so the spring
only handles what is left.

```
RNEA(q, v, a) = M(q)·a + C(q, v)·v + g(q)
τ_ff          = RNEA(q, q̇, q̈_d)
```

| Argument | Choice | Why |
|---|---|---|
| position | measured `q` | gravity acts on the real pose |
| velocity | measured `q̇` | compensates the real Coriolis forces, like gravity. `C(q,q̇)·q̇_d` would be the textbook (PD+) term, which one RNEA call cannot produce. Fall back to `q̇_d` if the measured noise is too much |
| acceleration | reference `q̈_d` | anticipates the intended motion |

`J̇` does not appear: the references are already in joint space, the conversion is the upstream controller's job.

> **Aha: gravity at the reference pose acts like an extra, pose dependent spring.**
> With `g(q_d)` and the arm stretched out horizontally, pushing axis2 down 20° overcompensates (gravity is largest
> at horizontal) and pushes back up, stiffer. Pushing up 20° also overcompensates, and now pushes away, softer. In
> other poses the error is a spring `−∂g/∂q` that can be negative: with the arm pointing straight up, `g(q_d) = 0`,
> and pushed over the uncompensated gravity pulls it further, an inverted pendulum.

> **Aha: measured acceleration in the feedforward is positive feedback.**
> `M·q̈_measured` pushes in the direction the robot already accelerates, with exactly the force its inertia would
> resist with: `M·q̈ = M·q̈ + rest` → the robot feels massless. Any model error or delay makes the effective mass
> negative, unstable. Rule: **feedforward uses reference values** (what should happen), state compensation (gravity,
> Coriolis) uses measured values (what is).

---

## 5. The closed loop

Full control law:

```
τ = RNEA(q, q̇, q̈_d) + Jᵀ·(−K·e − D·ė) − D_j·(q̇ − q̇_d)
```

Into `M·q̈ + C·q̇ + g = τ + Jᵀ·F_ext`, gravity and Coriolis cancel:

```
M·(q̈ − q̈_d) + D_j·(q̇ − q̇_d) = Jᵀ·(−K·e − D·ė + F_ext)
```

**To cartesian space** (drop `D_j` and the `J̇` terms, `Δq̈ = J⁻¹·ë`, `F = −K·e − D·ė + F_ext`):

```
M·J⁻¹·ë = Jᵀ·F                 Jᵀ cannot just be dropped, multiply both sides by J⁻ᵀ
J⁻ᵀ·M·J⁻¹·ë = F
Λ := J⁻ᵀ·M·J⁻¹ = (J·M⁻¹·Jᵀ)⁻¹   (second form also works for a non square J)

Λ(q)·ë + D·ė + K·e = F_ext
```

`Λ` is the joint inertia seen from the TCP, defined so the kinetic energy matches: `½·q̇ᵀ·M·q̇ = ½·ẋᵀ·Λ·ẋ`. At a
singularity `J⁻¹` grows, so `Λ` grows in the singular direction: the TCP feels rigid there, the same effect as in
section 1. `D_j` shows up as the extra anisotropic damping `J⁻ᵀ·D_j·J⁻¹`.

| Property | Result |
|---|---|
| statics | exact, `K·e = F_ext` (1 kg at the TCP with `K_p = 500` sags 2 cm in every pose) |
| stiffness, damping force | exactly as chosen |
| mass | `Λ(q)`, pose dependent, anisotropic, couples translation and rotation |
| `ω_n = √(K/Λ)` | varies with pose and direction |
| `ζ = D / (2·√(K·Λ))` | with a constant `D` it varies too: 4× the mass → `ζ` halves, critical damping becomes ~16% overshoot. Fixed by computing `D(q)` from `Λ(q)`, [section 8](#8-damping-from-a-damping-ratio) |

**Stability.** With a stationary reference the total energy

```
V = ½·q̇ᵀ·M·q̇ + ½·eᵀ·K·e
dV/dt = −ẋᵀ·D·ẋ − q̇ᵀ·D_j·q̇ ≤ 0
```

only decreases (`Ṁ − 2C` is skew symmetric, so Coriolis does no work). The controlled robot is **passive**: coupled
to any passive environment (table, wall, hand) it stays stable. A reference 1 cm inside a table presses with
`K·0.01 m` and stays there, where a stiff PI position controller would wind up to the torque limit or bounce.
Passivity does not make the impact itself gentle, that is set by `Λ`.

The proof breaks with discrete time and delay (bounded `K`, `D`), saturation (clamp, rate limit), gravity model error
(offset or negative stiffness) and a moving reference (PD+ term, irrelevant at slow speeds).

> **Aha: the TCP is not a point mass.**
> `F = m·a` with a scalar `m` keeps `a ∥ F`. The TCP mass is the matrix `Λ`, which scales **and rotates**:
> `ẍ = Λ⁻¹·F`. Example `Λ = [[2, 1], [1, 2]]` kg, `F = [0, −10]` N → `ẍ = [3.3, −6.7]` m/s², 27° off vertical.
> Heavy along the arm (whole arm swings), light across it (forearm only), so the acceleration tilts towards the
> light direction. Since `Λ(q)` changes along the way and `J̇·q̇`, `C·q̇` grow with speed, paths curve: a straight
> pull from the spring does not give a straight return.

---

## 6. Why no inertia shaping

Natural dynamics vs the desired ones:

```
Λ·ë   = F_ctrl + F_ext
Λ_d·ë = −K·e − D·ė + F_ext

→ F_ctrl = Λ·Λ_d⁻¹·(−K·e − D·ė) + (Λ·Λ_d⁻¹ − I)·F_ext
```

The last term vanishes only for `Λ_d = Λ`, which is option A. Any other `Λ_d` needs `F_ext` directly.

> **Aha: displacement and velocity come after the inertia has already acted.**
> Inertia decides how a force becomes motion. To change that, the force has to be known when it acts, before it
> turns into motion. Reconstructing it from the measured acceleration is the positive feedback trap from section 4
> again. A momentum observer can estimate `F_ext` from motor torques without differentiating, but its lag and
> friction errors break passivity as soon as the apparent mass is reduced.

What can be done instead is in [section 7](#7-accuracy-and-future-extensions).

---

## 7. Accuracy and future extensions

### What is accurate and what is not

Assuming a correct model, option A gives:

| Accurate | Not accurate |
|---|---|
| static stiffness: `K·e = F_ext` in every pose | apparent mass: `Λ(q)` instead of a designed `Λ_d`, pose dependent, anisotropic, coupled |
| stiffness and damping **force** laws as designed | natural frequency `ω_n` varies with pose and direction |
| damping ratio `ζ` in every mode and pose ([section 8](#8-damping-from-a-damping-ratio)) | `ζ` only as good as `Λ`: armature still 0 in the URDF |
| no tracking lag from the damper (velocity error) | return paths curve (`Λ` coupling, `J̇·q̇`, `C·q̇`) |
| stable and passive, also in contact | contact impact is set by `Λ`, not shaped |
| no blow up at singularities (`Jᵀ`, no inverse) | spring has no effect in a singular direction, null space motion only damped by `D_j` |

What the model itself gets wrong, and shows up as a static error `e ≈ F_unmodeled / K` or as a wrong dynamic:

| Source | Effect |
|---|---|
| joint friction (gears, belts) | tracking lag and a dead band around the target, usually the largest error |
| mass / CoM error | static gravity offset, can act as negative stiffness in some poses |
| reflected rotor inertia (`armature`) still 0 in the URDF (placeholder) | inertia feedforward too small (axis2 ~21% of `M`, see [docs/dynamics](../dynamics/README.md)), `Λ` ~20–30% too light, so the real `ζ` is lower than configured |
| Coriolis from one RNEA call | `C(q,q̇)·q̇` instead of the PD+ term `C(q,q̇)·q̇_d`, irrelevant at slow speeds |
| velocity noise | amplified by `D` (and quadratically by the Coriolis term), limits how large `D` can be |
| log map past 180° | spring flips direction, limited by error saturation |

### Future extensions

Roughly ordered by effort and benefit.

| Extension | What it fixes | Needs | Caveats |
|---|---|---|---|
| **armature values** | inertia feedforward and `Λ` for the damping | rbd side done (`<armature>` per joint, `τ_i += armature_i·q̈_i` in the RNEA). Open: real values in `ORT.yaml` (placeholders 0, waiting for the robot changes to be final), MuJoCo reading them from the URDF instead of `sim_params.yaml` | rotor inertia values are estimates, too high costs almost nothing, too low lowers `ζ` |
| **friction compensation** | lag and dead band from friction | friction model per joint (Coulomb + viscous), identified on the robot | overcompensation adds energy, keep it below the real friction |
| **stiffness in the tool frame** | intuitive rotation stiffness ("soft about the tool axis") | `K_base = R₆·K_tool·R₆ᵀ` per cycle (`R₆ = diag(R, R)`), the spring as a matrix product. The damping already handles a non diagonal `K` | switching the frame with an existing error changes the force direction, blend it |
| **stiffness changes at runtime** | new `K` (or frame) without deactivating, which restarts the ramp and drops the upstream | a service (stiffnesses, `ζ`, frame, transition time), handed to `update()` through a `RealtimeThreadSafeBox` | `ΔK·e` steps the spring force: blend old to new over the transition time, apply a request as one set, reject in `gc` mode. The damping follows `K` on its own |
| **momentum observer** | estimate of `F_ext` without a sensor, also usable for collision detection | measured joint torques (`state.tau`), `M`, `C`, `g` | lag (limited bandwidth), friction shows up as phantom forces |
| **inertia shaping without sensor** | designed apparent mass | observer estimate of `F_ext` | only **increasing** the mass is safe. Reducing it with a lagging estimate injects energy, passivity is lost |
| **inertia shaping with an F/T sensor** | designed, pose independent, decoupled apparent mass: straight return paths, same feel everywhere | wrist F/T sensor, `Λ`, `J̇·q̇` | sensor noise and delay, sensor sees only forces beyond the wrist, full passivity needs care when reducing the mass |

---

## 8. Damping from a damping ratio

The goal: one damping ratio `ζ` that holds in every pose and every direction, although `Λ(q)` changes (5–7× in
translation, 17–25× in rotation over the workspace, [task space inertia](../dynamics/task_space_inertia.md)). Measured
there before deciding: worth it, and the full version below rather than a per-axis shortcut.

### One mass, one spring, one damper

```
m·ẍ + d·ẋ + k·x = 0       try x = e^(s·t):   m·s² + d·s + k = 0
s = −d/(2m) ± √((d/2m)² − k/m)
```

The guess turns derivatives into multiplications by `s` (the same polynomial is the denominator of the Laplace
transfer function, its roots are the poles). `Re s` is the decay rate, `Im s` the oscillation frequency. The square
root decides: comparing `d/(2m)` with `ω_n = √(k/m)` gives

```
ζ = d / (2·√(k·m))        d = 2·ζ·√(k·m)
```

`ζ < 1` rings (overshoot 37 % at 0.3, 16 % at 0.5, 4.6 % at 0.7), `ζ = 1` returns fastest without overshoot, `ζ > 1`
creeps (slow pole at `ω_n·(−ζ + √(ζ²−1))`). Same `ζ` means the same response shape, only stretched in time by
`1/ω_n`. So `ζ` is the setting, `d` only the number that produces it for one `m`.

> **Aha: same D, heavier mass, worse in every way.**
> Tuned for `ζ = 0.7` at 2.6 kg, the same `D` gives `ζ = 0.3` at 13.8 kg: 37 % overshoot, half the frequency, and a
> 5× slower decay, since the decay rate `d/(2m)` does not depend on the spring. Tuned for the heavy pose instead, the
> light one creeps at `ζ = 1.6`. A constant `D` only chooses where it goes wrong.

### The mass matrix

```
τ = M(q)·q̈ + C(q, q̇)·q̇ + g(q)
```

`M` is torque **per** acceleration (kg·m²), `τ = M·q̈` exactly, since inertial forces are linear in the acceleration.
Column `i` is the torque at every joint when only joint `i` accelerates with 1 rad/s² and all others are held at
zero acceleration. `M_ji` is the torque joint `j` needs to **hold still** (swing the forearm fast, the shoulder has to
brace). `M` is `n×n` for `n` joints, symmetric and positive definite in every pose, from the kinetic energy

```
T = ½·q̇ᵀ·M·q̇ = ½·Σ_k (m_k·|J_v,k·q̇|² + (J_ω,k·q̇)ᵀ·I_k·(J_ω,k·q̇))      every term Aᵀ·B·A with B symmetric
```

The rotor turns `N` times faster than the joint, so it adds `J_rotor·N²` on the diagonal (armature). Built from the
RNEA: `q̇ = 0` removes Coriolis, gravity off removes `g`, `q̈ = e_i` leaves column `i`. Then `½·(M + Mᵀ)` against
rounding.

> **Aha: `M` never loses rank, `J` does.**
> Every motion has positive kinetic energy, so `M` is always invertible. A singularity is purely kinematic: `Λ` blows
> up because `J` loses rank, not because of the masses.

> **Aha: forgetting `gravity = 0` breaks more than the symmetry.**
> Every column gets `g(q)` added, tens of Nm against entries below 1 kg·m², so `M` stops being positive definite.
> Symmetrizing hides the asymmetry but not the error, so test poses need large gravity torques (and check `M − Mᵀ`
> before symmetrizing).

### What `Λ` means entry by entry

`F = Λ·ẍ`: `Λ_ii` is the force along `i` per acceleration along `i` with the **other axes held**, `Λ_ji` the force
needed in `j` to hold it. With the others **free**, a unit force gives `a = Λ⁻¹·e_i`, so the felt mass is
`1/(Λ⁻¹)_ii`, which for `Λ = [[a, b], [b, c]]` is `a − b²/c ≤ a`: freeing a constraint never makes motion harder. Off
diagonals are couplings, not masses: `Λ_xy = 0` means pushing along `x` does not swerve into `y`.

> **Aha: rotating the TCP in place is 10× heavier than turning a wrist joint.**
> Held position means the arm has to move along, because the wrist axes don't pass through the TCP. In the pose where
> axis6 lies along a base axis and through the TCP, `Λ_rz` drops to exactly the axis6 inertia (7.5e-4 kg·m²), which
> is why rotation spreads so much more than translation.

### Why damping per axis lands too low

`D_i = 2ζ·√(K_i·Λ_ii)` treats every axis as a 1D system. Exact when `Λ` and `K` are both diagonal, otherwise the
modes are mixed directions. Example `Λ = [[1, 0.8], [0.8, 1]]`, `K = I`: modes `(1, 1)` with mass 1.8 and `(1, −1)` with
0.2, both damped as if they had mass 1, so `ζ = 0.52` and `1.57` for a target of 0.7. A diagonal entry always lies
between the smallest and largest eigenvalue, so the heaviest mode is always under-damped. On the arm: 0.36–0.48, at a
level that depends on the stiffness ratio.

### Double diagonalization

Find coordinates where mass **and** stiffness are diagonal. A rotation can only diagonalize one symmetric matrix at a
time, so first turn the mass into `I`, the one matrix that stays diagonal under every rotation:

```
a)  Λ = L·Lᵀ (Cholesky),  z = Lᵀ·x       mass I,  stiffness K̃ = L⁻¹·K·L⁻ᵀ
b)  K̃ = V·diag(k0)·Vᵀ,     w = Vᵀ·z       mass I,  stiffness diag(k0),  k0 = ω² per mode
c)  d_i = 2ζ·√k0_i                         each mode a unit mass, damped exactly
d)  D = (L·V)·diag(d)·(L·V)ᵀ               back: same dissipated power ẋᵀ·D·ẋ = ẇᵀ·diag(d)·ẇ
```

Step a is the matrix version of measuring position in units of `√m` (`z = √m·x` turns `m·ẍ + k·x = 0` into
`z̈ + (k/m)·z = 0`). Step b uses that a symmetric matrix has perpendicular eigenvectors: displaced along one, the
spring pushes back exactly along it, which is what a mode is. Step d is the same pattern as `τ = Jᵀ·F`: velocities go
in with a matrix, forces come back with its transpose. Check: `(L·V)⁻¹·Λ·(L·V)⁻ᵀ = I`, `(L·V)⁻¹·D·(L·V)⁻ᵀ = diag(d)`.

> **Aha: step a messes up `K` too, that's fine.**
> `K` was diagonal and `K̃` is not. But after step a the mass is `I`, so step b is free to rotate for `K̃` alone.
> The other order fails: `K` is already diagonal, step a would still couple it.

> **Aha: one `ζ` for all modes is what makes `D` smooth.**
> Then `D = 2ζ·L·√K̃·Lᵀ`, a matrix square root, unique and continuous even where two `k0` cross and `V` is not
> unique. A `ζ` per mode would let `D` jump when the modes swap order.

### At singularities

```
Λ = (J·M⁻¹·Jᵀ + λ²·I)⁻¹
```

`J·M⁻¹·Jᵀ` is the inverse mass, bounded, only small in the singular direction. Adding `λ²` there caps `Λ` at
`1/λ²` and changes it by `≈ λ²·Λ` elsewhere (0.05 % at 4.6 kg).

> **Aha: the damped `J⁺` is the wrong inverse here.**
> `Λ = J⁻ᵀ·M·J⁻¹` needs the inverse to be **large** in the singular direction, that is what makes the arm heavy. The
> damped `J⁺` of the jog controller shrinks it on purpose (`σ/(σ² + λ²) → 0`), so `Λ` would peak near `σ ≈ λ` and
> drop back to 0 at the singularity, and the Cholesky would fail exactly there.

> **Aha: `Λ` blows up, the motors don't feel it.**
> The motors produce `Jᵀ·D·J`. In the singular direction `√Λ ~ 1/σ` and `J·J ~ σ²`, so the joint damping goes like
> `σ → 0`, the same place where the spring has no effect. For 6 joints the whole problem can be written in joint
> space with `M` and `Jᵀ·K·J`, both bounded, so it has to stay bounded. Sampled: at most 13.7 Nm·s/rad near
> singularities against 12.3 in regular poses. Per-axis damping reaches 875, because its diagonal `D` is not aligned
> with the singular direction and `J` does not cancel it.

### In the controller

`RobotarmRbd::calculate_task_space_inertia` returns `Λ` (Cholesky of `M`, `M⁻¹·Jᵀ` by solve, Cholesky of the inverse
mass). The controller does steps a–d with fixed size 6×6 types, about 7 µs per cycle. A wrong armature shifts `ζ` by
a nearly constant factor (half the armature: 0.51 instead of 0.70; double: 0.72), so estimate it high. Zero stiffness
gives `k0 = 0` and no Cartesian damping in that mode, `D_j` brakes it.

---

## Open items

Done and documented in the [controller README](../../src/robotarm_controllers/README.md#cartesianimpedancecontroller):
mapping to the code (explicit `Jᵀ`, norm clamped spring, NaN references), joint limit protection, damping from a
damping ratio.

1. **Armature:** real values in the URDF once the robot changes are final, one source for the controller and the
   simulation
2. **Validation of the damping:** release tests with `push_test` in several poses in the simulation (the measured `ζ`
   should match everywhere), then on the hardware
3. **Tuning:** `K`, `ζ`, `joint_damping` (still the gc values, too large for impedance mode), check the
   `torque_rise_time` is never active in normal use
4. **Later:** see [future extensions](#future-extensions), next in line are the tool frame and stiffness changes at
   runtime
