# Cartesian impedance controller: next steps

Handoff for two planned extensions of `cartesian_impedance_controller`
([CartesianImpedanceController.cpp](../../src/robotarm_controllers/src/CartesianImpedanceController.cpp)).
Background and the control law: [README.md](README.md) (theory notes) and the controller section of
the [package README](../../src/robotarm_controllers/README.md).

---

## 1. Mass-adaptive damping

### Goal

Constant damping ratio $\zeta$ in every pose and direction. Today $D$ is constant, but the apparent
mass $\Lambda(q) = (J M^{-1} J^T)^{-1}$ changes with the pose, so
$\zeta = D / (2\sqrt{K\Lambda})$ changes too: the same release settles cleanly in one pose and
overshoots in another (4× the mass halves $\zeta$). No inertia shaping, only the damping follows the
mass, so no force/torque sensor is needed.

**Decided: worth it, full version.** The sampling in
[docs/dynamics/task_space_inertia.md](../dynamics/task_space_inertia.md) shows:

- $\Lambda_{ii}$ spreads 5–7× (translation) and 17–25× (rotation) over regular poses, so any constant $D$ spreads
  $\zeta$ by 2.3–2.7× and 4–5×.
- The full version hits the target $\zeta$ in every pose and for every stiffness ratio. The simplified per-axis
  version stays in a narrow band but below the target, at a level that depends on the stiffness ratio.
- A wrong armature in the model mostly shifts $\zeta$ by a near-constant offset (half the armature: 0.51 instead
  of 0.70), which the configured $\zeta$ absorbs. Double the armature costs almost nothing, so estimate it high.
- Near singularities $\Lambda$ explodes, but the full version keeps the joint space damping $J^T D J$ bounded on
  its own (max 13.7 Nm·s/rad near singular against 12.3 regular). The simplified version reaches up to 875.
- Runtime ~7 µs per cycle on top of today's ~4 µs.

### Approach

1. **Armature in `RobotarmRbd`** (prerequisite): **done in the rbd**, values open. Reflected rotor inertia per
   joint from the URDF `<armature value>` (set in `ORT.yaml`), added on the diagonal of $M$
   (`τ_i += armature_i · q̈_i` in the RNEA). It is a large share of $M$ (axis2 ~21 %, see
   [docs/dynamics](../dynamics/README.md)) and 18–32 % of $\Lambda$. Still open:
   - the URDF values are placeholders (0). Until they are set, the controller model has no armature at all,
     worse than the ×0.5 case sampled above. Put in the `sim_params.yaml` estimates or measured values.
   - **one source**: MuJoCo still takes its armature from `sim_params.yaml`. Read `<armature>` from the URDF
     in `mjcf.py` instead, so the sim and the controller can't drift apart.
2. **$M(q)$** from 6 RNEA calls ($\dot q = 0$, $g = 0$, $\ddot q = e_i$ gives column $i$), armature included.
3. **$\Lambda$ regularized**: $\Lambda = (J M^{-1} J^T + \lambda^2 I)^{-1}$. Near a singularity $\Lambda$ grows
   without bound in the singular direction ($J$ loses rank, $M$ stays bounded). The regularization only keeps
   the Cholesky in step 4 well conditioned; the full law does not need it to bound the damping. The impedance
   law only uses $J^T$, the damped $J^+$ of `robotarm_rbd` is not involved and `lambda` does nothing in
   impedance mode today.
4. **Damping from $\zeta$** by double diagonalization, with **one $\zeta$ for all axes** and the stiffness per
   axis: $\Lambda = L L^T$ (Cholesky), $L^{-1} K L^{-T} = V\,\operatorname{diag}(k_0)\,V^T$ (symmetric eigen
   decomposition), then $D = 2\zeta\,(L V)\,\operatorname{diag}(\sqrt{k_0})\,(L V)^T$. With a single $\zeta$ this is a
   matrix square root, so $D$ changes smoothly with the pose, even where eigenvalues of $L^{-1} K L^{-T}$ cross.
   A $\zeta$ per mode would make $D$ jump when the modes swap order, so it is left out.
5. **Parameter `damping_method`**: `constant` (today's `cartesian_damping`), `diagonal`
   ($D_i = 2\zeta\sqrt{K_i \Lambda_{ii}}$, coupling ignored) and `full`. The two adaptive laws share steps 1–3.
   Keeping all three allows a fallback and a measured comparison in the same controller.

### Important

- **Stability is not at risk** as long as $D(q)$ stays positive definite, also when it changes every
  cycle (the dampers still only dissipate). The full law is positive (semi)definite by construction.
- **Zero stiffness:** with $K_i = 0$ the formula gives no damping in that direction. A free but damped axis
  (e.g. soft wrist) needs a minimum damping or a nominal stiffness for that axis.
- **`diagonal` needs a clamp:** near singularities its $J^T D J$ grows up to ~50× the regular value, so it needs
  a per axis upper limit on $D$ (or a much larger $\lambda$). `full` does not.
- **`joint_damping` acts in parallel** in impedance mode. With the current placeholder values it dominates:
  everything is overdamped and an elbow motion creeps back over ~2.6 s, so the adaptive damping would not show.
  Keep `joint_damping` small in impedance mode when tuning and let the Cartesian $D$ carry the damping.
- **Parameters:** `cartesian_damping` (N·s/m) stays for `constant`, the adaptive laws use a single
  `cartesian_damping_ratio`. The current gains are untested placeholders, tune after this is in.
- **Realtime:** $M$ is $n \times n$ with runtime size, preallocate it and its decomposition. $\Lambda$,
  $K$, $D$ are fixed size 6×6, use `Eigen::LLT<Matrix6d>` and `Eigen::SelfAdjointEigenSolver<Matrix6d>`
  constructed once. Extend `test_cartesian_impedance` so the allocation check covers it.
- **Execution time:** measured ~7 µs per cycle for `full` (~5 µs for `diagonal`) on a Ryzen 7 5800H, against
  the controller manager's thresholds (warn 100 µs, error 200 µs mean). Recheck on the target computer.
- **Noise:** larger $D$ in heavy directions amplifies velocity noise more. Watch for the
  *"Torque rate limited"* warning and buzzing.
- **Validation:** release tests in the simulation in several poses (`push_test` measures overshoot,
  $\zeta$ and frequency), once per `damping_method`: with `constant` the results spread, with `full` they
  should match. Then the unit test: the closed loop $\zeta$ from $D(q)$, $M(q)$ and $J(q)$ equals the
  configured value in a few poses, and $D$ is continuous along a path through a singularity.

---

## 2. Changing the stiffness online, base or tool frame

### Goal

Change the Cartesian stiffness while the controller runs, e.g. more stiffness along x for a task,
without deactivating, reconfiguring and reactivating it (which drops the arm into the start ramp and
interrupts any upstream). Optionally select whether the diagonal stiffness acts along the **base**
axes or the **TCP (tool)** axes, e.g. "soft about the tool axis, stiff in position".

### Approach

- A service on the controller node, e.g. `~/set_impedance` with a new type in `robotarm_interface`:
  6 stiffnesses, damping (or the damping ratio, see part 1), the frame (`base` / `tool`), an optional
  transition time; response `success` + `message`.
- The callback (non-realtime thread) validates like `on_configure` (finite, ≥ 0, 6 values, warning
  for stiffness without damping) and hands the new set to `update()` through a
  `realtime_tools::RealtimeThreadSafeBox`, the same pattern as the jog controller's twist box.
  `update()` picks it up with `try_get`, never blocks, never allocates.
- **Tool frame:** keep error, velocity and $J$ in the base frame and rotate the gains every cycle:
  $K_{base} = R\,K_{tool}\,R^T$ (same for $D$), with $R$ the current TCP orientation. The norm clamp of
  the spring works unchanged.

### Important

- **No steps:** changing $K$ while there is a position error steps the spring force ($\Delta K \cdot e$).
  Blend old to new gains over the transition time inside `update()`; the torque rate limit is only a
  backup. Switching the frame also changes the force direction with an existing error, blend it too.
- **Damping has to follow the stiffness**, otherwise $\zeta$ changes with every new $K$. With part 1
  this is automatic (damping ratio, `full` for any stiffness ratio), without it the service has to set $D$ together with $K$.
  Implementing part 1 first makes the service simpler.
- **One request, one consistent set:** stiffness, damping and frame are applied together in the same
  cycle, never half of a request.
- **Mode:** reject the service in `gc` mode (`success = false`), or don't create it there.
- **Alternative:** ROS parameters with an `on_set_parameters` callback instead of a custom service
  (works with `ros2 param set`). Downside: several parameters are not applied atomically, and the
  YAML no longer shows the active values. A service fits a GUI better.
- **Clients:** the GUI handover ([docs/controller_switching](../controller_switching/README.md)) needs
  the new service once it exists.
- **Tests:** stiffness change while running (no allocation in `update()`, torque continuous during
  the blend, final stiffness reached), tool frame (a pull along tool x sees $K_{tool,x}$ in any TCP
  orientation), invalid requests rejected and nothing applied.
