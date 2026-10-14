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

### Approach

1. **Armature in `RobotarmRbd`** (prerequisite): reflected rotor inertia per joint, added on the
   diagonal of $M$ (`τ_i += armature_i · q̈_i` in the RNEA). It is a large share of $M$ (axis2 ~21 %,
   see [docs/dynamics](../dynamics/README.md)) and today only exists in the MuJoCo model. Use **one
   source** for the values, so `sim_params.yaml` and the controller can't drift apart.
2. **$M(q)$** from 6 RNEA calls ($\dot q = 0$, $g = 0$, $\ddot q = e_i$ gives column $i$), plus armature.
3. **$\Lambda$ regularized**: $\Lambda = (J M^{-1} J^T + \lambda^2 I)^{-1}$, like the damped $J^+$.
   Near a singularity $\Lambda$ grows without bound in the singular direction.
4. **Damping from $\zeta$** by double diagonalization: solve the generalized eigenproblem of $K$ and
   $\Lambda$ ($K v = \mu \Lambda v$), which gives $Q$ with $\Lambda = Q^T Q$ and $K = Q^T K_0 Q$,
   $K_0$ diagonal. Then $D = 2\,Q^T \operatorname{diag}(\zeta_i \sqrt{k_{0,i}})\,Q$.
   A simpler first step: $D_i = 2\zeta_i\sqrt{K_i \Lambda_{ii}}$ per axis, ignoring the coupling.

### Important

- **Stability is not at risk** as long as $D(q)$ stays positive definite, also when it changes every
  cycle (the dampers still only dissipate).
- **Zero stiffness:** with $K_i = 0$ the formula gives $D_i = 0$. A free but damped axis (e.g. soft
  wrist) needs a minimum damping or a nominal stiffness for that axis.
- **Parameters:** `cartesian_damping` (N·s/m) becomes a damping ratio per axis
  (`cartesian_damping_ratio`). Decide whether constant $D$ stays available as a fallback.
- **Realtime:** $M$ is $n \times n$ with runtime size, preallocate it and its decomposition. $\Lambda$,
  $K$, $D$ are fixed size 6×6, use `Eigen::GeneralizedSelfAdjointEigenSolver<Matrix6d>` /
  `SelfAdjointEigenSolver<Matrix6d>` constructed once. Extend `test_cartesian_impedance` so the
  allocation check covers it.
- **Execution time:** 6 extra RNEA calls plus a 6×6 eigen decomposition per cycle. Check against the
  controller manager's execution time thresholds (warn 100 µs, error 200 µs mean).
- **Noise:** larger $D$ in heavy directions amplifies velocity noise more. Watch for the
  *"Torque rate limited"* warning and buzzing.
- **Validation:** release tests in the simulation in several poses (`push_test` measures overshoot,
  $\zeta$ and frequency): with constant $D$ the results spread, with adaptive damping they should
  match. Then the unit test: $\zeta$ from $D(q)$ and $\Lambda(q)$ equals the configured value in a few
  poses.

---

## 2. Changing the stiffness online, base or tool frame

### Goal

Change the Cartesian stiffness while the controller runs, e.g. more stiffness along x for a task,
without deactivating, reconfiguring and reactivating it (which drops the arm into the start ramp and
interrupts any upstream). Optionally select whether the diagonal stiffness acts along the **base**
axes or the **TCP (tool)** axes, e.g. "soft about the tool axis, stiff in position".

### Approach

- A service on the controller node, e.g. `~/set_impedance` with a new type in `robotarm_interface`:
  6 stiffnesses, damping (or damping ratios, see part 1), the frame (`base` / `tool`), an optional
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
  this is automatic (damping ratios), without it the service has to set $D$ together with $K$.
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
