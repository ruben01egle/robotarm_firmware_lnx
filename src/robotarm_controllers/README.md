# robotarm_controllers

`ros2_control` controllers for the robot arm, exported as pluginlib plugins
([robotarm_controllers_plugins.xml](robotarm_controllers_plugins.xml)):

| Plugin | Class | Purpose |
|---|---|---|
| `robotarm_controllers/CartesianJogController` | `cartesian_jog_controller::CartesianJogController` | Jerk limited Cartesian velocity jogging of the TCP |
| `robotarm_controllers/TeleopController` | `teleop_controller::TeleopController` | Jerk limited joint space position following for teleoperation |

Both claim `position` + `velocity` command interfaces of all joints, so only one of them can be
active at a time.

---

## Contents

- [Building and testing](#building-and-testing)
- [CartesianJogController](#cartesianjogcontroller)
  - [Usage](#usage)
  - [Overview of one control cycle](#overview-of-one-control-cycle)
  - [Background 1: twists, frames and the Jacobian](#background-1-twists-frames-and-the-jacobian)
  - [Background 2: the damped pseudoinverse](#background-2-the-damped-pseudoinverse)
  - [Background 3: orientation, integration and error](#background-3-orientation-integration-and-error)
  - [Background 4: smooth motion with Ruckig](#background-4-smooth-motion-with-ruckig)
  - [Background 5: limiting without breaking smoothness](#background-5-limiting-without-breaking-smoothness)
  - [Background 6: closing the loop, x_ref and drift correction](#background-6-closing-the-loop-x_ref-and-drift-correction)
  - [Background 7: incremental moves](#background-7-incremental-moves)
  - [Safety and error handling](#safety-and-error-handling)
  - [Realtime considerations](#realtime-considerations)
  - [Known limitations and roadmap](#known-limitations-and-roadmap)
- [TeleopController](#teleopcontroller)

---

## Building and testing

```bash
colcon build --packages-select robotarm_controllers
colcon test --packages-select robotarm_controllers && colcon test-result --verbose
```

By default the package builds as `Release`, because debug builds caused overruns in the realtime
loop. Pass `-DCMAKE_BUILD_TYPE=Debug` to override this.

`test_cartesian_jog_rt_alloc` runs `CartesianJogController::update()` without a controller manager
and counts every heap allocation inside `update()`. It does this by interposing `malloc` for the
whole test executable, so allocations in Ruckig, Eigen and realtime_tools are caught as well. A
negative control test makes sure the counter is actually active.

Log output is excluded from the count: the logging backend allocates for every printed message
(rosout publishes over DDS). In `update()` those are throttled warnings (at most one per second
and message) and the errors right before a stop, so they are accepted. The test wraps the log output
handler and pauses the counter inside it; the controller's own code up to the log call is still
counted. The increment tests also check distances, accumulation, the handover to jogging and the
abort in front of a joint limit.

---

## CartesianJogController

The controller moves the tool center point (TCP) with a commanded Cartesian velocity (a *twist*),
or by small relative steps (*increments*, e.g. "1 mm along x"). It does four things:

- smooths the twist with jerk-limited acceleration (Ruckig),
- keeps joint velocities and joint position limits within bounds without bending the path,
- corrects the numerical drift of velocity-level inverse kinematics,
- stops safely when the arm does not follow.

### Usage

#### Interfaces

| Type | Name | Notes |
|---|---|---|
| Command interfaces | `<joint>/position`, `<joint>/velocity` | for every joint of the kinematic chain, interleaved per joint |
| State interfaces | `<joint>/position`, `<joint>/velocity` | same layout |
| Subscription | `~/twist_cmds` (`geometry_msgs/msg/TwistStamped`) | i.e. `/cartesian_jog_controller/twist_cmds`, `header.frame_id` selects the frame |
| Subscription | `~/increment_cmds` (`robotarm_interface/msg/CartesianIncrement`) | relative steps, `header.frame_id` selects the frame like for twists, see [incremental moves](#incremental-moves) |

The joint names, joint limits and the TCP are **not** parameters. They come from the URDF
(`robot_description`) through `robotarm_rbd`. That way the controller cannot disagree with the
kinematic model.

**Twist convention:** `linear` is the velocity of the **TCP origin** in m/s, `angular` the angular
velocity in rad/s. The frame the components are expressed in is selected by `header.frame_id`:

| `header.frame_id` | Frame |
|---|---|
| empty, or the root link of the chain (`Base_1`) | **base frame** |
| the TCP link (`tcp`) | **tool frame**: axes of the TCP, rotate about the TCP |
| anything else | message dropped (throttled warning) |

Both names come from the URDF through `robotarm_rbd`, like the joints. In both frames the reference
point is the TCP, so a pure rotation turns the tool in place. Each component is clamped to
`max_linear_velocity` / `max_angular_velocity` **in the commanded frame** (a per-axis box, see
[limitations](#known-limitations-and-roadmap)), so every axis of the operator interface has the
same limit, whatever the orientation of the tool. The clamp happens in the subscriber callback, so
the realtime loop only ever sees valid twists. Non-finite messages are dropped.

The `header.stamp` is not used (see the watchdog below), so sender and controller clocks don't need
to be synchronized.

**Command timeout (watchdog):** a non-zero twist that is older than 250 ms (no new message) is
set to zero, so Ruckig brakes smoothly (the frame is kept, so a timeout never causes a frame
switch). A publisher therefore has to keep sending at > 4 Hz while jogging, also while a value is
held constant. The age is measured with the controller manager's `time` from the cycle in which the
message is first seen (a receive counter in the realtime box detects new messages), so there is
no clock mixing with the subscriber thread. On activation, the box and the local command are
reset, so a twist received while inactive is never replayed.

**Recommended sender protocol:** publish the twist only while a jog button is held, plus **one zero
twist** on release and for a stop button. Nothing is sent while idle. Every new twist message
(zero included) also ends a running increment, see below. An emergency stop does not go through
this topic: deactivate the controller (and use the hardware e-stop).

#### Incremental moves

`~/increment_cmds` moves the TCP by a relative step instead of a velocity. One message is one step,
e.g. `linear.x = 0.001` is 1 mm along x:

```
# robotarm_interface/msg/CartesianIncrement
std_msgs/Header header          # frame_id: '' or base link → base, tcp → tool (as for twists)
geometry_msgs/Vector3 linear    # m, per axis
geometry_msgs/Vector3 angular   # rad, per axis (not a rotation vector)
```

- Each component is **clamped** to `max_increment_linear` / `max_increment_angular`. The sender
  chooses the step size (e.g. 1, 2 or 3 mm buttons), the controller only bounds it.
- Steps **add up**: three messages of +1 mm give 3 mm, also when they arrive while the arm is
  still moving. Several messages within one control cycle are not lost.
- A group of steps that belong together is called a **run**. A run starts with the first step
  that is accepted and ends when the target is reached. While it runs, the controller is in
  *increment mode*; otherwise it is in *jog mode*.

| Situation | What happens |
|---|---|
| Step while jogging is idle (twist zero, Ruckig at rest) | a run starts in the frame of the step |
| Step during a run, same frame | added to the run's target |
| Step during a run, other frame | dropped (throttled warning) |
| Step while jogging or still braking | dropped (throttled warning) |
| Steps in base and tool frame within the same cycle while idle | dropped (throttled warning) |
| Target reached | back to jog mode, the arm is at rest |
| Any new twist message during a run | run ends, jog mode continues smoothly from the current motion |
| A joint gets stuck in front of its position limit | run aborted, the arm brakes (throttled warning) |

There is no feedback topic. A step the arm can't (fully) do is simply not (fully) done; the sender
sees the result in the TCP pose. What is exact and what isn't is explained in
[background 7](#background-7-incremental-moves).

#### Parameters

All parameters are read in `on_configure`. Values that violate a listed constraint make
configuration fail with an error message (`lambda` is checked by `robotarm_rbd`). The exception is
the Cartesian velocity, acceleration and jerk limits: they aren't validated yet. Non-positive
acceleration or jerk limits only show up as a Ruckig error on the first `update()`.

| Parameter | Default | Unit | Constraint | Meaning |
|---|---|---|---|---|
| `lambda` | `0.01` | – | > 0 | Damping $\lambda$ of the damped least squares Jacobian inverse ([background 2](#background-2-the-damped-pseudoinverse)) |
| `jinv_method` | `"svd"` | – | `"svd"` or `"ldlt"` | Jacobian inverse method: `svd` selective, adaptive damping (accurate), `ldlt` constant damping $\lambda$ (fast) |
| `max_linear_velocity` | `0.1` | m/s | | Per-axis clamp of the commanded linear velocity |
| `max_linear_acceleration` | `0.5` | m/s² | > 0 | Ruckig limit, linear DOFs |
| `max_linear_jerk` | `5.0` | m/s³ | > 0 | Ruckig limit, linear DOFs |
| `max_angular_velocity` | `0.5` | rad/s | | Per-axis clamp of the commanded angular velocity |
| `max_angular_acceleration` | `2.0` | rad/s² | > 0 | Ruckig limit, angular DOFs |
| `max_angular_jerk` | `20.0` | rad/s³ | > 0 | Ruckig limit, angular DOFs |
| `kp_linear` | `10.0` | 1/s | ≥ 0, `kp·dt < 1` | Gain of the position drift correction |
| `kp_angular` | `10.0` | 1/s | ≥ 0, `kp·dt < 1` | Gain of the orientation drift correction |
| `max_correction_linear` | `0.01` | m/s | ≥ 0 | Norm clamp of the linear correction twist |
| `max_correction_angular` | `0.05` | rad/s | ≥ 0 | Norm clamp of the angular correction twist |
| `max_tracking_error` | `0.1` | rad | > 0 | Max. $\lvert q_{cmd} - q_{meas}\rvert$ per joint before the controller stops |
| `joint_velocity_scale` | `0.2` | – | (0, 1] | Fraction of the URDF joint velocity limits used while jogging |
| `joint_limit_zone` | `0.2` | rad | > 0 | Width of the slow-down zone in front of a joint position limit |
| `joint_limit_margin` | `0.02` | rad | ≥ 0 | The joint stops this far before its position limit |
| `max_increment_linear` | `0.003` | m | > 0 | Per-axis clamp of one linear increment step |
| `max_increment_angular` | `0.0524` | rad | > 0 | Per-axis clamp of one angular increment step (3°) |

`dt` in the `kp` check is `1 / update_rate` of the controller manager. With the 500 Hz from
`robotarm_bringup`, `kp < 500 1/s`. Values of 10–20 1/s are plenty, see
[background 6](#background-6-closing-the-loop-x_ref-and-drift-correction).

Tuning hints:
- **`max_tracking_error`**: even when everything works, the measured position lags the command by
  one cycle plus the drive's following error. Jog at full speed, log the largest deviation, and set
  the threshold to about 3–5× that value.
- **`joint_limit_zone`**: has to be larger than the braking distance in joint space at jogging
  speed, because Ruckig decelerates with the Cartesian acceleration limit and not instantly. If the
  warning *"clamped to its hard position limit"* ever appears, increase the zone.
- **`joint_velocity_scale`**: URDF velocity limits are hardware limits, far too fast for jogging.
  Lower values make the arm slow down earlier in poses that need fast joints (near singularities).

Example (`robotarm_bringup/config/controllers.yaml`):

```yaml
cartesian_jog_controller:
  ros__parameters:
    lambda: 0.01
    jinv_method: "svd"
    max_linear_velocity: 0.1
    max_linear_acceleration: 0.5
    max_linear_jerk: 5.0
    max_angular_velocity: 0.5
    max_angular_acceleration: 2.0
    max_angular_jerk: 20.0
    kp_linear: 10.0
    kp_angular: 10.0
    max_correction_linear: 0.01
    max_correction_angular: 0.05
    max_tracking_error: 0.1
    joint_velocity_scale: 0.2
    joint_limit_zone: 0.2
    joint_limit_margin: 0.02
    max_increment_linear: 0.003
    max_increment_angular: 0.0524
```

#### Running it

`robotarm_bringup/launch/robotarm.launch.py` spawns the controller **inactive**. To activate it (and
release the teleop controller, which claims the same interfaces):

```bash
ros2 control switch_controllers --activate cartesian_jog_controller --deactivate teleop_controller
```

Send a twist, e.g. 2 cm/s along base x:

```bash
ros2 topic pub -r 50 /cartesian_jog_controller/twist_cmds geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: ''}, twist: {linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

or 2 cm/s along the tool z axis (e.g. towards the workpiece):

```bash
ros2 topic pub -r 50 /cartesian_jog_controller/twist_cmds geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'tcp'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.02}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

The example publishes at 50 Hz (`-r 50`). A single `--once` message only moves the arm for 250 ms
before the watchdog stops it. Stopping the publisher stops the arm.

An increment is a single message, e.g. 1 mm along base x:

```bash
ros2 topic pub --once /cartesian_jog_controller/increment_cmds robotarm_interface/msg/CartesianIncrement \
  "{header: {frame_id: ''}, linear: {x: 0.001, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### Lifecycle behavior

| Transition | What happens |
|---|---|
| `on_init` | Declares the parameters. |
| `on_configure` | Reads and validates parameters, initializes the kinematic model from the URDF, scales the joint velocity limits, sizes all buffers, creates both subscriptions. |
| `on_activate` | Reads the joint state (strict: all joints or fail), seeds `q_cmd` with it, commands velocity 0, computes `x_ref = FK(q_cmd)`, resets Ruckig to rest, the twist to zero, the pending increments and the mode to jog. |
| `update` | One control cycle, see below. |
| `on_deactivate` | Nothing. The hardware holds position when the interfaces are released. |
| `on_cleanup` | Destroys both subscriptions. Their callbacks read parameters and frame names, which a following `on_configure` rewrites. |

If `update()` returns `ERROR`, the controller manager **deactivates the controller in the same
cycle** (and the hardware holds position). The controller has to be activated again explicitly,
which re-seeds everything.

### Overview of one control cycle

```
 jog:        twist_cmds ─► [pre-scaling: scale the target twist] ───────► Ruckig (velocity mode)
 increment:  increment_cmds ─► Σ steps ─► [pre-scaling: scale max_velocity] ─► Ruckig (position mode)
                                                                                │ dx_ff (smooth)
                    x_ref ⊖ FK(q_cmd) ─► Kp, clamp ─► dx_corr                   │
                                                          └──────► (+) ◄────────┘
                                                                    │ dx_cmd
                                                    J⁺(q_cmd) ──►  dq_cmd ─► [post-scaling s] ─►
                                       q_cmd += dq_cmd·dt ─► [hard position clamp] ─► hardware
                                       x_ref ⊕= s·J·J⁺·dx_ff·dt
```

The controller has two modes, *jog* and *increment*. They share the whole cycle; where they
differ, the step has one path per mode, so the flow of information is always the same.

In code order (`update()` in [src/CartesianJogController.cpp](src/CartesianJogController.cpp)):

1. **Commands.** Take the latest twist (already clamped in the callback) and the pending increment
   steps from their realtime boxes.
   - *jog:* adopt a new twist (stamped with `time`), otherwise run the **watchdog** (zero a non-zero
     twist older than 250 ms). Steps while idle start an increment run.
   - *increment:* a new twist ends the run and is adopted.
2. Read the joint state (best effort) and run the **tracking check** $\lvert q_{cmd} - q_{meas}\rvert$.
3. Compute $J^+$ and $J$ at $q_{cmd}$.
4. **Frame** $F$ of this cycle: the twist frame (jog) or the run's frame (increment).
5. **Pre-scaling**, then write Ruckig's input.
   - *jog:* rotate the target into base, predict $\dot q_{pred} = J^+ R_{base,F}\,\dot x_{target}$,
     compute the joint velocity factor and the one-sided position limit factor, and scale the
     target twist (still in $F$) by their minimum. If $F$ changed, Ruckig's state is rotated into
     the new frame.
   - *increment:* add the new steps to the target position, predict the worst case joint velocity
     over all *corners* ([background 7](#background-7-incremental-moves)) and scale Ruckig's
     `max_velocity` by the factor. Abort the run if the factor drops below 0.01.
6. **Ruckig** runs in $F$. The output is rotated into base: the feedforward twist $\dot x_{ff}$.
   *increment:* `Finished` ends the run. Everything from here on is in base and the same for both
   modes.
7. **Drift correction:** pose error between `x_ref` and $FK(q_{cmd})$, times gain, norm clamped.
8. $\dot q_{cmd} = J^+(\dot x_{ff} + \dot x_{corr})$, then the **post-scaling** factor $s$ as a backstop.
9. Integrate $q_{cmd}$, advance `x_ref` by the realized feedforward $s\,J J^+ \dot x_{ff}$.
10. Hard clamp $q_{cmd}$ to the joint position limits (backstop).
11. Advance Ruckig's state (`pass_to_input`, or re-sync if $s<1$, including the position step).
12. Write $q_{cmd}$ and $\dot q_{cmd}$ to the command interfaces.

The following sections explain each of these steps.

### Background 1: twists, frames and the Jacobian

A **twist** $\dot x = [v^T\ \omega^T]^T \in \mathbb{R}^6$ only has meaning once you fix two things:

1. the **frame** its components are expressed in (base or tool), and
2. the **reference point** whose linear velocity $v$ is (the angular velocity $\omega$ is the
   same for every point of a rigid body).

The geometric Jacobian of `robotarm_rbd` (`RobotarmRbd::calculate_jacobian`) is built column by
column for revolute joints:

$$
J_i = \begin{bmatrix} z_i \times (p_{tcp} - p_i) \\ z_i \end{bmatrix}
$$

with the joint axis $z_i$ and joint origin $p_i$ in base coordinates. So
$\dot x = J(q)\,\dot q$ is **the velocity of the TCP origin and the angular velocity, both in base
coordinates** (Pinocchio calls this `LOCAL_WORLD_ALIGNED`). This is the natural choice for jogging:
"rotate about z" rotates the tool **about the TCP**, not about the base origin.

The commanded twist has to use the same convention:

- **Tool-frame jogging**: rotate the twist into base axes,
  $v_{base} = R\,v_{tool}$, $\omega_{base} = R\,\omega_{tool}$, with $R$ the TCP orientation. Only
  the axes change, the reference point stays the TCP. Where exactly this rotation happens matters
  for smoothness, see [Ruckig in the commanded frame](#ruckig-in-the-commanded-frame).
- **Another reference point** $p$: the linear part changes, the angular part doesn't:
  $v_{tcp} = v_p + \omega \times (p_{tcp} - p)$.

If the reference point is wrong, rotating "in place" makes the TCP orbit around some other point.

### Background 2: the damped pseudoinverse

To get joint velocities from a twist, $J$ has to be inverted. $J$ is $6\times n$ and becomes
ill-conditioned near singularities, so the controller uses **damped least squares** (DLS):

$$
\dot q = \arg\min_{\dot q}\ \lVert J\dot q - \dot x\rVert^2 + \lambda^2\lVert \dot q\rVert^2
\quad\Rightarrow\quad
J^+ = (J^TJ + \lambda^2 I)^{-1}J^T
$$

(computed with an LDLT factorization in `calculate_jacobian_inverse`).

The singular value decomposition $J = U\Sigma V^T$ shows what the damping does:

$$
J^+ = V\,\mathrm{diag}\!\left(\frac{\sigma_i}{\sigma_i^2+\lambda^2}\right)U^T,
\qquad
J J^+ = U\,\mathrm{diag}\!\left(\frac{\sigma_i^2}{\sigma_i^2+\lambda^2}\right)U^T
$$

- For $\sigma_i \gg \lambda$ the factor is $\approx 1/\sigma_i$: it behaves like the true inverse.
- For $\sigma_i \to 0$ (singularity) the factor goes to $0$ instead of $\infty$, so joint velocities
  stay bounded. In return, motion along that direction is suppressed.
- $J J^+ \neq I$: **the twist actually realized is $J J^+\dot x$**, slightly attenuated in every
  direction by $\sigma_i^2/(\sigma_i^2+\lambda^2)$. Far from singularities ($\sigma \sim 0.3$–$1$,
  $\lambda = 0.01$) that's $10^{-4}$–$10^{-3}$, negligible. As $\sigma \to \lambda$ it becomes
  substantial.

This "realized twist" $J J^+ \dot x$ becomes central in
[background 6](#background-6-closing-the-loop-x_ref-and-drift-correction).

Constant $\lambda$ is a compromise: it costs a little accuracy everywhere and still damps only
moderately near singularities. **Adaptive damping** (on the roadmap) removes that compromise,
e.g. with

$$
\lambda^2 = \begin{cases} \lambda_0^2\left(1 - (\sigma_{min}/\varepsilon)^2\right) & \sigma_{min} < \varepsilon \\ 0 & \text{otherwise} \end{cases}
$$

That gives the exact pseudoinverse away from singularities and damping only close to them. It needs
$\sigma_{min}$, so one SVD per cycle, which takes microseconds at $6\times6$.

### Background 3: orientation, integration and error

Position behaves like a vector: integrate by adding, compute the error by subtracting. Orientation
doesn't, and this is where velocity-level Cartesian control usually goes wrong.

**$\omega$ isn't the derivative of any three angles.** Integrating $\omega$ componentwise into
roll-pitch-yaw (or any Euler angles) is wrong as soon as you rotate about more than one axis.
Rotations have to be composed:

$$
R_{k+1} = \exp\!\big([\omega\,\Delta t]_\times\big)\,R_k
$$

$\exp([\phi]_\times)$ is the rotation by the angle $\lVert\phi\rVert$ about the axis
$\phi/\lVert\phi\rVert$ (axis-angle, Rodrigues). In code:

```cpp
Eigen::Vector3d rot = omega * dt;                                 // rotation vector
R = Eigen::AngleAxisd(rot.norm(), rot.normalized()) * R;          // left-multiply
```

- **Left vs right multiplication:** $R$ maps tool coordinates to base coordinates.
  $R_\Delta R$ rotates about an axis given in **base** coordinates; $R R_\Delta$ about an axis
  given in **tool** coordinates. $\omega$ comes from the Jacobian in base coordinates, so the
  increment goes on the left. (For example, with the tool pointing down, "spin about base z"
  and "spin about tool z" turn in opposite directions.)
- **Zero rotation:** `normalized()` of a zero vector returns the zero vector (Eigen ≥ 3.3), and
  `AngleAxisd(0, ·)` is the identity, so no special case is needed.
- **Don't use `Isometry3d::prerotate`.** It left-multiplies the *whole* transform, so it
  would also rotate the translation about the base origin.

**Orientation error.** The rotation that takes the current orientation to the reference is

$$
R_{err} = R_{ref}\,R_{cur}^T \quad\text{(expressed in base, matching } \omega\text{)},
$$

and its **log map** gives a 3-vector, axis times angle:

$$
e_{rot} = \theta\,u, \qquad (\theta, u) = \text{axis-angle of } R_{err}
$$

`Eigen::AngleAxisd(R_err)` computes it exactly for $\theta \in [0,\pi]$. $e_{rot}$ points along the
axis to rotate about and its length is how far off you are, so $K_p e_{rot}$ is directly a valid
angular velocity. Subtracting RPY angles would break at wraparound (179° → −179°), at gimbal lock,
and because RPY differences aren't vectors in the frame of $\omega$.

The two operations mirror each other:
- **error:** rotation matrix → `AngleAxisd` → vector $\theta u$ (log)
- **update:** vector $\omega\Delta t$ → `AngleAxisd` → rotation matrix (exp)

Translation and rotation are integrated **separately** ("translate the TCP linearly, rotate about
the TCP"). The full $SE(3)$ exponential of a twist would produce a screw motion (a helix), which
isn't what an operator expects from a jog.

### Background 4: smooth motion with Ruckig

[Ruckig](https://github.com/pantor/ruckig) is an **online trajectory generator**. Given the current
kinematic state, a target state and limits, it computes the time-optimal jerk-limited trajectory
each cycle and returns the next sample. The target can change at any time, and it just replans from
where it is, in microseconds.

The controller runs one **6-DOF Ruckig in velocity mode on the twist**:

```cpp
ruckig_input_.control_interface = ruckig::ControlInterface::Velocity;
ruckig_input_.synchronization   = ruckig::Synchronization::None;
// DOF 0..2: linear limits, DOF 3..5: angular limits
```

Each cycle:

```cpp
input.target_velocity = scaled operator twist;   // write unconditionally
ruckig_.update(input, output);                   // replans only if the input changed
dx_ff = output.new_velocity;                     // smooth twist
output.pass_to_input(input);                     // its own output becomes the next current state
```

Points that matter:

- **Feed back Ruckig's own output, not measurements.** Measured state is noisy. Feeding it back
  makes Ruckig replan every cycle and jitter. Ruckig's state is the *intent*. Measured state is used
  only for seeding and safety checks.
- **`pass_to_input` copies only the current state.** Targets, limits and modes stay untouched, so
  you don't need any change detection: Ruckig compares the input with the last call by itself.
- **Velocity mode ignores the position fields** and **ignores `max_velocity`**
  (`ControlInterface::Velocity: "Ignores the current position, target position, and velocity
  limits"`). The velocity limit is enforced by clamping the twist *before* Ruckig. Only
  acceleration and jerk limits are active. `max_position`/`min_position` only work in Ruckig Pro.
- **Angular "positions" mean nothing.** Ruckig would integrate $\omega$ componentwise, which is the
  wrong integration from background 3. Only `new_velocity` is used, and the pose lives in `x_ref`.
- **Synchronization `None`:** each axis is its own slider and ramps at its own maximum rate. `Time`
  (all DOFs finish together) would slow fast axes down and couple them: nudging x would replan y. A
  direction-preserving straight line in velocity space would be `Phase`, not `Time`. That's the
  right choice for direction-type inputs (spacemouse), not for per-axis sliders. Increments use
  `None` as well, see [background 7](#synchronization-none-not-phase).
- Limits are **per DOF**: a diagonal xyz jog can reach $\sqrt{3}\times$ the per-axis speed.

#### Ruckig in the commanded frame

Ruckig runs in the frame of the current command ($F$ = base or tool), not always in base. Its
output is rotated into base afterwards with $R_{base,F}$ (identity for base, `x_ref.linear()` for
tool, both at the start of the cycle):

```cpp
dx_ff = R_base_F * output.new_velocity;          // linear and angular half separately
```

Why not rotate the tool twist into base *before* Ruckig? A constant tool-frame twist with
translation **and** rotation is a screw motion: in base its linear part keeps turning ($\dot v_{base}
= \omega \times v_{base}$). Ruckig only sees the current target and chases it, so the velocity
trails slightly behind the target (about $a^2/2j$, roughly 0.15° at the default limits). In the
commanded frame the target is constant, Ruckig sits exactly on it, and the rotation into base is
applied exactly. It also keeps the per-axis limits and the synchronization on the axes the operator
actually moves.

The scaling factors are computed in base (the Jacobian is in base) but are scalars, and a scalar
times a twist is the same in every frame. So only the *prediction* uses the rotated twist; the
target handed to Ruckig stays in $F$. The `s < 1` re-sync and `pass_to_input` work in $F$ unchanged.

**Frame switch.** When a command arrives in a different frame than Ruckig's state, the state is
rotated into the new frame before the update, with $R_{new,old} = R$ (tool → base) or $R^T$
(base → tool):

```cpp
current_velocity     = R_new_old * current_velocity;      // linear and angular half separately
current_acceleration = R_new_old * current_acceleration;
```

The velocity is exact (a snapshot, just new coordinates). The acceleration isn't quite: Ruckig's
acceleration is the derivative of its velocity *coordinates*, and the tool frame turns. With
$v_{base} = R\,v_{tool}$ and $\dot R = R\,[\omega_{tool}]_\times$:

$$
\dot v_{tool} = R^T a_{base} - \omega_{tool}\times v_{tool}
$$

The $\omega \times v$ term is dropped. This is a jump of $\lvert\omega\rvert\lvert v\rvert$ in the
starting acceleration of the next plan (0.05 m/s² at the default limits, a tenth of the
acceleration limit), only when switching while translating *and* rotating. The angular part is
exact ($\omega \times \omega = 0$). With jog sliders that snap back to zero, switches usually happen
at rest, where nothing is rotated at all.

#### What smooth Cartesian motion does *not* guarantee in joint space

Ruckig bounds $\ddot x$ and $\dddot x$. The joints see

$$
\dot x = J\dot q
\;\Rightarrow\;
\ddot x = J\ddot q + \dot J\dot q
\;\Rightarrow\;
\ddot q = J^+(\ddot x - \dot J\dot q)
$$

So joint acceleration isn't just $J^+\ddot x$. Even constant Cartesian velocity needs nonzero joint
acceleration, because $J$ changes along the path ($\dot J \dot q \neq 0$). Jerk adds further terms
in $\ddot J$. **Cartesian jerk limits do not imply joint jerk limits**, especially where $J^+$ is
large (near singularities).

Why the controller accepts this:

- Away from singularities $J$ changes slowly. A smooth twist gives smooth $\dot q$, and joint
  accelerations stay roughly at $J^+\ddot x$, bounded and moderate. With Cartesian limits well
  below the arm's capabilities, this is enough for hand jogging.
- **You can't clip joint acceleration like velocity.** $\ddot q$, $\dot q$ and $q$ are linked by
  integration. Limiting $\ddot q$ consistently *is* trajectory generation, i.e. a second Ruckig in
  joint space. When that one hits its limits, it **bends the Cartesian path**, and the drift
  correction only pulls it back afterwards. So it wasn't adopted. The rest of the problem is
  handled at the source: slow down where $J^+$ amplifies (velocity pre-scaling, singularity scaling
  on the roadmap).
- Joint acceleration can be monitored by finite-differencing the *commanded* velocity,
  $(\dot q_{cmd,k} - \dot q_{cmd,k-1})/\Delta t$. That's noise-free and includes every scaling and
  correction.

### Background 5: limiting without breaking smoothness

The key observation:

> **Anything changed on the target twist *before* Ruckig gets ramped smoothly. Anything changed
> *after* Ruckig causes steps.**

So all *predictable, gradual* limits act on Ruckig's target, and the limiter after Ruckig is only a
backstop.

#### Pre-scaling (before Ruckig)

One extra matrix-vector product per cycle predicts the joint velocities the operator's twist would
need right now:

$$
\dot q_{pred} = J^+(q_{cmd})\,\dot x_{target}
$$

**Joint velocity factor:**

$$
s_{vel} = \min\!\Big(1,\ \min_i \frac{\dot q_{max,i}}{\lvert\dot q_{pred,i}\rvert}\Big)
$$

**One-sided position limit factor.** The sign of $\dot q_{pred,i}$ tells which limit joint $i$ is
heading towards. Let $d_i$ be the distance to *that* limit minus `joint_limit_margin`. The allowed
speed ramps down linearly inside the zone:

$$
\dot q_{allowed,i} = \dot q_{max,i}\cdot \mathrm{clamp}\!\left(\frac{d_i}{\text{zone}},\,0,\,1\right),
\qquad
s_{pos} = \min_i \frac{\dot q_{allowed,i}}{\lvert\dot q_{pred,i}\rvert}
$$

Moving *away* from a limit measures the distance to the opposite limit, so it's unrestricted. Joints
without position limits (`max <= min` in the model, e.g. continuous) are skipped.

The target is then scaled as a whole:

$$
\dot x_{target} \leftarrow \min(s_{vel}, s_{pos})\cdot\dot x_{target}
$$

- **Direction is preserved**: all six components are scaled by the same factor, so the TCP
  stays on its straight line and only gets slower. One joint near its limit slows the whole motion,
  which is intended.
- Ruckig sees its target velocity decrease gradually and **decelerates with its normal jerk
  limits**.
- $J$ changes very little from one cycle to the next, so the prediction is accurate.

Example: jogging x at 0.1 m/s towards a stretched pose. If joint 2 would need 2.0 rad/s against a
1.5 rad/s limit, then $s = 0.75$ and the target becomes 0.075 m/s. The arm slows down smoothly while
J worsens, instead of being clipped.

This approach was chosen over deriving Cartesian limits from the kinematic chain analytically. That
derivation is pose-dependent and gets out of hand quickly, while the per-cycle prediction only asks
*"from here, would this twist be too fast?"*.

#### Post-scaling (after Ruckig, backstop)

$$
\dot q_{cmd} = J^+(\dot x_{ff} + \dot x_{corr}),\qquad
s = \min\!\Big(1,\ \min_i \frac{\dot q_{max,i}}{\lvert\dot q_{cmd,i}\rvert}\Big),\qquad
\dot q_{cmd} \leftarrow s\,\dot q_{cmd}
$$

In normal operation $s = 1$. It only triggers when Ruckig's output differs from the scaled target:

1. **Ruckig lags a falling target.** When approaching a singularity quickly, $s_{vel}$ drops faster
   than the Cartesian acceleration limit allows Ruckig to decelerate.
2. **The correction twist** is added after Ruckig and wasn't part of the prediction.
3. **Ramp between two twists** with `Synchronization::None`: the intermediate twist lies in the
   box spanned by start and target, not on the line between them, and its $J^+$-image can exceed a
   limit that neither end exceeds.

Why clamp in software when the drives have limiters anyway: **consistency of $q_{cmd}$**. If the
drive silently limited velocity while the controller integrated the unlimited value, $q_{cmd}$
would run ahead of the real arm. FK, the correction and the Jacobian would then refer to a pose the
arm isn't in, and the drive would jump when the limit releases. Clamping in software keeps
$q_{cmd}$ executable.

When $s < 1$, Ruckig's state is **re-synced** to what was actually executed:

```cpp
current_velocity     = s * new_velocity;
current_acceleration = s * new_acceleration;
```

Otherwise Ruckig would think it's at full speed while the arm crawls, and the speed would jump back
without a ramp when the limit releases. The acceleration is an approximation: with
$v_{real} = s(t)\,v_{ff}(t)$ the true value is $s\,a_{ff} + \dot s\,v_{ff}$, and the $\dot s$ term
is dropped. That's acceptable, because it's only the starting state of the next plan. It's
plausible, has the right direction and stays within limits ($s\le 1$). The exact value would need a
finite difference across a step, which explodes. Jerk isn't part of Ruckig's state: every new plan
starts with zero jerk.

#### Hard position clamp (backstop)

After integration, $q_{cmd}$ is clamped to `[min, max]` and the clamped joint's velocity command is
set to 0. With a properly sized `joint_limit_zone`, this never triggers (it logs a warning when it
does). A known limitation: the clamp isn't reflected in `x_ref`, so the drift correction pushes
against it while it's active, bounded by `max_correction_*`.

### Background 6: closing the loop, x_ref and drift correction

#### Why open-loop integration drifts

Plain velocity-level IK, $q \mathrel{+}= J^+\dot x\,\Delta t$, is open loop. Each step is slightly
wrong:

- **Linearization**: $J$ is treated as constant over $\Delta t$; the true motion is $FK(q+\dot q\Delta t) - FK(q)$.
- **DLS damping**: $J J^+ \neq I$ (background 2).
- **Scaling and clamping**: the joints don't do the full twist.

These errors add up. Jogging pure x for 30 cm ends a few mm off in y and slightly tilted, and
nothing notices.

#### Which state lives where

| Quantity | Space | Source | Role |
|---|---|---|---|
| $q_{cmd}$ | joint | **integrated** $q_{cmd} \mathrel{+}= \dot q_{cmd}\Delta t$ | master state: commanded, used for $J$ and FK |
| current TCP pose | Cartesian | **computed** each cycle, $FK(q_{cmd})$ | actual side of the error |
| `x_ref` | Cartesian | **integrated** from the feedforward | reference side of the error |
| $q_{meas}$, $\dot q_{meas}$ | joint | **measured** | seeding on activate, tracking check |

- The current Cartesian pose is never integrated separately. Two integrated states describing the
  same thing would drift apart, which is exactly the error to be measured. FK is exact and cheap.
- The loop is closed on **$q_{cmd}$, not on measurements**. It only corrects kinematic integration
  error. Closing on encoder values would fight the drives' own position loops and inject noise.
- The error has to be measured in **Cartesian space with FK**. The error is *created* in the
  Cartesian → joint conversion, and only the forward direction is exact and unique. A joint space
  reference would need IK, which the whole Jacobian approach exists to avoid.

#### The correction

With both sides at the same time step $k$ (the error is computed **before** `x_ref` advances,
otherwise the feedforward would be commanded twice):

$$
e = \begin{bmatrix} p_{ref} - p(q_{cmd}) \\ \log\!\big(R_{ref}R(q_{cmd})^T\big) \end{bmatrix},
\qquad
\dot x_{corr} = \begin{bmatrix} \mathrm{clamp}_{\lVert\cdot\rVert}(K_{p,lin}\,e_{lin},\ c_{lin}) \\ \mathrm{clamp}_{\lVert\cdot\rVert}(K_{p,ang}\,e_{rot},\ c_{ang}) \end{bmatrix}
$$

$$
\dot x_{cmd} = \dot x_{ff} + \dot x_{corr}
$$

- **Discrete stability:** the error shrinks by about a factor of $(1 - K_p\Delta t)$ per cycle.
  For $K_p\Delta t \ge 1$ it overshoots every cycle and oscillates, so configuration rejects it. The
  correction only removes small linearization errors, so 10–20 1/s is plenty.
- **The norm clamp** (not per component, which would change the direction) keeps the correction
  a gentle nudge (≈10 % of jog speed). It never drives the arm on its own.
- The correction goes in **before the post-scaling**, so the backstop sees the full joint velocity.
- **The correction is never added to `x_ref`**: the reference would chase its own correction, and
  the error would never shrink.
- When the twist is zero, `x_ref` stands still and the correction holds the pose exactly, with no
  creep.

#### How x_ref advances, and why it changed

**First version:** $x_{ref} \mathrel{\oplus}= s\,\dot x_{ff}\,\Delta t$, the reference follows the
commanded feedforward, reduced only by the post-scaling factor.

This failed in tests. Near singularities, or when commanding past the edge of the workspace, the
arm physically can't follow $\dot x_{ff}$, but `x_ref` kept moving. The error grew without bound
(**windup**), and the correction, clamped but persistent, pulled the arm in strange directions as
it tried to chase an unreachable reference. Anti-windup patches only treated the symptom.

**Current version:** advance `x_ref` by the twist the joint command **actually realizes**:

$$
\dot q_{ff} = s\,J^+\dot x_{ff}, \qquad
x_{ref} \mathrel{\oplus}= J\,\dot q_{ff}\,\Delta t = s\,J J^+\dot x_{ff}\,\Delta t
$$

($\oplus$: translation added, rotation composed from the left via exp, background 3)

With the SVD from background 2, $J J^+ = U\,\mathrm{diag}(\sigma_i^2/(\sigma_i^2+\lambda^2))\,U^T$ is
a **soft projection onto the directions the arm can currently move in**. That splits the error
sources cleanly:

| Error source | First version | Current version |
|---|---|---|
| Unreachable directions (singularity, workspace edge) | accumulates in `x_ref` → windup | never enters `x_ref` |
| DLS damping error ($J J^+ \neq I$) | corrected | not tracked anymore |
| Linearization drift ($J$ changes within $\Delta t$) | corrected | **still corrected** |
| Scaling ($s<1$) | represented via $s$ | represented via $s$ |

What this buys and what it costs:

- **No windup, by construction.** `x_ref` only moves where the joints move, so there's no state
  that can run away and nothing to anti-windup. Pushing outward at full reach removes the radial
  component; `x_ref` stays put, nothing pulls, and moving back works immediately.
- **The drift correction keeps its real job.** $J\dot q\Delta t$ vs. the true FK step is second
  order, small, but cumulative over long jogs (especially orientation). That's corrected.
- **Cost: the DLS error is no longer corrected.** The TCP follows $J J^+\dot x_{ff}$ instead of
  $\dot x_{ff}$: a little slower and minimally off direction, scaled by
  $\sigma_i^2/(\sigma_i^2+\lambda^2)$. This isn't drift: the pose doesn't move away from `x_ref`.
  `x_ref` itself just tracks the command slightly less exactly. Far from singularities that's
  0.01–0.1 % and unnoticeable. Adaptive damping (background 2) brings it to zero there.
- $J$ and $J^+$ are both evaluated at $q_{cmd}$ in the same cycle, so the projection is consistent
  with the joint command.
- Ruckig is **not** re-synced to the realized twist. It keeps the operator's smoothed intent;
  syncing it would make jogging at the workspace edge feel sticky.

### Background 7: incremental moves

An increment ("move 1 mm along x") is a position goal, while everything so far works with
velocities. The tempting designs bring a lot of extra machinery. The one used here needs almost
none, because it reuses the jog pipeline and only changes **what goes into Ruckig**.

#### The idea: a second input to the same Ruckig

| | Jog mode | Increment mode |
|---|---|---|
| Ruckig `control_interface` | `Velocity` | `Position` |
| Input | `target_velocity = scaled twist` | `target_position += step` per click |
| Ruckig state | own output (`pass_to_input`) | own output (`pass_to_input`) |
| Used downstream | `new_velocity` only | `new_velocity` only |
| Pre-scaling acts on | the target twist | `max_velocity` |
| Everything after Ruckig | identical | identical |

Downstream nothing knows that an increment is running: $\dot x_{ff}$ is Ruckig's `new_velocity`,
rotated into base, exactly as while jogging. Ruckig's positions never leave Ruckig. They are only
bookkeeping for the run:

- they count **from the start of the run**: `current_position = 0`, `target_position = 0` when it
  starts (velocity mode keeps integrating a position nobody uses, so it has to be reset),
- clicks add up in `target_position`,
- `target_position − current_position` is what's left of the run (used for the pre-scaling),
- `Finished` tells when the run is done.

**Why the distance comes out right.** Ruckig plans a velocity profile from rest to rest whose
integral is the step. The controller integrates exactly that velocity into `q_cmd` and `x_ref`.
Ruckig integrates exactly, the controller per cycle as $\sum v_k\,\Delta t$ with the velocity at the
end of each cycle. Because the profile starts and ends at $v = 0$, that sum equals the trapezoidal
rule, whose error is $\propto \Delta t^2\,(a_{end} - a_{start}) = 0$. 1 mm stays 1 mm to far below
a µm.

#### Why Ruckig is fed only its own state

The obvious alternative is to close a loop around Ruckig: keep a goal pose `x_goal` and recompute
Ruckig's current position from `x_ref` every cycle ("how far is the arm really from the goal?").
That's exact, but it was rejected, because every problem the arm has leaks into Ruckig:

- when the arm can't follow (singularity, workspace edge), Ruckig's position stands still and
  `Finished` never comes, so a stuck detection with timeouts or progress checks is needed,
- Ruckig's velocity keeps "cruising" in a direction nothing moves in (a phantom velocity), which
  must be cleaned up before handing over to jogging,
- Ruckig replans every cycle, so its planned duration is useless as a deadline.

Feeding Ruckig only its own output (its **intent**) makes all of that disappear, and it's the same
philosophy as jog mode ([background 6](#background-6-closing-the-loop-x_ref-and-drift-correction)):
Ruckig holds the intent, `x_ref` follows what is realized, and what the arm can't do simply isn't
done, without windup. `Finished` is then guaranteed: every plan reaches the target, lower limits
only make it later. The price is the same as while jogging: near singularities the arm falls short
of the step by what $J J^+$ swallows.

#### Getting the steps into the realtime loop

The jog box only holds the **latest** twist. That's right for a velocity, but wrong for steps:
two clicks between two cycles would lose one. So increments have their own box that **collects**:

```cpp
struct PendingIncrementCmd { Vector6d base, tool; };   // one slot per frame

// callback (non-RT): static checks, clamp, then add to the slot of its frame
// update() (RT):      try_set([&](PendingIncrementCmd& p){ taken = p; p = {}; })   // take and clear
```

- Adding and taking happen under the box's lock, so nothing is lost or counted twice. If the RT
  side doesn't get the lock, the steps simply wait for the next cycle.
- **One slot per frame**, so a click in the wrong frame can't block or mix with the right ones.
- The callback only does checks that depend on the message alone (finite, clamp, frame name).
  Whether a step is *accepted* depends on the controller state (mode, run frame, Ruckig at rest),
  and that state lives in `update()`. Reading it from the callback would be a data race, and the
  state could change between the check and the moment `update()` sees the step anyway.

#### Synchronization: `None`, not `Phase`

With `Phase`, all axes move on a straight line, but only if Ruckig's current motion is collinear
with the remaining distance. A second click on another axis while the first is still moving breaks
that, and Ruckig falls back to `Time` sync: all axes must finish together, so **the first axis
suddenly slows down** to wait for the second. With `None`, every axis has its own time-optimal
profile: x finishes its step as if nothing happened, y runs its own, the path is a rounded L. With
one axis per click both are identical anyway.

#### Pre-scaling without a target velocity

In jog mode the pre-scaling scales the operator's twist. In position mode there is no target
velocity; Ruckig decides how fast it goes. The lever is the limit instead: velocity mode ignores
`max_velocity`, position mode enforces it. So

$$
\texttt{max\_velocity}_i = s \cdot v_{max,i}
$$

with the same $s$ for all six DOFs. **Only the velocity is scaled**: what's protected are joint
velocity limits and the velocity ramp in front of position limits. A lower acceleration limit would
only make braking weaker, i.e. the braking distance in front of a joint limit longer.

The scaling is **exact, not conservative**: $s\cdot v_{max}$ is the highest speed the joints allow,
and Ruckig moves at $\min(\text{own peak}, s\cdot v_{max})$. A 2 mm step peaks at about 17 mm/s
with the default jerk limits, so a cap of $s\cdot 100$ mm/s only has an effect below $s \approx 0.17$.

**What twist to predict with?** With `None`, every axis that still has distance to go moves
somewhere between standstill and full speed towards its target, independently of the others:

$$
v_i \in [\,0,\ \operatorname{sign}(d_i)\,v_{max,i}\,], \qquad d = \texttt{target\_position} - \texttt{current\_position}
$$

All twists that can occur form a **box** (a rectangle for two moving axes):

```
 v_y
 −vmax ┌───────────┐            example: d = (+2 mm, −1 mm, 0, ...)
       │  possible │            x moves forwards: v_x ∈ [0, +vmax]
       │   twists  │            y moves backwards: v_y ∈ [0, −vmax]
     0 └───────────┘
       0          +vmax  v_x
```

The pre-scaling has to hold for **every** point of the box, e.g. right after a click on y, x is
still at full speed while y hasn't started yet.

**The worst case is always at a corner.** A joint velocity is linear in the twist:
$\dot q_j = a\,v_x + b\,v_y$ (a row of $J^+$). Hold $v_y$ fixed: a linear function of $v_x$ is
largest at one end of its interval, never in the middle. The same holds for $v_y$, so the worst
point has both at an end, i.e. it's a corner. Instead of infinitely many points, only the corners
have to be checked.

**All corners, not just "everything at full speed".** Example: $a = +3$, $b = -2$ (x pushes joint
$j$ forwards, y backwards), joint limit 0.25 rad/s, $v_{max} = 0.1$:

| Corner | Situation | $\dot q_j$ |
|---|---|---|
| (0.1, 0) | x at full speed, y not yet moving | **+0.3** rad/s |
| (0, −0.1) | x done, y at full speed | +0.2 rad/s |
| (0.1, −0.1) | both at full speed | +0.5 rad/s |

Here both axes add up, so "both at full speed" happens to be the worst. With $b = +2$ instead, the
axes cancel: "both" gives 0.1 rad/s, while "x alone" gives 0.3 rad/s. Checking only "both" would
then give $s = 1$, although the joint would run at 1.2× its limit right after the click.

**Building the corners.** Start with one corner (standstill). Every moving axis doubles the list:
each corner so far once without and once with that axis at full speed:

```
start:     [ (0, 0) ]
axis x:    [ (0, 0), (+v, 0) ]
axis y:    [ (0, 0), (+v, 0), (0, −v), (+v, −v) ]
```

Every corner except the first (standstill) then gets the same prediction as a jog twist: rotate
into base, $\dot q_{pred} = J^+ \dot x_{corner}$, joint velocity factor and position limit factor.
$s$ is the minimum over all corners. One moving axis gives one corner (the jog prediction once),
two give 3, all six at most 63. The corners live in a `std::array` of 64 fixed-size vectors on the
stack (64 = $2^6$, all six axes moving), so nothing is allocated.

**Not covered: an axis that reverses.** +1 mm and right after −2 mm on the same axis: Ruckig first
brakes, then reverses. While braking, the axis moves against $\operatorname{sign}(d_i)$, which the
box doesn't contain. That's accepted: braking only lowers the speed, the braking distance of
millimeter steps is tiny, and the post-scaling and the hard clamp are the backstop.

**Abort in front of a joint limit.** In the slow-down zone the position limit factor goes to 0 at
the margin. In jog mode that's harmless (target 0 means stop). In position mode it's not: with
`max_velocity = 0` and distance left, Ruckig has no solution and returns an error, which would
deactivate the controller; with a tiny limit it crawls asymptotically and never finishes. So
`s < 0.01` ends the run: Ruckig switches to velocity mode with target 0 and brakes from its current
(already slow) state. Clicks towards the limit then do nothing, clicks away from it work, because
the position limit factor is one-sided.

#### Post-scaling and the resync of the position

When the post-scaling clips ($s < 1$), only the fraction $s$ of this cycle's step is executed. Ruckig
would otherwise believe it moved the full step and report `Finished` short of the target. So the
resync also advances the position only by the executed part:

```cpp
current_position += s * (new_position - current_position);    // plus velocity, acceleration as before
```

In velocity mode Ruckig ignores positions, so this line runs in both modes.

#### Ending a run

- `Finished` in increment mode → back to velocity mode with target 0. Ruckig is at rest, the handover
  is seamless. The mode check matters: in velocity mode Ruckig also reports `Finished` as soon as the
  target velocity is reached, i.e. almost always while jogging.
- A new twist message → velocity mode **from the current state** (velocity and acceleration are not
  reset), so the arm continues or brakes without a jump. If the twist is in another frame, the
  normal frame switch rotates Ruckig's state.
- `s < 0.01` → abort, see above.

#### What is exact

Ruckig integrates each component of $\omega$ separately, while `x_ref` composes rotations
([background 3](#background-3-orientation-integration-and-error)). Both agree only while the
rotation axis doesn't change. Rotations don't commute ("10° about x, then 10° about y" ≠ the other
order), the difference is about $\tfrac12\,\theta_1\theta_2$.

| Case | Exact? |
|---|---|
| translations, any number of axes, also overlapping | yes |
| rotation about **one** axis, any size | yes |
| base frame: translation and rotation at the same time | yes (translate the TCP, rotate about the TCP) |
| clicks that don't overlap in time | yes, each step is done before the next starts |
| rotations about different axes overlapping in time | no: error $\approx \tfrac12\,\theta_1\theta_2$, e.g. 3° + 3° → 0.08° |
| tool frame: translation while rotating | no: the translation follows the turning tool axes |

Plus, as in jog mode: near singularities the arm falls short of the step ($J J^+$), and "1 mm" means
1 mm of `x_ref` in the URDF model, so the real arm is as accurate as its calibration. Moves are
relative and the operator sees the resulting pose, so this is the right trade for a jogger. Exact
paths or target poses (straight lines between poses, arcs, screw motions) are the job of a
planner, not of this controller.

### Safety and error handling

**Tracking check.** Every cycle, before anything is computed:

$$
\max_i \lvert q_{cmd,i} - q_{meas,i}\rvert > \texttt{max\_tracking\_error} \;\Rightarrow\; \texttt{ERROR}
$$

$q_{cmd}$ is the previous cycle's command, so the normal lag (one cycle plus the drive's following
error) has to fit within the threshold. It catches:

- a blocked arm or a collision,
- drive faults,
- **frozen state**: a hardware `read()` that stops updating still returns values. While jogging,
  $q_{cmd}$ moves and $q_{meas}$ doesn't. While standing still, nothing is commanded, so nothing is
  at risk.

There is deliberately no debounce. And never "recover" by setting $q_{cmd} = q_{meas}$, because that
hides exactly what should be caught, e.g. the arm pressing against an obstacle.

**Return `ERROR` means stop.** The controller manager deactivates a controller whose `update()`
doesn't return `OK` (and activates configured fallback controllers, if any). Here the hardware holds
position when the interfaces are released, so `ERROR` is a clean safety stop. It's used only for
real failures:

| Case | Handling |
|---|---|
| Tracking error exceeded | `ERROR` |
| Jacobian / FK failure (only on invalid input, e.g. NaN in $q_{cmd}$) | `ERROR` |
| Ruckig `result < 0` | `ERROR` (error code logged) |
| State read misses (`get_optional()` → `nullopt`) | keep last value, continue, throttled warning |
| Joint space clipping ($s<1$), hard position clamp | continue, throttled warning |
| Increment dropped (moving, wrong frame) or run aborted at a joint limit | continue, throttled warning |

Logs that come right before an `ERROR` aren't throttled (they happen once), while logs in paths
that continue are.

**State reads.** In hardware_interface (Jazzy), `get_optional()` returns `nullopt` **only on lock
contention**. It tries `try_lock` on a shared mutex up to 10 times with `yield()` in between.
Configuration errors (wrong type, missing pointer) throw instead. A miss is transient and harmless,
so `update()` uses the best-effort `fetch_robotarm_state()`, and a persistent problem shows up in
the tracking check. `on_activate` uses the strict variant, because $q_{cmd}$ is seeded from those
values. A failed activation is harmless: just activate again.

**Increments have no deadman.** Jogging stops 250 ms after the last message; an accepted increment
is executed even if the sender has gone quiet. What bounds it: each step is clamped to
`max_increment_*`, and steps only add up while a run is active. There is no cap on the distance
still outstanding (see [limitations](#known-limitations-and-roadmap)), so a sender that floods
the topic can queue up a long move. A new twist message (zero = stop) ends the run at any time.

### Realtime considerations

`update()` must not allocate. The rule for the data layout in
[CartesianJogController.hpp](include/robotarm_controllers/CartesianJogController.hpp):

> A **member** exists only if it (a) has to survive between cycles, or (b) has a runtime size and
> has to be preallocated. Everything else is a **local**.

| Struct | Contents | Lifetime |
|---|---|---|
| `Measured state` | $q_{meas}$, $\dot q_{meas}$ | refreshed each cycle |
| `Reference ref` | $q_{cmd}$, $\dot q_{cmd}$, `x_ref` | persistent, reset on activate |
| `Scratch tmp` | $\dot q_{pred}$, $J$, $J^+$ | valid within one cycle only |

- Cartesian quantities (`Matrix<double,6,1>`, `Vector3d`, `Isometry3d`, `AngleAxisd`) are
  fixed size, so they're **stack locals at zero cost**, with no resize and no member needed.
- Joint-space quantities ($n$ joints at runtime) are `VectorXd` members, sized once in
  `on_configure`. Use `noalias()` for products into them, and in-place updates.
- Passing a fixed-size vector to a `const VectorXd&` parameter **creates a heap temporary**.
  Joint-space vectors therefore always stay `VectorXd`.
- `robotarm_rbd` is allocation-free after `initialize()` (it has its own scratch buffers and
  error buffer).
- The realtime box holds a `geometry_msgs/Twist` plus a `Frame` enum, not the `TwistStamped`:
  `header.frame_id` is a `std::string`, and copying it in `update()` can allocate. The string is
  matched to the enum in the subscriber callback, outside the realtime thread.
- The increment box holds two fixed-size `Matrix<double,6,1>` slots. `update()` takes and clears
  them with `try_set`, a lambda capturing by reference fits `std::function`'s small buffer.
- The corners of the increment pre-scaling are a `std::array` of 64 fixed-size vectors on the stack
  (~3 KB). Ruckig's arrays are read and written through `Eigen::Map`, which views them without
  copying.
- `test_cartesian_jog_rt_alloc` enforces all of this from the very first cycle after activation.

### Known limitations and roadmap

| Item | Status / notes |
|---|---|
| ~~Command timeout (watchdog)~~ | **Done.** A non-zero twist older than 250 ms is set to zero, see [interfaces](#interfaces). Timeout is hard-coded (`twist_cmd_timeout_`), not a parameter. |
| **Singularity slowdown** | Missing as an explicit term. Velocity pre-scaling already slows down as $J^+$ grows. Planned: condition number $\kappa$, one-sided (MoveIt Servo style: compare $\kappa$ at $q + \dot q_{pred}\delta$). |
| **Adaptive damping** | Planned in `RobotarmRbd::calculate_jacobian_inverse`, removes the DLS error far from singularities (background 2). |
| ~~Tool-frame jogging~~ | **Done.** Selected with `header.frame_id`, Ruckig runs in the commanded frame, see [Ruckig in the commanded frame](#ruckig-in-the-commanded-frame). |
| Frame switch acceleration | The $\omega \times v$ term is dropped when Ruckig's state is rotated into the new frame (small jump in the starting acceleration, only when switching while translating and rotating). |
| ~~Position increments~~ | **Done.** `~/increment_cmds`, Ruckig in `Position` mode with `None` sync, fed only its own state, see [background 7](#background-7-incremental-moves). |
| Increment: no outstanding cap | Steps add up without a bound on the distance still to go. A cap per axis (e.g. 20 mm / 10°) would bound how far the arm moves after the last message. |
| Increment: rotations and tool frame | Overlapping rotations about different axes are off by $\approx \tfrac12\theta_1\theta_2$; a tool-frame translation during a rotation follows the turning tool axes. Freezing the tool orientation at the start of a run would make the tool frame behave like the base frame. |
| Increment: stop and click in one cycle | A zero twist and a click within the same cycle while idle start the run (the zero twist passes the "twist is zero" check). Practically never happens. |
| Increment: limit below the current speed | When the pre-scaling lowers `max_velocity` below Ruckig's current speed, Ruckig is expected to brake within its limits. Not verified, but millimeter steps barely get up to speed. |
| Increment: DLS shortfall | Steps fall short by what $J J^+$ attenuates: measured 0.5 % in the regular pose of the tests with `lambda = 0.01` (below $10^{-5}$ with `lambda = 0.001`). Adaptive damping would remove it. |
| Logging in the realtime path | Every printed log allocates (rosout over DDS). Only throttled warnings and errors before a stop are logged in `update()`. The rt_alloc test excludes them deliberately. |
| Per-axis twist clamp | Box limit: a diagonal motion can reach $\sqrt3\times$ the per-axis speed. |
| Hard clamp not in `x_ref` | Correction pushes against an active clamp (bounded); should never trigger with a sized zone. |
| $J$ computed twice | `calculate_jacobian_inverse` computes $J$ internally and the controller computes it again. An rbd call returning both would save one evaluation. |
| Cartesian limits not validated | `max_*_velocity/acceleration/jerk` aren't checked in `on_configure`. |
| `x_ref` orthonormality | The rotation isn't re-orthonormalized. Floating-point drift only matters for very long runs. |
| Joint acceleration / jerk | Not strictly bounded (background 4). Monitor with finite differences of $\dot q_{cmd}$. |

---

## TeleopController

> **Status:** skeleton. This section will be expanded once the controller is reworked.

### Usage

#### Interfaces

| Type | Name | Notes |
|---|---|---|
| Command interfaces | `<joint>/position`, `<joint>/velocity` | joints from the `joints` parameter |
| State interfaces | `<joint>/position`, `<joint>/velocity` | |
| Subscription | `~/commands` (`std_msgs/msg/Float64MultiArray`) | target joint positions in rad, one per joint |
| Service | `~/scale_speed` (`robotarm_interface/srv/SetFloat64`) | runtime speed scale |

#### Parameters

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `joints` | `[]` | – | Joint names (must not be empty) |
| `max_velocity` | `1.5` | rad/s | Ruckig velocity limit, all joints |
| `max_acceleration` | `2.0` | rad/s² | Ruckig acceleration limit |
| `max_jerk` | `2.0` | rad/s³ | Ruckig jerk limit |
| `sync_joints` | `false` | – | `true`: `Synchronization::Time` (all joints arrive together), `false`: `None` |

### Behavior

- Joint space Ruckig in **position mode**: the latest command is the target position, with target
  velocity 0.
- On the first cycle after activation, Ruckig is seeded from the measured state; after that, its
  own output is fed back (`pass_to_input`).
- Until the first command arrives, the reference is NaN and nothing is commanded.
- **Speed scaling** applies a time scaling: scaling time by $1/k$ scales the $n$-th derivative by
  $k^n$, so the limits are scaled as $v\cdot k$, $a\cdot k^2$, $j\cdot k^3$. That slows the motion
  down while keeping its shape.

### Background

<!-- TODO: design rationale, choice of position mode, synchronization, interaction with the
     cartesian jog controller (switching while moving) -->

### Known limitations

<!-- TODO -->
