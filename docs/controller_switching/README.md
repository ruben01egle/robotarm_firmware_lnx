# Controllers and how to switch them

Handover for clients that activate controllers on the robot arm (e.g. a GUI). Describes the
controller setup as of the feedforward chain (`ff_controller`) and what changed compared to
`1f88fa7`, the last commit before the impedance work. Source of truth: `robotarm_bringup/config/controllers.yaml` and
`robotarm_bringup/launch/robotarm.launch.py`. Controller internals are documented in
`src/robotarm_controllers/README.md`.

---

## What changed

| Before (`1f88fa7`) | Now | Note |
|---|---|---|
| `teleop_controller` (`robotarm_controllers/TeleopController`) | `joint_stream_controller` (`robotarm_controllers/JointStreamController`) | **renamed**, same behavior. Topic and service move with the name, see below |
| `cartesian_jog_controller` | `cartesian_jog_controller` | unchanged |
| `cartesian_impedance_controller` = gravity compensation | `gc_controller` | **gravity compensation now has its own name** |
| – | `cartesian_impedance_controller` = Cartesian impedance | **same name, new behavior**: it now holds the TCP with a virtual spring instead of floating |
| – | `joint_stream_controller_impedance` | new, joint stream feeding the impedance controller |
| – | `cartesian_jog_controller_impedance` | new, jogging feeding the impedance controller |
| – | stiffness of `cartesian_impedance_controller` | **new input**: settable at runtime as parameters, in the base or the tcp frame, see [runtime stiffness](#runtime-stiffness-cartesian_impedance_controller) |
| – | `ff_controller` | new: adds the arm dynamics as feedforward torque to the position controlled modes |
| `joint_stream_controller`, `cartesian_jog_controller`, `joint_trajectory_controller` drive the hardware directly | chained to `ff_controller` | **`ff_controller` has to be activated in the same request**, alone they fail to activate |

A client that activated `cartesian_impedance_controller` for gravity compensation (hand guiding)
has to activate `gc_controller` instead. A client that activated one of the position controlled
controllers alone has to add `ff_controller` to the same switch request.

---

## All controllers

All are spawned **inactive** by `robotarm.launch.py`, except `joint_state_broadcaster` (active).

| Name | Plugin | Claims | Input | Behavior |
|---|---|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | state only | – | publishes `/joint_states`, always active |
| `ff_controller` | `robotarm_controllers/FeedforwardController` | `position` + `velocity` + `torque_ff` of all joints, **exports reference interfaces** | reference interfaces from an upstream controller | position controlled with the arm dynamics as feedforward torque. Without upstream: holds the pose it was activated in |
| `joint_stream_controller` | `robotarm_controllers/JointStreamController` | the reference interfaces of `ff_controller` | topic + service, see below | position controlled, follows streamed joint targets |
| `cartesian_jog_controller` | `robotarm_controllers/CartesianJogController` | the reference interfaces of `ff_controller` | topics, see below | position controlled, Cartesian jogging and increments of the TCP |
| `joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` (stock) | the reference interfaces of `ff_controller` | `FollowJointTrajectory` action, see below | position controlled, executes trajectories (e.g. planned by MoveIt) |
| `gc_controller` | `robotarm_controllers/CartesianImpedanceController` (mode `gc`) | `effort` of all joints | none | gravity compensation, the arm floats and can be guided by hand, stays where it is let go |
| `cartesian_impedance_controller` | `robotarm_controllers/CartesianImpedanceController` (mode `impedance`) | `effort` of all joints, **exports reference interfaces** | reference interfaces from an upstream controller, stiffness parameters | compliant: the TCP is pulled towards a reference pose by a virtual spring/damper. Without upstream: holds the pose it was activated in |
| `joint_stream_controller_impedance` | `robotarm_controllers/JointStreamController` | the reference interfaces of `cartesian_impedance_controller` | same as `joint_stream_controller` | joint stream, executed compliantly |
| `cartesian_jog_controller_impedance` | `robotarm_controllers/CartesianJogController` | the reference interfaces of `cartesian_impedance_controller` | same as `cartesian_jog_controller` | jogging, executed compliantly |

The `*_impedance` instances are the same classes as their position controlled counterparts,
configured with `chain_to: "cartesian_impedance_controller"` instead of `chain_to: "ff_controller"`.
Only `ff_controller`, `gc_controller` and `cartesian_impedance_controller` touch the hardware.

---

## Chaining

Two controllers are chainable: `ff_controller` (position controlled) and
`cartesian_impedance_controller` (compliant). Both export the same kind of reference interfaces, and
the same rules apply to both.

```
joint_stream_controller      ─┐
cartesian_jog_controller     ─┼─(reference interfaces)─► ff_controller ─(position, velocity, torque_ff)─► hardware
joint_trajectory_controller  ─┘
```

- Only **one** upstream of `ff_controller` can be active at a time.
- The upstream can only be activated while `ff_controller` is active, or in the **same** switch
  request. Use the same request: activating `ff_controller` first and the upstream in a second
  request restarts `ff_controller`, and the feedforward briefly drops to 0 (axis 2 sags slightly).
- `ff_controller` can only be deactivated together with its active upstream, in the same request.
- Changing the upstream (e.g. 1 → 2) in one request keeps `ff_controller` active without a restart.
- `ff_controller` alone is valid (holds the activation pose), but not a combination to use.

`cartesian_impedance_controller` exports per joint
`cartesian_impedance_controller/<joint>/position`, `/velocity` and `/acceleration`. An upstream
controller claims them and writes a joint space reference every cycle:

```
joint_stream_controller_impedance  ─┐
                                    ├─(reference interfaces)─► cartesian_impedance_controller ─(effort)─► hardware
cartesian_jog_controller_impedance ─┘
```

- Only **one** upstream can be active at a time (both claim the same reference interfaces).
- The upstream can only be activated while `cartesian_impedance_controller` is active, or in the
  **same** switch request.
- `cartesian_impedance_controller` can only be deactivated together with its active upstream, in the
  same switch request.
- Without an upstream (or before the upstream's first command) the impedance controller holds the
  pose it was activated in.
- **Standalone has no reference input, by design**: no topic or service for a target pose. A new
  target only comes from a chained upstream, which guarantees a smooth trajectory. Standalone is
  the compliant hold (combination 4).
- `gc_controller` is not chainable and ignores references.

---

## Valid combinations

Exactly one of these is active at a time (plus `joint_state_broadcaster`):

| # | Active controllers | Arm behavior | Control |
|---|---|---|---|
| 0 | none | drives hold their position | – |
| 1 | `ff_controller` + `joint_stream_controller` | position controlled | joint targets |
| 2 | `ff_controller` + `cartesian_jog_controller` | position controlled | twists / increments |
| 3 | `gc_controller` | floats, hand guiding | – |
| 4 | `cartesian_impedance_controller` | compliant hold of the activation pose | – |
| 5 | `cartesian_impedance_controller` + `joint_stream_controller_impedance` | compliant | joint targets |
| 6 | `cartesian_impedance_controller` + `cartesian_jog_controller_impedance` | compliant | twists / increments |
| 7 | `ff_controller` + `joint_trajectory_controller` | position controlled | trajectories |

**Why they are exclusive:**
- 1, 2 and 7 claim the same reference interfaces of `ff_controller`, the controller manager rejects
  two of them.
- 3 and 4–6 claim the same `effort` interfaces, the controller manager rejects both.
- `effort` and `position`/`velocity`/`torque_ff` on the same joint at once (e.g. 1 + 3) is rejected
  by the hardware interface.

The controller manager runs with `strictness: strict`: a switch request that is not valid as a whole
fails completely and changes nothing.

---

## Switching

Always switch from one combination to the next in **one** request: deactivate everything of the old
combination and activate everything of the new one together. The hardware validates the final state
of a request, so a direct change between position and effort control works in one step.

CLI equivalents (the service is `/controller_manager/switch_controller`,
`controller_manager_msgs/srv/SwitchController`):

```bash
# 0 -> 7: start trajectory execution
ros2 control switch_controllers --activate ff_controller joint_trajectory_controller

# 7 -> 1: change the upstream, ff_controller stays active
ros2 control switch_controllers --deactivate joint_trajectory_controller \
  --activate joint_stream_controller

# 1 -> 3: joint stream to gravity compensation
ros2 control switch_controllers --deactivate joint_stream_controller ff_controller \
  --activate gc_controller

# 3 -> 5: gravity compensation to compliant joint streaming
ros2 control switch_controllers --deactivate gc_controller \
  --activate cartesian_impedance_controller joint_stream_controller_impedance

# 5 -> 6: change the upstream, the impedance controller stays active
ros2 control switch_controllers --deactivate joint_stream_controller_impedance \
  --activate cartesian_jog_controller_impedance

# 6 -> 2: compliant jogging to position controlled jogging
ros2 control switch_controllers \
  --deactivate cartesian_jog_controller_impedance cartesian_impedance_controller \
  --activate ff_controller cartesian_jog_controller
```

**Hardware:** `moteus_hardware_system` starts `inactive` (`hardware_components_initial_state`), and
controllers can't be activated while it is inactive. It has to be activated first:

```bash
ros2 control set_hardware_component_state moteus_hardware_system active
```

---

## Inputs per controller

Topic and service names are relative to the controller name, so the direct and the chained instance
of a class have the same interface under different names.

| Controller | Name | Type | Notes |
|---|---|---|---|
| `joint_stream_controller`, `joint_stream_controller_impedance` | `/<name>/commands` | `std_msgs/msg/Float64MultiArray` | target joint positions in rad, 6 values, order `axis1`…`axis6` |
| | `/<name>/scale_speed` (service) | `robotarm_interface/srv/SetFloat64` | speed scale in (0, 1], other values are rejected (`success = false`) |
| `cartesian_jog_controller`, `cartesian_jog_controller_impedance` | `/<name>/twist_cmds` | `geometry_msgs/msg/TwistStamped` | TCP velocity, `header.frame_id` empty or the base link = base frame, `tcp` = tool frame. A non-zero twist has to be repeated at > 4 Hz, it is zeroed after 250 ms |
| | `/<name>/increment_cmds` | `robotarm_interface/msg/CartesianIncrement` | relative TCP step, same frame selection |
| `cartesian_impedance_controller` | parameters `cartesian_stiffness_frame`, `cartesian_linear_stiffness`, `cartesian_angular_stiffness` | ROS parameters | stiffness and its frame, settable at runtime, see [runtime stiffness](#runtime-stiffness-cartesian_impedance_controller). No topics or services |
| `gc_controller` | – | – | no topics or services, rejects stiffness sets |
| `joint_trajectory_controller` | `/joint_trajectory_controller/follow_joint_trajectory` (action) | `control_msgs/action/FollowJointTrajectory` | joint trajectory, all six joints. Include accelerations (e.g. MoveIt time parameterization), otherwise the feedforward jumps at every waypoint. A goal is aborted when the tracking error exceeds the `constraints` in `controllers.yaml` |
| `ff_controller` | – | – | no topics or services |

Renamed topics from before: `/teleop_controller/commands` → `/joint_stream_controller/commands`,
`/teleop_controller/scale_speed` → `/joint_stream_controller/scale_speed`.

---

## Runtime stiffness (`cartesian_impedance_controller`)

The Cartesian spring of `cartesian_impedance_controller` is set through ROS parameters on its node
`/cartesian_impedance_controller`. The parameters always hold the value the controller uses, so
reading them gives the current state.

| Parameter | Type | Unit | Runtime | Meaning |
|---|---|---|---|---|
| `cartesian_stiffness_frame` | string | – | yes | `"base"` or `"tcp"`: frame the six stiffness values are given in |
| `cartesian_linear_stiffness` | double[3] | N/m | yes | stiffness along x, y, z of that frame |
| `cartesian_angular_stiffness` | double[3] | N·m/rad | yes | stiffness about x, y, z of that frame |
| `max_linear_stiffness` | double | N/m | no | per axis upper bound of the linear values |
| `max_angular_stiffness` | double | N·m/rad | no | per axis upper bound of the angular values |
| `cartesian_damping_ratio` | double | – | no | damping ratio ζ, the damping follows the stiffness automatically |

"Runtime: no" parameters are only read when the controller is configured. Setting them later is
accepted but has no effect until a reconfigure, a client should treat them as read only.

**Frame variants:**
- `base`: the axes are the fixed axes of the robot base. A stiff z keeps the TCP height regardless
  of the tool orientation.
- `tcp`: the axes move with the tool (with the reference orientation of the TCP). A stiff tcp z keeps
  the tool from moving along its own axis, wherever it points.
- Switching the frame does not convert the values, the same six numbers are reinterpreted in the
  new frame. With equal values on all three axes (per linear / angular) the frame makes no
  difference.

**Chained or standalone:** the stiffness works the same in combinations 4, 5 and 6, it belongs to
`cartesian_impedance_controller` and not to the upstream. Standalone (4) it softens or stiffens the
hold of the activation pose, and changing the upstream (5 ↔ 6) keeps it.

**Setting:**
- Each value has to be finite, ≥ 0 and ≤ its limit, each array exactly 3 entries, the frame `base`
  or `tcp`. Anything else is rejected with `successful = false` and a `reason`, and the controller
  keeps its previous values.
- Sets are always rejected on `gc_controller` (same class, gc mode).
- Allowed while `cartesian_impedance_controller` is active or inactive. Active: takes effect in the
  next control cycle. Inactive: used from the next activation on. The values survive a
  deactivate / activate.
- The new value acts **immediately, without a blend**. With the arm away from its reference
  (pushed, in contact), raising the stiffness steps the spring force by ΔK · error, bounded by the
  spring force limits (`max_spring_force`, `max_spring_moment`). A frame switch with unequal values
  does the same.
- Set the frame and the six values in **one** call to `set_parameters_atomically`: either all are
  applied or none. With separate calls the controller can run a cycle with the new frame and the old
  values.

**Services and topics** (standard ROS parameter interface of the node):

| Name | Type | Use |
|---|---|---|
| `/cartesian_impedance_controller/get_parameters` | `rcl_interfaces/srv/GetParameters` | read values, frame and limits |
| `/cartesian_impedance_controller/set_parameters_atomically` | `rcl_interfaces/srv/SetParametersAtomically` | apply frame and values together, one result with `successful` and `reason` |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | committed changes of all nodes, filter on `node == "/cartesian_impedance_controller"` |

CLI for testing (one parameter per call, not atomic):

```bash
ros2 param get /cartesian_impedance_controller cartesian_linear_stiffness
ros2 param set /cartesian_impedance_controller cartesian_linear_stiffness "[500.0, 500.0, 300.0]"
ros2 param set /cartesian_impedance_controller cartesian_stiffness_frame tcp
```

### Suggested GUI behavior

Functionality only, not the look. The GUI keeps two sets of values: **committed** (what the
controller uses, from the parameters) and **staged** (what the user is editing). Only Apply sends
anything.

| Element / event | Behavior |
|---|---|
| automatically on activation of the controller | Read frame, the six values and both limits with `get_parameters`. Set the slider ranges to [0, limit] per linear / angular axis. Committed = staged = the read values. Subscribe to `/parameter_events`. |
| Sliders, value fields, frame selector | Change the **staged** values only, nothing is sent. Input outside [0, limit] is not possible. |
| Pending indicator | Shows whether staged differs from committed, per value or as a whole. |
| **Apply now** | Enabled only when staged ≠ committed. Sends the frame and all six values in one `set_parameters_atomically` call. `successful`: committed = staged. Rejected: show the `reason`, keep the staged values for correcting, committed stays unchanged. Disable the controls until the response arrives. |
| **Reset to current** | Staged = committed, discards the edits, sends nothing. Optionally re-read with `get_parameters` first. |
| Change from another client (`/parameter_events`) | Update committed. Without pending edits the staged values follow. With pending edits keep them, and mark that the committed values changed underneath. |
| Controller state | The controls apply to `cartesian_impedance_controller` only. Editing is possible while it is inactive (takes effect on activation). While `gc_controller` is active the settings have no effect on the arm, show that they are for the impedance controller. After a reconfigure (or a restart of the controller manager) read everything again, the limits may have changed. |
| Frame change in the staged set | Hint that the numbers are reinterpreted, not converted. |

Optional, if the step on Apply turns out too harsh in testing: confirm before applying an increase
while the controller is active. A gradual change belongs into the controller (not yet implemented),
sending intermediate values from the GUI does not guarantee a smooth change.

---

## Behavior worth knowing on the client side

- **Activation of an effort controller** (`gc_controller`, `cartesian_impedance_controller`): the
  torque is ramped in over 1 s from the measured effort, so the arm is not yet fully held right after
  activation.
- **Activation of `ff_controller`**: the moteus position loops hold the arm from the first cycle.
  The feedforward torque is ramped in from 0 over 1 s on top.
- **Activation of an upstream**: it starts from the measured joint position, so it does not jump.
  A new joint stream target or twist is only executed after activation.
- **Error stop**: a controller whose update fails is deactivated by the controller manager. When
  `cartesian_impedance_controller` or `ff_controller` stops, the drives hold position. When an
  upstream stops (e.g. the jog controller's tracking check), its downstream controller holds the last
  reference.
  Which other controllers of a chain the controller manager deactivates along with a failing one
  is not verified yet on this setup. A client should read the controller states
  (`/controller_manager/list_controllers`) rather than assume a switch stayed in effect.
- **Simulation** (`robotarm_mujoco/sim.launch.py`) loads the same controllers, but its hardware has
  no `torque_ff` interface yet. `ff_controller` can't be activated there, so combinations 1, 2 and 7
  are currently not available in the sim.
