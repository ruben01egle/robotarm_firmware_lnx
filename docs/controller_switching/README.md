# Controllers and how to switch them

Handover for clients that activate controllers on the robot arm (e.g. a GUI). Describes the
controller setup as of commit `20c9a13` and what changed compared to `1f88fa7`, the last commit before
the impedance work. Source of truth: `robotarm_bringup/config/controllers.yaml` and
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

A client that activated `cartesian_impedance_controller` for gravity compensation (hand guiding)
has to activate `gc_controller` instead.

---

## All controllers

All are spawned **inactive** by `robotarm.launch.py`, except `joint_state_broadcaster` (active).

| Name | Plugin | Claims | Input | Behavior |
|---|---|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | state only | – | publishes `/joint_states`, always active |
| `joint_stream_controller` | `robotarm_controllers/JointStreamController` | `position` + `velocity` of all joints | topic + service, see below | position controlled, follows streamed joint targets |
| `cartesian_jog_controller` | `robotarm_controllers/CartesianJogController` | `position` + `velocity` of all joints | topics, see below | position controlled, Cartesian jogging and increments of the TCP |
| `gc_controller` | `robotarm_controllers/CartesianImpedanceController` (mode `gc`) | `effort` of all joints | none | gravity compensation, the arm floats and can be guided by hand, stays where it is let go |
| `cartesian_impedance_controller` | `robotarm_controllers/CartesianImpedanceController` (mode `impedance`) | `effort` of all joints, **exports reference interfaces** | reference interfaces from an upstream controller | compliant: the TCP is pulled towards a reference pose by a virtual spring/damper. Without upstream: holds the pose it was activated in |
| `joint_stream_controller_impedance` | `robotarm_controllers/JointStreamController` | the reference interfaces of `cartesian_impedance_controller` | same as `joint_stream_controller` | joint stream, executed compliantly |
| `cartesian_jog_controller_impedance` | `robotarm_controllers/CartesianJogController` | the reference interfaces of `cartesian_impedance_controller` | same as `cartesian_jog_controller` | jogging, executed compliantly |

The `*_impedance` instances are the same classes as their direct counterparts, configured with
`chain_to: "cartesian_impedance_controller"`. They don't touch the hardware.

---

## Chaining

`cartesian_impedance_controller` is a chainable controller. It exports per joint
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
- `gc_controller` is not chainable and ignores references.

---

## Valid combinations

Exactly one of these is active at a time (plus `joint_state_broadcaster`):

| # | Active controllers | Arm behavior | Control |
|---|---|---|---|
| 0 | none | drives hold their position | – |
| 1 | `joint_stream_controller` | position controlled | joint targets |
| 2 | `cartesian_jog_controller` | position controlled | twists / increments |
| 3 | `gc_controller` | floats, hand guiding | – |
| 4 | `cartesian_impedance_controller` | compliant hold of the activation pose | – |
| 5 | `cartesian_impedance_controller` + `joint_stream_controller_impedance` | compliant | joint targets |
| 6 | `cartesian_impedance_controller` + `cartesian_jog_controller_impedance` | compliant | twists / increments |

**Why they are exclusive:**
- 1 and 2 claim the same `position`/`velocity` interfaces, the controller manager rejects both.
- 3 and 4–6 claim the same `effort` interfaces, the controller manager rejects both.
- `effort` and `position`/`velocity` on the same joint at once (e.g. 1 + 3) is rejected by the
  hardware interface.

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
# 1 -> 3: joint stream to gravity compensation
ros2 control switch_controllers --deactivate joint_stream_controller --activate gc_controller

# 3 -> 5: gravity compensation to compliant joint streaming
ros2 control switch_controllers --deactivate gc_controller \
  --activate cartesian_impedance_controller joint_stream_controller_impedance

# 5 -> 6: change the upstream, the impedance controller stays active
ros2 control switch_controllers --deactivate joint_stream_controller_impedance \
  --activate cartesian_jog_controller_impedance

# 6 -> 2: compliant jogging to position controlled jogging
ros2 control switch_controllers \
  --deactivate cartesian_jog_controller_impedance cartesian_impedance_controller \
  --activate cartesian_jog_controller
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
| `gc_controller`, `cartesian_impedance_controller` | – | – | no topics or services |

Renamed topics from before: `/teleop_controller/commands` → `/joint_stream_controller/commands`,
`/teleop_controller/scale_speed` → `/joint_stream_controller/scale_speed`.

---

## Behavior worth knowing on the client side

- **Activation of an effort controller** (`gc_controller`, `cartesian_impedance_controller`): the
  torque is ramped in over 1 s from the measured effort, so the arm is not yet fully held right after
  activation.
- **Activation of an upstream**: it starts from the measured joint position, so it does not jump.
  A new joint stream target or twist is only executed after activation.
- **Error stop**: a controller whose update fails is deactivated by the controller manager. When
  `cartesian_impedance_controller` stops, the drives hold position. When an upstream stops (e.g. the
  jog controller's tracking check), `cartesian_impedance_controller` holds the last reference.
  Which other controllers of a chain the controller manager deactivates along with a failing one
  is not verified yet on this setup. A client should read the controller states
  (`/controller_manager/list_controllers`) rather than assume a switch stayed in effect.
- **Simulation** (`robotarm_mujoco/sim.launch.py`) only has effort interfaces: it loads
  `gc_controller`, `cartesian_impedance_controller` and both `*_impedance` upstreams, but not
  `joint_stream_controller` and `cartesian_jog_controller`. Combinations 1 and 2 are not available
  there.
