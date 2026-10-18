# robotarm_mujoco

MuJoCo simulation of the robotarm, for the controllers that command **effort**. The position and
velocity controllers (joint stream, Cartesian jog) keep using `mock_components` through
`robotarm_bringup`, since the moteus internal loop is what they rely on.

The sim runs [`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control) as the
ros2_control hardware plugin, and uses the same `controllers.yaml` as the real robot.

## Usage

```bash
ros2 launch robotarm_mujoco sim.launch.py              # MuJoCo viewer window
ros2 launch robotarm_mujoco sim.launch.py headless:=true rviz:=true
```

| Launch argument | Default | |
|---|---|---|
| `tool` | `none` | file name in `robotarm_description/urdf/tools/` without `.xacro` |
| `headless` | `false` | no MuJoCo viewer |
| `rviz` | `false` | also start RViz |
| `software_rendering` | `true` | Mesa software OpenGL for the viewer and RViz. In the dev container, hardware rendering hangs on VS Code's forwarded X display; set `false` once the container has working GPU access |
| `sim_params` | installed `config/sim_params.yaml` | simulation parameters (armature, friction, timestep) |
| `controller_params` | none | yaml applied after `robotarm_bringup/config/controllers.yaml`, for gains you only want in the sim |
| `joint_state_rate` | `50` | Hz, `/joint_states` rate. `500` for the push test, 50 Hz is too coarse to fit a response |

As on the real robot, the hardware component starts `inactive` and the controllers are only loaded:
`gc_controller`, `cartesian_impedance_controller` and the upstream controllers chained to it,
`joint_stream_controller_impedance` and `cartesian_jog_controller_impedance`. The joint stream and jog
instances that drive position/velocity directly can't run here.

```bash
ros2 control set_hardware_component_state moteus_hardware_system active
ros2 control switch_controllers --activate gc_controller                     # gravity compensation and hold
ros2 control switch_controllers --deactivate gc_controller \
  --activate cartesian_impedance_controller joint_stream_controller_impedance   # impedance, streamed targets
```

Physics runs from the start, so the arm falls limp onto its joint limits before you activate
anything, like the real arm with the motors off. To start from a defined pose, pause the sim, reset
it with a pose override, then resume:

```bash
ros2 service call /mujoco_ros2_control_node/set_pause mujoco_ros2_control_msgs/srv/SetPause "{paused: true}"
ros2 service call /mujoco_ros2_control_node/reset_world mujoco_ros2_control_msgs/srv/ResetWorld \
  "{state_overrides: {joint_states: {name: [axis1,axis2,axis3,axis4,axis5,axis6], position: [0.3,-0.9,0.8,0.5,0.6,0.4]}}}"
ros2 service call /mujoco_ros2_control_node/set_pause mujoco_ros2_control_msgs/srv/SetPause "{paused: false}"
```

With the controller already active, its hold is still anchored at the old pose. Activating it after the
reset by hand is too slow: the CLI needs over a second after the unpause, and the limp arm falls in the
meantime. `push_test --pose` does the whole sequence correctly, see below.

In the viewer, double-click a link and Ctrl + right-drag to push it. This is only for rough poking: the
drag is a spring from the grab point to the mouse, so nothing moves until it beats gravity, joint limits or
the hold's breakaway torque, and the stretched spring then flings the arm. Its strength is
`mouse_stiffness` in `sim_params.yaml` (10, MuJoCo's default 100 overpowers the motors). For defined
pushes use the push test.

The viewer and RViz need access to your desktop's X server. If they fail with *"Authorization
required, but no authorization protocol specified"*, run `xhost +local:` on the host (not in the
container), once per login.

## Push test

`push_test` pushes a body with a defined wrench for a set time, releases it and fits the free response of
every joint: overshoot, damping ratio ζ, damped frequency and 5% settling time, next to the ζ and f_n the
gains should give at that pose (from the mass matrix, see
[docs/dynamics/README.md](../../docs/dynamics/README.md)).

```bash
ros2 launch robotarm_mujoco sim.launch.py joint_state_rate:=500
ros2 control set_hardware_component_state moteus_hardware_system active

# reset to a free pose, controller holding it, then 10 N down on the elbow for 0.3 s
ros2 run robotarm_mujoco push_test --pose 0.3 -0.9 0.8 0.5 0.6 0.4
# bend axis5: 0.5 N sideways at a 10 cm "tool tip" in front of the wrist center, held 1.5 s
ros2 run robotarm_mujoco push_test --pose 0.3 -0.9 0.8 0.5 0.6 0.4 \
  --body stage6 --point 0 0 0.1 --force 0.5 0 0 --duration 1.5
```

| Option | Default | |
|---|---|---|
| `--pose` | | 6 joint angles [rad]: deactivate the controller, pause, reset to the pose, activate again (the hold anchors there), unpause. Without it the test starts wherever the arm is |
| `--body` | `stage3` | MJCF body, `stage1`..`stage6` |
| `--force`, `--torque` | `0 0 -10`, `0 0 0` | N and Nm, world frame |
| `--point` | `0 0 0` | m, application point in the body frame |
| `--duration` | `0.3` | s, push length. Short: a knock; long: the arm settles at the deflection first, cleanest fit |
| `--record` | `3.0` | s, recorded after the release |
| `--csv` | | writes `t, axis1..axis6` for plotting, `t = 0` at the push |
| `--tool`, `--params`, `--controllers` | `none`, installed files | model and gains for the expected values |

Before pushing it prints the **joint torques of the push** at the current pose, and warns when

- a joint rests on a position limit or a link rests on the floor. They take part of the push and the
  response says nothing about the controller. The arm falls limp onto both before the controller is
  active, so use `--pose`.
- the pushed body's own joint gets no torque. `stage5` and `stage6` have their origin in the wrist center,
  where the axes of axis5 and axis6 cross: a force there turns neither. Use `--point` for a lever, and a
  force perpendicular to the joint axis (check the printed torques, the axes turn with the pose).

Reading the table: **anchor slid ≠ 0** means the push exceeded the breakaway torque and the hold moved;
stay below it (`τ_push < hold_breakaway_torque`) to measure ζ of the hold itself. **coupled** means the
joint moved further after the release than it was deflected at release, dragged by the other joints or
its momentum, so there is no decay to fit. Joints that moved less than 1 mrad are not evaluated.

The wrench comes from mujoco_ros2_control's `ExternalWrenchPlugin`, which `sim.launch.py` always loads
([config/sim_plugins.yaml](config/sim_plugins.yaml)), service `/external_wrench/apply_wrench`. The plugin
applies force and torque in the **body frame**, although its description says world frame: a vertical
"world" force on `stage3` turned the vertical axis1. `push_test` rotates its world frame input into the
body frame at the pose before the push.

## How the model is made

`sim.launch.py` converts the URDF to MJCF on **every start**, into `$ROS_HOME/robotarm_mujoco/`
(default `~/.ros/robotarm_mujoco/`). Nothing generated is committed. A changed URDF, tool or
`sim_params.yaml` takes effect on the next launch, after `colcon build` has installed it.

To get the model without ROS running, for example for MuJoCo's own viewer:

```bash
ros2 run robotarm_mujoco robotarm_mjcf -o /tmp/robotarm_mjcf [--tool none] [--params my_params.yaml]
ros2 run mujoco_vendor simulate /tmp/robotarm_mjcf/scene.xml
```

The converter ([`robotarm_mujoco/mjcf.py`](robotarm_mujoco/mjcf.py)) runs xacro on
[`urdf/robotarm_sim.urdf.xacro`](urdf/robotarm_sim.urdf.xacro), the same file the launch uses as
`robot_description`. It hands the URDF to MuJoCo's built-in URDF compiler, then adds what a URDF
cannot express:

- **Geoms.** Visual geoms go in group 2 with no contacts and keep the URDF colours. The convex
  collision pieces go in group 3, hidden by default.
- **Sites.** `Base_1`, `flange`, `tcp` and tool links sit on fixed joints, so MuJoCo merges them into
  their parent body. Each one gets a site at its frame instead.
- **Joint parameters** from `config/sim_params.yaml`: `armature`, `damping`, `frictionloss`.
- **Actuators.** One `<motor>` per joint, named like the joint and limited to the URDF effort limit.
- **Contacts.** The base is excluded against the first link, because MuJoCo's parent/child contact
  filter does not cover bodies fused into the world.
- **Scene.** `scene.xml` adds a floor and light and includes `robotarm.xml`.

The converter from `mujoco_ros2_control` is not used: it throws away the URDF collision meshes and
pulls in several extra Python packages.

## Simulation parameters

[`config/sim_params.yaml`](config/sim_params.yaml) holds what the URDF does not have, per joint:

- `armature`, or `rotor_inertia` with `gear_ratio`: the reflected rotor inertia,
  `rotor_inertia · gear_ratio²`. It dominates on the geared joints; for axis2, 30² multiplies the
  rotor inertia by 900.
- `damping` and `frictionloss`: belt and bearing friction. `frictionloss: 0` is the strict setting
  for checking gravity compensation, because friction would hide model errors.

Current values are **estimates**: the LA8308 rotor inertia (about 180 kg·mm², no datasheet value
found) and the wrist motor rotor inertia behind the axis5/axis6 armature. They are marked `ESTIMATE`
in the file. Without armature, axis6's inertia is only 2.6e-5 kg·m², which is why its controller gains in
`robotarm_bringup` are so small; revisit them once the wrist motors are known.

## Joint list duplication

The sim has its own ros2_control block
([`urdf/robotarm_sim.ros2_control.xacro`](urdf/robotarm_sim.ros2_control.xacro)): effort only,
no `torque_ff`, no transmissions. The joint list therefore exists both here and in
`robotarm_description`. `test_sim_ros2_control_joints_match_the_hardware` fails if they differ.

## What the sim does and does not model

It models rigid body dynamics from the CAD inertials, reflected rotor inertia, viscous and dry joint
friction, joint limits, effort limits and self-collision. It does not model the moteus current loop
or torque bandwidth, cogging, belt elasticity or backlash, encoder quantisation or CAN latency. The
controller loop runs at the configured 500 Hz against a 1 kHz physics step.

## Tests

```bash
colcon test --packages-select robotarm_mujoco && colcon test-result --verbose
```

[`test/test_mjcf.py`](test/test_mjcf.py) compares the converted model against Pinocchio built from
the same URDF:

- joints and limits
- gravity torques and the mass matrix, net of armature
- the TCP pose
- that pure gravity feed-forward holds a pose for 2 s
- that the vendored MuJoCo (the version the hardware plugin runs) compiles the scene

`robotarm_rbd` is checked against Pinocchio in its own tests. If both pass, the simulated dynamics
are the ones the controllers compute with.
