# robotarm_mujoco

MuJoCo simulation of the robotarm, for the controllers that command **effort**. The position and
velocity controllers (teleop, Cartesian jog) keep using `mock_components` through
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

As on the real robot, the hardware component starts `inactive` and the impedance controller is only
loaded:

```bash
ros2 control set_hardware_component_state moteus_hardware_system active
ros2 control switch_controllers --activate cartesian_impedance_controller
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

In the viewer, double-click a link and Ctrl + right-drag to push it.

The viewer and RViz need access to your desktop's X server. If they fail with *"Authorization
required, but no authorization protocol specified"*, run `xhost +local:` on the host (not in the
container), once per login.

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
found) and the axis5/axis6 armature (left at 0). They are marked `ESTIMATE` or `TODO` in the file.
With armature 0, axis6's inertia is only 2.6e-5 kg·m², which is why its controller gains in
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
