# robotarm_mujoco_hardware

ros2_control hardware plugin of the MuJoCo simulation (`robotarm_mujoco_hardware/RobotarmMujocoSystem`). It is
[`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control)'s `MujocoSystemInterface` (0.1.2)
with a thin layer on top, so the sim behaves like the real arm towards the controllers. Everything else, the
physics, the viewer, the reset/pause services and the plugins, is the base class.

Kept as its own `ament_cmake` package so `robotarm_mujoco` stays a Python package, and as the place for later
sim improvements on the hardware side (e.g. a moteus-like position loop).

## What it changes

| Situation | `mujoco_ros2_control` alone | here |
|---|---|---|
| a controller claims `position` and `velocity` of a joint (joint stream, Cartesian jog) | the started interfaces are applied one by one and each one replaces the joint's mode, so the last one wins: the velocity PID runs and the position command is ignored | the position PID runs (position is passed last, `position_last()`) |
| nothing claimed, after activating the hardware | the position PID drives to the start command, all zero | holds the current position |
| nothing claimed, after a controller is released | no mode, the last torque stays applied | holds the position at the release |
| a newly claimed command | keeps the previous controller's value | starts at NaN position / 0 velocity / 0 effort, like `moteus_interface` |
| NaN position command | NaN reaches the PID | holds the position the joint had at the claim (moteus: NaN = current position) |
| world reset (`reset_world` service, viewer) | every joint switches to position hold at the reset pose, also joints an effort controller claims | the claimed modes are restored, idle joints hold the reset pose |

"Holds" means the joint's position PID with that position as its target, the closest the sim has to the
moteus idle mode (NaN position, 0 velocity: hold where you are). The PID gains are in
`robotarm_mujoco/config/sim_pids.yaml`, see the [robotarm_mujoco README](../robotarm_mujoco/README.md).

## How

The interfaces the controllers see are the plugin's own copies of the base class's interfaces (same names):
`read()` copies the states in, `write()` the commands out, with the hold and NaN handling above.
`perform_command_mode_switch()` tracks the claims per joint, puts position last and adds a position claim for
every joint left idle. A reset is detected in `write()`: outside a reset the base class never changes a
command value itself, so a position command that differs from what `write()` gave it last means a reset.

## Upstream

The "last interface wins" behaviour is that of `mujoco_ros2_control` 0.1.2 (`perform_command_mode_switch`).
Check newer versions before upgrading: if position + velocity is handled there, `position_last()` can go.

## Tests

```bash
colcon test --packages-select robotarm_mujoco_hardware && colcon test-result --verbose
```

`test_position_last` covers the reordering. The rest needs a running sim and has no automated test: it was
checked by hand with the controller switches, releases and resets of the `robotarm_mujoco` README.
