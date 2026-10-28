# robotarm_mujoco_hardware

ros2_control hardware plugin of the MuJoCo simulation (`robotarm_mujoco_hardware/RobotarmMujocoSystem`). It is
[`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control)'s `MujocoSystemInterface` (0.1.2)
used as an effort-only physics backend, with a layer on top that behaves like the real arm towards the
controllers. The physics, the viewer, the reset/pause services and the plugins are the base class.

Kept as its own `ament_cmake` package so `robotarm_mujoco` stays a Python package, and as the place for later
sim improvements on the hardware side.

## What it does

| Claimed on a joint | Torque to the joint |
|---|---|
| `effort` | the command, as it is (NaN → 0) |
| `position` and/or `velocity` (and `torque_ff`) | servo, `tau = kp·(p* − p) + kd·(v* − v) + ki·∫(p* − p) dt + tau_ff`, before every physics step |
| nothing | servo holding the position the joint had at the release (or at activating the hardware) |

- **Servo** ([`JointServo.hpp`](include/robotarm_mujoco_hardware/JointServo.hpp)): the structure of the
  moteus position loop, on the velocity MuJoCo measures. Nothing is differentiated. Output (feedforward
  included) and integral clamped to the URDF effort limit, the integral pauses while the output saturates. Gains from the hardware
  parameter `pids_config_file`, see the
  [robotarm_mujoco README](../robotarm_mujoco/README.md#position-and-velocity-servo).
- **On the physics thread** ([`ServoPlugin.cpp`](src/ServoPlugin.cpp)): a `mujoco_ros2_control` plugin runs
  the servos in `pre_step()`, before every physics step (1 kHz), with the live state and the physics time. The
  hardware interface only posts each joint's target (`p*`, `v*` and the controller manager time `t_c` it
  belongs to) through the [`ServoBridge`](include/robotarm_mujoco_hardware/ServoBridge.hpp); the plugin moves it
  to the physics time, `p* + v*·(t − t_c)` (at most 50 ms ahead, so a stalled controller manager cannot let it
  run away). Under load the controller manager's time lags the physics by a few ms and jitters, a servo in the
  control loop compared targets and states of different moments and the torque spiked. The plugin has to be
  loaded (`mujoco_plugins` parameter, `robotarm_mujoco/config/sim_plugins.yaml`), the hardware interface
  refuses to start without it.
- **`torque_ff`** (written by `ff_controller`): the moteus feedforward torque. It is clamped to the effort
  limit (NaN → 0), posted with the target and held until the next one, then added to the servo torque. It
  belongs to the position/velocity group: claimed alone, the servo holds the position and adds it. The sim
  servo is much stiffer than the moteus loop (ω = 80 rad/s, `sim_pids.yaml`), so in the sim feedforward
  shows up mostly as an integral near 0 and less lag at high accelerations, not as a big tracking gain.
- **NaN position or velocity only:** `p*` integrates the velocity command from the position at the claim,
  like the moteus does with a NaN position.
- **Newly claimed commands** start at NaN position / 0 velocity / 0 torque_ff / 0 effort, like in `moteus_interface`, so a
  stale value of the previous controller is never used.
- **Effort together with position/velocity/torque_ff** on one joint is rejected in `prepare_command_mode_switch`, like
  by `moteus_interface`.
- **World reset** (`reset_world` service, viewer): joints without a position command hold the reset pose
  (signalled by the plugin's `on_reset`).
- **Hardware deactivated:** the servos let go, the arm hangs limp, like the real one with the motors stopped.

## Why not `mujoco_ros2_control`'s own position/velocity

It has PIDs for `<motor>` actuators, but:

- With `position` and `velocity` claimed on a joint, each started interface replaces the joint's mode, so only
  the last one runs: the velocity PID, and the position command is ignored.
- Its PID differentiates the position error over the controller period. The sim's periods jitter (2 or 3 ms of
  sim time) and the measured position is not always from the same physics step as the period assumes, so
  the derivative is noisy: the torque bounced between the effort limits while the arm moved smoothly.
- Nothing claimed: drives to the start command (all zero) after activation, keeps the last torque after a
  release. A world reset switches every joint to its position PID, also joints an effort controller claims.

So the base class gets only `effort` and no PIDs (stripped from its `HardwareInfo` in `on_init`), and this
plugin computes every torque.

## How

`on_init` reads the URDF's command interfaces and the servo gains, passes the base class a copy of the
`HardwareInfo` with only `effort` and without `pids_config_file` (the base class loads the `mujoco_plugins`
there), then hands joints and gains to the bridge. The hardware interface exports its own command interfaces
(position, velocity, torque_ff, effort as in the URDF) and copies of the base class's state interfaces: `read()` copies
the states in, `write()` gives effort-claimed torques to the base class's effort interfaces and posts the
servo targets. Both classes are in this one library, so they share the bridge.

## Tests

```bash
colcon test --packages-select robotarm_mujoco_hardware && colcon test-result --verbose
```

`test_joint_servo` covers the servo law (P, D on the velocity error, integral, feedforward, clamps, zero period), the gains
file and the bridge. The plugins themselves need a running sim and have no automated test: they were checked
by hand with the controller switches, releases, resets and the torque during joint stream moves, also with the
viewer and the CPU fully loaded.
