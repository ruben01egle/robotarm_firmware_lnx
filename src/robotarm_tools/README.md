# robotarm_tools

Tools for working on the real arm.

- [recorder](#recorder): records full rate joint states and drive diagnostics to a csv
- [motion_runner](#motion_runner): drives the arm through configured single axis sweeps and records them

## recorder

Records every message of a joint state broadcaster's `dynamic_joint_states` to a csv: full rate joint states and,
with `hw_diagnostics` on, the moteus drive diagnostics. Use it for tuning, oscillation analysis and identification
runs.

It talks to the controller manager by itself. On start it loads, configures and activates
`joint_state_broadcaster_hires` (500 Hz, in `robotarm_bringup/config/controllers.yaml`, not spawned at startup).
On stop it deactivates it again, unless it was already active before.

Needs a sourced workspace and a running controller manager with active hardware
(`allow_controller_activation_with_inactive_hardware: false` rejects activating the broadcaster otherwise).

### Command line

```bash
ros2 run robotarm_tools record -o slow_loaded.csv        # until Ctrl-C
ros2 run robotarm_tools record -o run.csv --duration 30  # stops after 30 s
```

| Argument | Default | |
|---|---|---|
| `-o`, `--output` | `joint_states_<date>_<time>.csv` | csv path, missing directories are created |
| `--duration` | `0` | s, `0` = until Ctrl-C |
| `--broadcaster` | `joint_state_broadcaster_hires` | broadcaster to record, must be in `controllers.yaml` |
| `--controller-manager` | `/controller_manager` | |
| `--rate` | `500` | Hz, publish rate of the broadcaster, only used to count missed samples |

At the end it prints the number of samples, the duration and how many samples were missed (gaps in the stamps).

### From Python

Blocking, same as the command line:

```python
from robotarm_tools.recorder import record

summary = record('run1.csv', duration=30)   # duration=0: until Ctrl-C
```

Inside a test script, record while the script moves the arm:

```python
from robotarm_tools.recorder import Recorder

with Recorder('ident/axis2_pose3.csv') as rec:
    for v in (0.01, 0.02, 0.04):
        rec.mark(f'v{v}_up')
        move_axis2(+0.6, v)
        rec.mark(f'v{v}_down')
        move_axis2(-0.6, v)
        rec.mark('')            # unlabeled from here on
```

`start()` and `stop()` instead of `with` work as well. `stop()` returns the same summary dict as `record()`:
`output`, `samples`, `duration`, `missed`, `columns`.

The recorder has its own node and executor in a background thread, so it does not interfere with the script's
own nodes and spinning. If the script has not called `rclpy.init()`, the recorder does so in `start()` (without
rclpy's signal handlers, Ctrl-C stays a `KeyboardInterrupt`) and shuts rclpy down in `stop()`. If the script
initializes rclpy with the default signal handlers, Ctrl-C shuts the context down before `stop()` runs, the
broadcaster can then not be deactivated and stays active (a warning says so). Use
`rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` in scripts that should be stoppable by Ctrl-C.

`Recorder` arguments:

| Argument | Default | |
|---|---|---|
| `output` | `joint_states_<date>_<time>.csv` | csv path |
| `broadcaster` | `joint_state_broadcaster_hires` | |
| `controller_manager` | `/controller_manager` | |
| `rate` | `500.0` | Hz, only for counting missed samples |
| `use_sim_time` | `False` | set `True` in simulation if the controller manager uses sim time, `mark()` compares against it |
| `node_name` | `robotarm_recorder` | must be unique if several recorders run at the same time |
| `timeout` | `5.0` | s, per controller manager service call |

### Segments

`mark(label)` labels all samples from now on with `label`, until the next `mark()`. The label applies by the
sample's stamp, the control loop time, not by when the message arrives. Every row is labeled exactly with the
segment it was recorded in, no matter the transport delay. Before the first `mark()` the segment is empty.

### csv layout

One row per broadcaster message:

| Column | |
|---|---|
| `stamp` | s, control loop time of the sample (message header) |
| `segment` | label of the last `mark()` |
| `<name>_<interface>` | one column per state interface, e.g. `axis2_position`, `axis2_effort` |

The columns are fixed by the first message, sorted by name. An interface missing from a later message is written
as `nan`. Joint values are in joint space: rad, rad/s, Nm.

With `hw_diagnostics: true` (`moteus_hardware_system` in `controllers.yaml`), there are additional
per-actuator columns, prefixed with the actuator names from the URDF transmissions (`axis1_motor` …
`axis4_motor`, `wrist_a`, `wrist_b`):

| Interface | Unit | |
|---|---|---|
| `rotor_position` | rad | rotor angle from the onboard encoder (encoder 0), within one rotor revolution `[0, 2π)`. Independent of homing. Electrical angle = `rotor_position * pole_pairs` |
| `pid_p`, `pid_i`, `pid_d` | Nm | proportional, integral and derivative term of the drive's position loop |
| `torque_ff` | Nm | feedforward torque the drive received |

They are actuator values: for the wrist, `wrist_a`/`wrist_b` are the two motors, not `axis5`/`axis6`. The torques
are in Nm at the drive's output, like the torque the drive reports for `effort`. All are int16 in the reply: 0.01 Nm
for the torques, 0.0001 rev for the rotor angle. In torque mode (`effort` interface) `pid_p` and `pid_d` are 0.

`hw_diagnostics` is read when the hardware component is initialized: changing it needs a restart of the bringup,
not only a reconfigure.

The regular `joint_state_broadcaster` (50 Hz, `/joint_states`) is limited to the joints and `position`,
`velocity`, `effort` in `controllers.yaml`, so the diagnostics only appear in the hires broadcaster.

## motion_runner

Drives the arm through a series of **sweeps** from a yaml config and records them, unattended. A sweep moves one
axis from a start pose by a stroke and back, at constant speed, once per configured speed. This gives clean
constant velocity segments of a single axis, with the other axes fixed in a chosen pose (and so a chosen load).

What it does:

1. Connects to the mode manager, `/joint_states`, `/robot_description` and the joint stream controller (its
   joints and limits). Checks the config against them: pose length, axis names, every pose and stroke end inside
   the URDF joint limits minus `limit_margin`, every speed within the controller's `max_velocity`.
2. Prints the plan with an estimated duration and a warning that the arm has to be able to move freely, and
   waits for `yes` (skip with `--yes`).
3. Requests mode `JOINT_STREAM` from the mode manager, which activates the hardware and the controllers.
4. Per sweep: moves to the start pose at `approach_speed`, starts a [recorder](#recorder) on `<sweep>.csv`,
   then for every speed moves out by the stroke and back. Before each move it waits `settle_time` at rest.
   Each move is a segment in the csv and in `run.json`.
5. Requests mode `ARMED` at the end. Also on an error or Ctrl-C: the motion stops with the switch to `ARMED`
   right away, the recorder is closed after that.

The speed of a move is set with the controller's `scale_speed` service, `scale = speed / max_velocity`. A move
accelerates with the full `max_acceleration`/`max_jerk` and runs at the requested speed for almost the whole
stroke.

A move counts as arrived when every joint is within `position_tolerance` of the target and slower than
`velocity_tolerance`. If that takes more than 1.5 times the expected duration plus 10 s, the run is aborted (arm
blocked, drive fault, tolerance too tight for the load).

### Command line

```bash
ros2 run robotarm_tools motion_runner axis2.yaml -o ~/logs                # all sweeps
ros2 run robotarm_tools motion_runner axis2.yaml -o ~/logs --sweeps ready forearm_out
ros2 run robotarm_tools motion_runner axis2.yaml --dry-run                # check and print the plan only
```

| Argument | Default | |
|---|---|---|
| `config` | | yaml, see below |
| `-o`, `--output-dir` | `.` | each run goes to `<dir>/<name>_<date>_<time>/` |
| `--sweeps` | all | only these sweeps, e.g. to repeat some of them |
| `--dry-run` | | only checks the config syntax and prints the plan, needs no ROS. Limits are not checked |
| `--yes` | | no confirmation |
| `--controller-manager` | `/controller_manager` | |
| `--mode-manager` | `/mode_manager` | |

Needs the bringup with the mode manager running, the hardware may be inactive. Set `hw_diagnostics: true` in
`controllers.yaml` to get the drive diagnostics in the recordings; the runner warns if they are missing.

### From Python

```python
from robotarm_tools.motion_runner import run

run_dir = run('axis2.yaml', output_dir='~/logs', sweeps=['ready'], confirm=True)
```

The config can also be a dict with the same structure. Without `rclpy.init()` by the caller, the runner
initializes rclpy itself without signal handlers. `MotionRunner(load_config(...), output_dir).run()` is the same
without the sweep selection.

### Config

Example: [config/example_axis2.yaml](config/example_axis2.yaml), installed to
`share/robotarm_tools/config/`.

```yaml
name: axis2_ripple              # run directory name, letters/digits/_/-
description: free text, copied to run.json

approach_speed: 0.3             # rad/s, moves to the start poses          (defaults shown)
settle_time: 1.0                # s at rest before each move, recorded
position_tolerance: 0.01        # rad, arrival check
velocity_tolerance: 0.02        # rad/s, arrival check
limit_margin: 0.05              # rad, distance to the URDF limits

poses:                          # all joints, in the order of the joint stream controller's joints
  ready: [0.0, -1.3, 1.5, 0.0, 1.0, 0.0]
  forearm_out: [0.0, -1.3, 0.5, 0.0, 0.0, 0.0]

defaults:                       # optional, for every sweep: axis, stroke, speeds, repeat
  axis: axis2
  stroke: 0.6                   # rad, signed: out to pose + stroke, back to the pose
  speeds: [0.01, 0.02, 0.04, 0.08]
  repeat: 1                     # all speeds this many times

sweeps:                         # in this order, a sweep's keys override the defaults
  - name: ready                 # unique, also the csv name
    pose: ready
  - name: forearm_out
    pose: forearm_out
    stroke: -0.6
```

Unknown keys are errors, so a typo does not silently fall back to a default. The runner checks the joint limits,
**not collisions**: check new poses and strokes on the arm, at low speed, before running a config unattended.

### Output

One directory per run, `<output-dir>/<name>_<date>_<time>/`:

| File | |
|---|---|
| `<sweep>.csv` | recording of the sweep, [recorder layout](#csv-layout). The `segment` column holds the segment label during a move and `settle` at rest |
| `run.json` | everything about the run, written after every segment, so an aborted run keeps what it did |

A new invocation never writes into an older run, a repeated sweep gets a new directory. Collect several runs by
their `run.json`.

`run.json`:

| Key | |
|---|---|
| `name`, `description` | from the config |
| `status` | `running`, `complete` or `aborted` |
| `error` | why it was aborted, else `null` |
| `started`, `finished` | local time, ISO 8601 |
| `host` | |
| `stamps` | the clock of the segment stamps: ROS time in s, the same as the csv `stamp` column |
| `controller` | joint stream controller name, `joints`, `max_velocity`, `max_acceleration`, `max_jerk` |
| `hw_diagnostics` | whether the recordings contain the drive diagnostics |
| `config` | the resolved config: all defaults filled in, only the sweeps that ran |
| `files` | per csv: `sweep`, `status`, `samples`, `missed`, `columns` |
| `segments` | one entry per move, in order |

A segment:

| Key | |
|---|---|
| `label` | `<sweep>_v<speed>_r<repeat>_<out\|back>`, as in the csv `segment` column |
| `file` | csv it is in |
| `sweep`, `axis`, `pose` | |
| `direction` | `out` (pose → pose + stroke) or `back` |
| `speed` | rad/s, commanded |
| `repeat` | 1 … `repeat` |
| `from`, `to` | rad, measured position of the axis at the start, target |
| `t_start`, `t_end` | s, stamp of the command and of the arrival. The csv rows with this label lie in between |
| `status` | `complete`, `running` (never left: run crashed) or `aborted` |

A segment covers the whole move, including the acceleration at the start and the braking at the end. For
constant speed, select the rows where the measured velocity is close to `speed`.
