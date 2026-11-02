# Identification runs

Sweep configs for the [motion runner](../../src/robotarm_tools/README.md#motion_runner), to identify the torque ripple
(orders 20/40 of the LA8308, 14/28 of the MN5008) and the friction of every axis. Background: the axes oscillate
in some poses during slow, loaded moves, worst on axis 2, locked to the rotor angle at 20 and 40 per motor
revolution and amplified by a closed loop resonance around 4 Hz.

| Config | Content | Duration |
|---|---|---|
| [check.yaml](check.yaml) | one axis 2 sweep, 2 speeds: functional check before the real runs | ~2 min |
| [axis2.yaml](axis2.yaml) | 2 load levels + torque sign change, dense frequency grid for orders 20 and 40 | ~38 min |
| [axis3.yaml](axis3.yaml) | 2 load levels, medium grid | ~13 min |
| [axis1.yaml](axis1.yaml) | arm tucked and extended (inertia, no gravity load), coarse grid | ~10 min |
| [axis4.yaml](axis4.yaml) | one pose, coarse grid | ~6 min |
| [wrist.yaml](wrist.yaml) | axis 5 and 6 (both wrist motors), one pose each, coarse grid | ~8 min |

Durations include the moves to the start poses. Run them in this order, axis 2 first: if the session is cut
short, the most important data is complete.

```bash
ros2 run robotarm_tools motion_runner docs/identification/axis2.yaml -o ~/identification
```

## Design

- **Speeds from a ripple frequency grid.** The ripple of order 20 has the frequency
  `f20 = 20 * ratio * v / 2pi`, so the same frequency needs a different speed on every axis. Each config lists
  the speeds with their frequency. Dense around the 4 Hz resonance of axis 2 for both orders: order 40 runs at
  `2 * f20`, so it passes the resonance at f20 = 1.5 .. 2.5 Hz, order 20 at f20 = 3 .. 5 Hz. Coarser where the
  ripple is less pronounced or the resonance is unknown: a log grid of 1 .. 16 Hz covers order 40 at 2 .. 32 Hz. Above the resonance the response falls with about 1/f², so the grid
  stops at 12-16 Hz. A few fast speeds per axis for friction. These cover high ripple frequencies anyway.
- **Stroke: 3 rotor revolutions**, enough to separate the orders cleanly. Every move then takes
  `3 * pole_pairs / f` seconds, independent of the axis.
- **Poses: the same stroke range at different loads.** The load varies a lot along a stroke, but within one
  pose it is tied to the axis position. A second pose over the same range at a different load separates load
  effects from position effects. Loads are from the URDF gravity model. On axis 2, one more pose crosses the torque
  sign change within the stroke.
- **Output position map**: the strokes above cover only part of each joint's range (axis 2: 36°), not enough
  for effects tied to the output angle, e.g. the intermediate shaft of axis 2 (6 turns per output turn) or a
  spot like the vibration of axis 3 around q3 = 0. Every config ends with a `range` sweep over the whole joint
  range minus 0.1 rad (~6°) at one speed, f20 = 12 Hz (above the resonance, the rotor ripple stays small
  and sits at a far higher order). Axes 2 and 3 get two poses with different loads at the same position. On axis
  3 they shift the torque sign change from q3 ≈ 0 to q3 ≈ −0.87: an effect at a fixed q3 is position, one that
  moves with the sign change is load (belt slack, backlash). The upper end of axis 3 stops at +2.05, above about
  +2.15 the forearm comes within 2 cm of the upper arm.
- **No power cycle repetition**: the old logs `docs/tuning/slow_loaded*.csv` (3 boots) show the same order 20
  amplitude within a few percent. With the rotor angle now recorded, the phase of runs from different
  sessions can be compared directly.
- **Collisions**: every approach move (joints unsynchronized at `approach_speed`) and every stroke was checked
  against the URDF collision meshes with a 2 cm margin, against each other and the ground plane. Nothing else
  (table edges, cables, the surroundings) is checked. Watch out for: axis 1 `range` turns the arm almost a full
  circle (±169°), axis 2 `range_straight` starts with the straight arm pointing far back (q2 = −2.69).
- **Checked in the MuJoCo sim**: all configs run through without timeouts, segments and stamps are consistent.
  The sim says nothing about ripple or friction.

## Before the first run

1. `hw_diagnostics: true` for `moteus_hardware_system` in `robotarm_bringup/config/controllers.yaml`, restart
   the bringup (it is read at hardware init).
2. Run [check.yaml](check.yaml), then check its output directory:
   - `run.json`: `status` complete, `hw_diagnostics` true, `files.mid.csv.missed` 0 or close to it.
   - `mid.csv` has the columns `axis2_motor_rotor_position`, `_pid_p`, `_pid_i`, `_pid_d`, `_torque_ff`.
     `rotor_position` is a sawtooth from 0 to 2pi, 3 teeth per stroke.
   - **Gains**: during the constant speed part of a move, the command is linear in time. A least squares fit
     `pid_p ~ a * axis2_position + b * t + c` gives `a = -kp`, and `pid_d ~ a * axis2_velocity + c` gives
     `a = -kd`. Expected for axis 2, in rad: kp = 10000 / 2pi ≈ 1590 Nm/rad, kd = 900 / 2pi ≈ 143 Nm/(rad/s).
     A clearly different value means the drive does not run the gains of the config dump.
   - At the 4 Hz speed the ripple should be clearly visible in `axis2_effort` and `pid_p`.
