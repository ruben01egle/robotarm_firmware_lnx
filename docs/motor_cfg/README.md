# moteus Motor Configuration

Setup guide and reference for the six moteus r4.11 controllers of the arm, all with
AS5048B output encoders:

- **Axis 1–3**: Eaglepower LA8308, one- or two-stage GT3 belt gearboxes.
- **Axis 4**: Eaglepower LA8308, 1:4 gearbox.
- **Axis 5/6 (wrist)**: two T-Motor MN5008 KV170 motors, each with a 1:6 belt in front of
  a differential. The two controllers drive the two differential inputs, not the joints.
  They replaced the original BE4108 (X4108) 380KV motors, which overheated; see
  [wrist_motor_selection.md](wrist_motor_selection.md).

The `actuator<id>_moteus_r411_<motor>.cfg` files in this folder are full register dumps
of each controller (`conf enumerate` / `--dump-config` output). They are the **source of
truth for what is currently flashed**. Most of their ~2100 lines are either firmware
defaults or values written by the calibration; this document only covers the values
that were set up specifically for this robot. The files are named per actuator (CAN ID),
not per joint, because controllers 5 and 6 each drive both wrist joints.

- [Units](#units)
- [Setup procedure](#setup-procedure)
- [Overview](#overview)
- [Known differences to the target values](#known-differences-to-the-target-values)
- [Background: why each value is set](#background-why-each-value-is-set)

## Units

`motor_position.rotor_to_output_ratio` is set on every axis, so **every position,
velocity, acceleration and gain below is at the joint output, after the gearbox**
(verified in the firmware: limits are compared against `position_.position` /
`position_.velocity`, which are output-frame values).

| Quantity | moteus unit | From URDF (rad) |
|---|---|---|
| position (`servopos.*`) | rev | `rad / 2π` |
| velocity (`servo.max_velocity`, `default_velocity_limit`, `max_velocity_derate`) | rev/s | `(rad/s) / 2π` |
| acceleration (`servo.default_accel_limit`) | rev/s² | `(rad/s²) / 2π` |
| `pid_position.kp` / `ki` / `kd` / `ilimit` | Nm/rev, Nm/(rev·s), Nm/(rev/s), Nm | – |

`moteus_interface` only converts rev ↔ rad (`× 2π`, see `MoteusInterface.cpp`), it does
not apply any sign or ratio. Direction and ratio live on the controller.

**Axis 5/6 exception:** their "output" is the differential input after the 1:6 belt,
not a joint. `DifferentialTransmission` maps between the two (all in rev):

```
a = j5 + j6        j5 = (a + b) / 2        τ_j5 = τ_a + τ_b
b = j5 − j6        j6 = (a − b) / 2        τ_j6 = τ_a − τ_b
```

with `a` = controller 5 (`wrist_a`), `b` = controller 6 (`wrist_b`). This assumes a
differential with 1:1 bevel gears. All axis 5/6 limits and gains below are in this
actuator space.

## Setup procedure

All steps use `moteus_tool` (`python3 -m pip install moteus`) and an fdcanusb. Inside
`--console`, settings are changed with `conf set <param> <value>` and are only persistent
after `conf write`.

1. **Flash firmware v1** (or newer). The configs were dumped with v1; in v1
   `servo.pid_dq.max_desired_rate` was renamed to `servo.max_current_desired_rate`.

2. **Set the CAN ID.** A fresh controller answers on ID 1. The new ID takes effect
   immediately, so reconnect with `--target <new id>` before writing:
   ```
   python3 -m moteus.moteus_tool --target 1 --console
   conf set id.id 2
   ```
   Then `--target 2`, `conf write`. IDs must match the `can_id` params in
   `robotarm.ros2_control.xacro` (1–4 for axis 1–4, 5 for `wrist_a`, 6 for `wrist_b`).

3. **Calibrate the motor** with the motor **free to spin and without belt** (per the
   moteus calibration guide, the rotor turns in both directions at speed and must not
   see any load):
   ```
   python3 -m moteus.moteus_tool --target <id> --calibrate --cal-bw-hz 400
   ```
   This writes all `motor.*` values, the onboard encoder compensation table and the
   current loop gains (`servo.pid_dq_*`). Recalibrate after a motor/controller swap or
   when the controller is remounted on the motor.

4. **Configure the output encoder (aux2, I2C)** – see [Encoders](#encoders):
   ```
   conf set aux2.pins.0.mode 13
   conf set aux2.pins.1.mode 13
   conf set aux2.i2c.i2c_hz 400000
   conf set aux2.i2c.i2c_mode 1
   conf set aux2.i2c.pullup 0
   conf set aux2.i2c.devices.0.type 1
   conf set aux2.i2c.devices.0.address 64
   conf set aux2.i2c.devices.0.poll_rate_us 1000
   conf set motor_position.sources.1.aux_number 2
   conf set motor_position.sources.1.type 7
   conf set motor_position.sources.1.i2c_device 0
   conf set motor_position.sources.1.cpr 65536
   conf set motor_position.sources.1.reference 1
   conf set motor_position.sources.1.pll_filter_hz 10
   conf set motor_position.sources.1.offset <counts>
   ```
   (Axis 1 currently uses I2C device slot 1 instead of 0 – same function, see below.)
   On axis 5/6 the encoder on controller 5 measures joint 5 and the one on controller 6
   measures joint 6 (`joint1/2_encoder_actuator` in the xacro), not the controller's
   own output. That is fine because moteus never uses it, see [Encoders](#encoders).

5. **Gearbox and direction** (per axis, see [Overview](#overview)):
   ```
   conf set motor_position.rotor_to_output_ratio <ratio>
   conf set motor_position.output.sign <±1>
   ```

6. **Shared limits and protection**: `servo.max_voltage`, flux braking,
   `servo.pwm_rate_hz`, `servo.max_current_desired_rate`, `servo.max_velocity_derate`.

7. **Per-axis limits and PID**: `servopos.position_min/max`, `servo.max_velocity`,
   `servo.default_velocity_limit`, `servo.default_accel_limit`,
   `servo.pid_position.kp/ki/kd/ilimit`. Derive the limits from the URDF
   ([formulas](#limits-come-from-the-urdf)).

8. **`conf write`**, then dump a backup into this folder and commit it:
   ```
   python3 -m moteus.moteus_tool --target <id> --dump-config > actuator<id>_moteus_r411_<motor>.cfg
   ```
   with `<motor>` = `eaglepower_8308` (actuator 1–4) or `mn5008` (actuator 5/6).
   A dump can be put back on a controller with `--restore-config <file>`. The
   calibration values in it belong to that specific motor/controller pair.

## Overview

### Shared by all axes

Shared by all six controllers unless a row says otherwise; for axis 5/6 differences, see
[Axis 5/6](#axis-56-wrist-mn5008-kv170).

| Parameter | Value | What it does |
|---|---|---|
| `servo.max_voltage` | 35 V | Bus voltage above which the controller faults and drops all torque |
| `servo.flux_brake_margin_voltage` | 8 V | Flux braking starts at `max_voltage − margin` = **27 V** |
| `servo.flux_brake_resistance_ohm` | 0.06 Ω | Braking gain: d-axis current = overvoltage / 0.06 Ω (16.7 A/V) |
| `servo.max_current_A` | 100 A (axis 1–4 only) | Phase current limit (board default), hard backstop for torque |
| `servo.fault_temperature` | 75 °C | Board temperature fault (default) |
| `servo.pwm_rate_hz` | 20000 | PWM / current loop rate (default 30000) |
| `servo.max_current_desired_rate` | 50000 A/s | Max slew of the desired current (default 10000) |
| `servo.max_velocity_derate` | 0.5 rev/s | Motoring current falls to 0 at `max_velocity + derate` |
| `servo.default_timeout_s` / `timeout_mode` | 0.1 s / 12 | Command watchdog: hold zero velocity after 100 ms without commands (defaults) |
| `motor_position.commutation_source` | 0 | Onboard SPI encoder (source 0) commutates the motor |
| `motor_position.output.source` | 0 | Output position/velocity = onboard encoder × gear ratio |
| `motor_position.output.reference_source` | −1 | moteus does **not** resolve the output position from the AS5048B |
| `motor_position.sources.1.*` | I2C, cpr 65536, reference 1, PLL 10 Hz | AS5048B on the joint output, read by `moteus_interface` at startup |
| `aux2.pins.0/1.mode` | 13 | aux2 pins 0/1 are I2C SCL/SDA |
| `aux2.i2c.*` | 400 kHz, mode 1, pullup 0 | Fast-mode I2C, internal pull-ups off |
| `aux2.i2c.devices.N` | type 1, address 64, poll 1000 µs | AS5048B at 0x40, polled at 1 kHz |
| calibration | `--cal-bw-hz 400` → `servo.pid_dq_hz` 400 | Current loop bandwidth |

### Per axis (axis 1–4, LA8308)

| Parameter | Axis 1 (base) | Axis 2 (shoulder) | Axis 3 (elbow) | Axis 4 (forearm roll) |
|---|---|---|---|---|
| `id.id` | 1 | 2 | 3 | 4 |
| gearbox | 1:6, one belt stage | 1:30 (5 × 6), two stages | 126:11 ≈ 1:11.45, two stages | 1:4 |
| `motor_position.rotor_to_output_ratio` | 0.166667 | 0.033333 | 0.087302 | 0.25 |
| `motor_position.output.sign` | −1 | 1 | 1 | 1 |
| `motor_position.sources.1.i2c_device` | 1 | 0 | 0 | 0 |
| `servopos.position_min` [rev] | −0.486111 | −0.444444 | −0.333333 | −0.472222 |
| `servopos.position_max` [rev] | 0.486111 | 0.055556 | 0.369444 | 0.472222 |
| `servo.default_velocity_limit` [rev/s] | 0.6 | 0.4775 | 0.4775 | 0.4775 |
| `servo.max_velocity` [rev/s] | 0.6 | 0.4775 | 0.4775 | 0.4775 |
| `servo.default_accel_limit` [rev/s²] | 12 | 4 | 20 | 12 |
| `servo.pid_position.kp` [Nm/rev] | 5000 | 12000 | 6000 | 1000 |
| `servo.pid_position.ki` [Nm/(rev·s)] | 5000 | 5000 | 5000 | 500 |
| `servo.pid_position.kd` [Nm/(rev/s)] | 10 | 40 | 10 | 5 |
| `servo.pid_position.ilimit` [Nm] | 7.2 | 36 | 14.5 | 6 |

All values in this table match the axis 1–4 dumps. The axis 1 velocity limits are
1.2 × URDF = 0.5997 rev/s, rounded to 0.6. Axis 4 uses the same motor and shared
settings as axis 1–3. Its `ilimit` is 1.2 × the URDF effort of 5 Nm. Its calibrated
`motor.Kv` is 131, about 9 % below axis 1–3 (142–144). That is within normal motor
spread, as long as the motor was free during calibration.

### Axis 5/6 (wrist, MN5008 KV170)

Both controllers use the same values. The MN5008 is a much smaller motor than the
LA8308, so several shared values differ. **⚠️ = differs from axis 1–4.** The values
are as flashed in the `actuator5/6_moteus_r411_mn5008.cfg` dumps.

Motor choice and thermal tests: [wrist_motor_selection.md](wrist_motor_selection.md),
[mn5008_testing.md](mn5008_testing.md). Results for the old BE4108 motors are in
[x4108_testing.md](x4108_testing.md); their dumps are in the git history.

Calibration results:

| | Actuator 5 | Actuator 6 |
|---|---|---|
| `motor.poles` | 28 | 28 |
| `motor.phase_invert` | 1 | 1 |
| `motor.resistance_ohm` | 0.1506 | 0.1442 |
| `motor.Kv` | 156.4 | 155.5 |
| Kt at the motor (≈ 8.27 / Kv) | ≈ 0.053 Nm/A | ≈ 0.053 Nm/A |
| Kt after 1:6 | ≈ 0.317 Nm/A | ≈ 0.319 Nm/A |

`phase_invert` only reflects the phase wiring order. Both Kv values are ~8 % below the
nominal 170 and within 0.5 % of each other. The 4 % difference in resistance comes from
calibration temperature: measured cold, both are 0.146 Ω.

| Parameter | Axis 1–4 | Actuator 5 (`wrist_a`) / 6 (`wrist_b`) | Why |
|---|---|---|---|
| `id.id` | 1–4 | 5 / 6 | `can_id` in the xacro |
| gearbox | – | 1:6 belt, then differential | |
| ⚠️ `motor_position.rotor_to_output_ratio` | – | **0.166667** | Output = differential input after the 1:6 belt (1:1 bevel gears assumed) |
| ⚠️ `motor_position.output.sign` | – | **1 / 1 (verify on test, dump has −1 / 1)** | Both controllers +: j5 moves +, j6 stays. 5 + / 6 −: j6 moves + |
| `motor_position.output.reference_source` | −1 | −1 (**must stay**) | The AS5048B measures a joint, not this controller's output |
| ⚠️ `servo.max_current_A` | 100 A | **25 A** | ≈ 7.9 Nm per actuator after 1:6. Peak only, see cooling below |
| ⚠️ `servopos.position_min` [rev] | – | **−0.802778** | −(114° + 175°) / 360° = −0.802778, see [Limits](#limits-come-from-the-urdf) |
| ⚠️ `servopos.position_max` [rev] | – | **0.8055** | +(115° + 175°) / 360° = 0.805556, rounded 0.02° below the URDF |
| ⚠️ `servo.default_velocity_limit` [rev/s] | – | **0.955** | 1.2 · (2.5 + 2.5 rad/s) / 2π = 0.9549 |
| ⚠️ `servo.max_velocity` [rev/s] | – | **0.955** | as above |
| `servo.max_velocity_derate` [rev/s] | 0.5 | 0.5 | unchanged |
| ⚠️ `servo.default_accel_limit` [rev/s²] | 4–12 | **20** | Loose upper bound, wrist inertia is small |
| ⚠️ `servo.pid_position.kp` [Nm/rev] | 1000–12000 | **1000** | 0.001 rev error → 1 Nm |
| ⚠️ `servo.pid_position.ki` [Nm/(rev·s)] | 500–5000 | **500** | |
| ⚠️ `servo.pid_position.kd` [Nm/(rev/s)] | 5–40 | **2** | |
| ⚠️ `servo.pid_position.ilimit` [Nm] | 7.2–36 | **1.0** | Wrist gravity load is small. Raise towards ~1.5 Nm per actuator if the wrist sags under payload (2.5 Nm wrist pitch = 1.25 Nm per actuator) |
| calibration `--cal-bw-hz` | 400 | 400 → `servo.pid_dq_hz` 400 | Drop to default 200 if the current loop is noisy |
| `servo.flux_brake_*` | 8 V / 0.06 Ω | unchanged | |
| `servo.pwm_rate_hz`, `max_current_desired_rate`, `max_voltage`, timeout, encoder (aux2) settings | | unchanged | |

Points to check:

- **Torque.** At 25 A each actuator gives ≈ 7.9 Nm after 1:6. Each actuator carries
  `(τ5 ± τ6) / 2`, so the URDF effort of 4 Nm on one joint needs ≈ 2 Nm (≈ 6.3 A) per
  motor, and 4 Nm on both joints at once needs ≈ 4 Nm (≈ 12.6 A) on one motor. Both
  are reachable well within 25 A.
- **Cooling.** Each motor needs its 35 mm fan for continuous load. With fans, the tests
  give ≈ 9–10 A continuous per motor (≈ 5 Nm wrist pitch); without fans only
  ≈ 4–4.5 A, with the PETG mount reaching 60 °C (see
  [mn5008_testing.md](mn5008_testing.md)). 25 A is a peak (≈ 1.5 · I² · R ≈ 140 W per
  motor). There is no motor thermistor, so nothing stops a stalled wrist without fans
  from overheating the PETG mounts.
- **Stiffness.** A joint 5 error moves both actuators, so the joint stiffness is
  2 × `kp`. A joint 6 error also gives 2 × `kp`.
- **Bevel ratio.** If the differential's bevel gears are not 1:1, j6 and its limits
  scale with that ratio. The `ratio` param in the xacro is not used by
  `DifferentialTransmission`.

## Known differences to the target values

All six dumps were updated on 2026-10-24. Limits, velocity settings, the secondary
encoders, the axis 4 direction (checked) and the axis 4–6 PID tuning are done. What is still open is
listed below. Remove this section once everything is done.

### TODO

- [ ] Actuator 5 and 6 (`--target 5` / `--target 6 --console`, each):
  - [ ] Verify both `output.sign` with the differential test in the
    [axis 5/6 table](#axis-56-wrist-mn5008-kv170). Actuator 5 has −1, the plan
    assumes 1.
  - [ ] `conf write`
- [ ] Test each axis: move to both URDF position limits and at URDF velocity (joint
  stream controller is capped at 2.5 rad/s for all joints) and check that the joint
  tracks without hitting the moteus limits. For axis 5/6 also move both joints at full
  speed at once, and check motor temperature.
- [ ] Re-dump actuator 5/6 after any changes above (step 8 of the
  [setup procedure](#setup-procedure)) and commit.
- [ ] Document why axis 3 uses `ilimit 14.5` (see [Position PID](#position-pid)).

### Differences

| Parameter | Axis | In dump | Target | Reason |
|---|---|---|---|---|
| `motor_position.output.sign` | 5 | −1 | 1 (verify on test) | see [axis 5/6](#axis-56-wrist-mn5008-kv170) |

## Background: why each value is set

### Encoders

Each joint has two encoders:

- **Source 0 – onboard AS5047 (SPI, rotor side).** Used for commutation and, through
  `rotor_to_output_ratio`, as the output position for the position loop. Through the
  gearbox its resolution at the joint is multiplied by the ratio, which gives a much
  finer position loop than the output encoder could.
- **Source 1 – AS5048B (I2C, joint output).** An absolute magnetic encoder on the
  output shaft of each joint. Its only job is to tell where the joint is after power-on:
  the onboard encoder is absolute over one rotor turn only, and with a 6–30× reduction
  that is not enough to know the joint angle.

moteus can do this startup disambiguation itself (`output.reference_source = 1`), but
then the offset between the two encoders is stored in the controller. Whenever a belt
skips a tooth, that offset changes and the controller config would have to be
rewritten. Therefore `reference_source` is left at −1: moteus only reads the AS5048B,
and `moteus_interface` fetches the raw value (`kEncoder1Position`) on every
`on_configure()`, applies the joint-space trim from the URDF (`encoder_offset`) and sets
the output position with `MakeOutputExact`. A slipped belt is fixed by re-configuring
the hardware component (and, if needed, adjusting `encoder_offset`), without touching
the controller. Details: [moteus_interface README](../../src/moteus_interface/README.md#non-standard-secondary-encoder-homing).

Settings:

- `aux2.pins.0/1.mode 13`: the encoder is wired to aux2 pins 0/1 as I2C. aux2 SPI stays
  disabled (`aux2.spi.mode 1`).
- `i2c_hz 400000`, `i2c_mode 1`: fast-mode I2C, the maximum moteus supports.
- `pullup 0`: the internal 2.2 kΩ pull-ups are off; the bus uses the pull-ups on the
  encoder breakout.
- `devices.N.type 1` (AS5048), `address 64` (0x40, AS5048B default),
  `poll_rate_us 1000`: read at 1 kHz, matching the control loop rate.
- `sources.1.type 7` (I2C), `cpr 65536`, `reference 1`: 16-bit counts, mounted on the
  output (not the rotor).
- `sources.1.offset`: integer counts, applied as `(raw + offset) · sign / cpr`. Set so
  the encoder reads close to 0 at the mechanical zero of the joint; the remaining trim is
  done in software via `encoder_offset`.
- `sources.1.pll_filter_hz 10`: strong low-pass filter. The value is only used as a
  static position at startup, so latency does not matter but noise does.
- `sources.1.sign 1`: moteus requires all sources to count in the same direction.
  Check by turning the joint by hand and comparing `motor_position.sources.0/1` in
  `tview`.
- On axis 5/6 the AS5048B wired to each controller measures a joint (5 or 6), not that
  controller's differential input. moteus must therefore never use it
  (`reference_source` stays −1); `DifferentialTransmission::home()` combines both
  readings into actuator positions.
- Axis 1 has the AS5048B in I2C device slot 1 and axes 2/3 in slot 0. Both work the same,
  because `sources.1.i2c_device` points at the slot in use.

### Gearbox and direction

`rotor_to_output_ratio` is output turns per rotor turn:

- Axis 1: one belt stage 1:6 → 0.166667. The base has no gravity load, so a low ratio
  keeps it fast.
- Axis 2: two stages 1:5 · 1:6 = 1:30 → 0.033333. The shoulder carries the whole arm
  and needs the highest holding torque.
- Axis 3: designed as 1:12, but the belt length and shaft distance did not fit during
  assembly, so the output pulley was made smaller. The configured value is 0.087302
  (126:11 ≈ 1:11.45); the report quotes ≈ 11.7, the config value is the one in use.
- Axis 4: 1:4 → 0.25.
- Axis 5/6: 1:6 belt per motor in front of the differential → 0.166667 (raised from the
  original 1:4 / 1:5.5 for the MN5008, see
  [wrist_motor_selection.md](wrist_motor_selection.md)). The differential itself
  is not configured on the controller; `moteus_interface` handles it (see
  [Units](#units)).

`motor_position.output.sign` makes the positive moteus direction match the positive
joint direction in the URDF (of axis 1–3 only axis 1 needs −1). On axis 5/6 the signs
must match the differential equations: both controllers positive → j5 positive, j6
still; controller 5 positive and 6 negative → j6 positive.

### Limits come from the URDF

All position and velocity limits are derived from the joint `<limit>` tags in
`robotarm_description/urdf/robotarm_geometry.xacro`. After changing the URDF, recompute
them:

- `servopos.position_min/max = lower/upper / 2π` (no headroom: these are hard stops in the
  controller, and the planners already stay within the URDF).
- `servo.default_velocity_limit = servo.max_velocity = 1.2 · velocity / 2π`.

Axis 5/6 (differential): each controller sees `a = j5 + j6` or `b = j5 − j6`, so the
limits are the extremes of that sum:

- `servopos.position_min = (j5 lower + j6 lower) / 2π` = −(114° + 175°) / 360°
  = −0.802778 rev, `servopos.position_max = (j5 upper + j6 upper) / 2π`
  = (115° + 175°) / 360° = 0.805556 rev. The j6 range is symmetric, so `b = j5 − j6` has
  the same extremes as `a`.
- `servo.default_velocity_limit = servo.max_velocity = 1.2 · (v5 + v6) / 2π`
  = 1.2 · 5 / 2π = 0.9549 rev/s.

These limits are a diamond around the joint-space box. A controller cannot stop j5 at
115° while j6 is at 0, so on axis 5/6 the moteus limits only catch gross errors. The real
joint limits are enforced only by the planners/URDF.

### Velocity limits

The two velocity parameters do different things:

- **`default_velocity_limit` / `default_accel_limit`** limit moteus' internal trajectory
  in position mode. `moteus_interface` sends position + velocity every cycle and no
  per-command limits, so these defaults apply. moteus moves its internal set point
  toward the commanded position (which itself moves at the commanded velocity), and
  speeds up to catch up if it lags, but never faster than `velocity_limit`. Setting it
  20 % above the URDF limit lets the controller follow a command at the URDF limit and
  still have headroom to correct tracking error. For the acceleration limit see
  [Acceleration limits](#acceleration-limits).
- **`max_velocity` / `max_velocity_derate`** are not a speed limit but a current
  limit: above `max_velocity` the allowed motoring current falls linearly and reaches 0
  at `max_velocity + max_velocity_derate`. Braking current is never limited, so the axis
  can always slow down. It is a safety net against runaway, e.g. after a bad command.
  `default_velocity_limit` is clamped to `max_velocity`, so `max_velocity` must not be
  lower; both are set to 1.2 × URDF. With `derate = 0.5 rev/s`, motoring current is 0 at
  about 1.1 rev/s (axis 1), 0.98 rev/s (axis 2–4) and 1.45 rev/s (axis 5/6 actuators).

### Acceleration limits

`servo.default_accel_limit` (12 / 4 / 20 rev/s² ≈ 75 / 25 / 126 rad/s²) is deliberately
loose: an upper bound, not a tuned value. The URDF has no acceleration limits, and the
ROS controllers already command smooth trajectories (joint stream controller: 8 rad/s², impedance
variant: 4 rad/s²), so in normal
operation this limit never acts. It only matters when moteus' internal set point has to
jump or catch up, e.g. after a large position step.

Rough rigid-body estimate with the inertias from the report (outstretched arm, 0.5 kg
payload, no belt losses or motor inertia):

| Axis | Limit | Inertia | Torque needed | Available | Reachable |
|---|---|---|---|---|---|
| 1 | 75 rad/s² | ≈ 0.75 kg·m² | ≈ 57 Nm | 6 Nm design / 8 Nm URDF | no, ≈ 8–10 rad/s² |
| 2 | 25 rad/s² | ≈ 0.73 kg·m² | ≈ 18 Nm + 18.4 Nm gravity | 30 Nm design / 35 Nm URDF | about at the limit |
| 3 | 126 rad/s² | ≈ 0.09 kg·m² | ≈ 11.3 Nm + ~5.5 Nm gravity | 9 Nm design / 15 Nm URDF | no, ≈ 40–100 rad/s² |

Axis 4 (12 rev/s²) and axis 5/6 (20 rev/s² per actuator) are not estimated yet; the
report has no inertias for them.

On axis 1–3 the motor saturates before the limit is reached. If the set point jumps,
these axes therefore move at full torque and the integral term winds up to `ilimit`,
which can cause some overshoot instead of a clean ramp. This is accepted. Axis 3 was
raised from 6 to 20 rev/s², so like axis 1 and 2 its limit no longer shapes the motion.

### Voltage and flux braking

The arm runs on a 24 V supply; the moteus r4.11 is rated up to 44 V.

- `servo.max_voltage 35`: above 35 V the controller faults. This is well below the board
  limit and also protects the power supply from regenerated energy.
- When an axis brakes, the motor works as a generator and pushes the bus voltage up.
  Above `max_voltage − flux_brake_margin_voltage` = 27 V, the controller adds negative
  d-axis current in proportion to the overvoltage:

  ```
  i_d = −(V_bus − 27 V) / flux_brake_resistance_ohm
  ```

  d-axis current makes no torque; it turns the excess energy into heat in the motor
  windings. To the bus, the motor looks like a resistor of
  `flux_brake_resistance_ohm`, which is a gain, not a physical value (0.06 Ω happens to be
  close to the calibrated phase resistance of ~0.05 Ω, but the firmware does not use one
  for the other).
- Margin 8 V instead of the default 3 V: braking starts 3 V above the nominal supply
  voltage, so supply ripple does not trigger it, while there is still 8 V of room before
  the fault. 0.06 Ω (16.7 A per volt) is gentler than the default 0.025 Ω; the braking
  current reaches `max_current_A` (100 A) at about 33 V, still below the fault.
- The report's worst-case estimate (≈30 J stored energy in a full-speed stop) is small
  compared to the thermal capacity of the windings, so no external brake resistor is
  used.

### Current loop and PWM

- `--cal-bw-hz 400`: current loop bandwidth 400 Hz instead of the default 200 Hz, halving
  the torque response time to ~2.5 ms for a stiffer position loop. Downside: more audible
  noise.
- `servo.max_current_desired_rate 50000` A/s: the default 10000 A/s slew limit would cap
  how fast the 400 Hz current loop can change its set point.
- `servo.pwm_rate_hz 20000`: lowered from 30 kHz because the 1 kHz `ros2_control` loop
  over the fdcanusb was not stable at 30 kHz (commit `cd8fa64`). Lower PWM also increases
  efficiency, at the cost of peak power and bandwidth.
- `servo.max_current_A 100`: board default, left as the only hard torque backstop.

### Position PID

Gains are in output units: with `kp = 5000`, an error of 0.001 rev (0.36°) on axis 1
produces 5 Nm. They are three to four orders of magnitude above the moteus defaults
(`kp 4, ki 1, kd 0.05`) because the output side sees the full gear ratio and the arm is
meant to hold position stiffly.

Tuning procedure (per axis, with `tview`, report §7.0.4):

1. `kd`, `ki`, `ilimit` = 0, start `kp` at 0.001. Increase `kp` until the joint is stiff
   enough; if it oscillates, back off.
2. Start `kd` at 1 % of `kp`. Deflect the joint by hand and let go; increase `kd` until
   it returns without overshoot.
3. Set `ilimit` to about 20 % above the maximum expected static load torque, then raise
   `ki` starting at the level of `kp`.

`ilimit` = 1.2 × design torque from the drive sizing (report table 2.2): 1.2 · 6 Nm =
7.2 Nm (axis 1), 1.2 · 30 Nm = 36 Nm (axis 2). Axis 3 uses 14.5 Nm, which does not
match its 9 Nm design torque (1.2 · 9 = 10.8 Nm); the reason is not documented.
Axis 4 uses 1.2 × its URDF effort (1.2 · 5 Nm = 6 Nm). Axis 5/6 use 1.0 Nm per actuator;
it may be raised if the wrist sags under payload (see the
[axis 5/6 section](#axis-56-wrist-mn5008-kv170)).
`ilimit` only bounds the integral term, it is not a torque limit.

History (from the config commits):

| Date | Axis 1 kp / kd | Axis 2 kp / kd | Axis 3 kp / kd |
|---|---|---|---|
| 2026-05-29, first setup | 1000 / 5 | 1000 / 15 | 1000 / 15 |
| 2026-06-02, tuning | 3000 / 5 | 12000 / 40 | 6000 / 10 |
| 2026-07-15, fw v1 | 5000 / 10 | 12000 / 40 | 6000 / 10 |

Axis 4–6 were tuned on 2026-10-24: axis 4 `kp 1000 / ki 500 / kd 5`, actuator 5/6
`kp 1000 / ki 500 / kd 2`.

Axis 2 is the stiffest because it holds the whole arm against gravity. The single-axis
tuning is only a starting point; final tuning has to be done on the assembled arm.
