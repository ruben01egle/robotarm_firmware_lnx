# moteus Motor Configuration

Setup guide and reference for the three moteus r4.11 controllers driving axis 1–3
(Eaglepower LA8308 motors, two-stage GT3 belt gearboxes, AS5048B output encoders).

The `axis*_moteus_r411_eaglepower_8308.cfg` files in this folder are full register dumps
of each controller (`conf enumerate` / `--dump-config` output). They are the **source of
truth for what is currently flashed**. Most of their ~2100 lines are either firmware
defaults or values written by the calibration; this document only covers the values
that were set up specifically for this robot.

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
   `robotarm.ros2_control.xacro` (1, 2, 3).

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
   python3 -m moteus.moteus_tool --target <id> --dump-config > axis<id>_moteus_r411_eaglepower_8308.cfg
   ```
   A dump can be put back on a controller with `--restore-config <file>`. The
   calibration values in it belong to that specific motor/controller pair.

## Overview

### Shared by all axes

| Parameter | Value | What it does |
|---|---|---|
| `servo.max_voltage` | 35 V | Bus voltage above which the controller faults and drops all torque |
| `servo.flux_brake_margin_voltage` | 8 V | Flux braking starts at `max_voltage − margin` = **27 V** |
| `servo.flux_brake_resistance_ohm` | 0.06 Ω | Braking gain: d-axis current = overvoltage / 0.06 Ω (16.7 A/V) |
| `servo.max_current_A` | 100 A | Phase current limit (board default), hard backstop for torque |
| `servo.fault_temperature` | 75 °C | Board temperature fault (default) |
| `servo.pwm_rate_hz` | 20000 | PWM / current loop rate (default 30000) |
| `servo.max_current_desired_rate` | 50000 A/s | Max slew of the desired current (default 10000) |
| `servo.max_velocity_derate` | 0.5 rev/s (target, see below) | Motoring current falls to 0 at `max_velocity + derate` |
| `servo.default_timeout_s` / `timeout_mode` | 0.1 s / 12 | Command watchdog: hold zero velocity after 100 ms without commands (defaults) |
| `motor_position.commutation_source` | 0 | Onboard SPI encoder (source 0) commutates the motor |
| `motor_position.output.source` | 0 | Output position/velocity = onboard encoder × gear ratio |
| `motor_position.output.reference_source` | −1 | moteus does **not** resolve the output position from the AS5048B |
| `motor_position.sources.1.*` | I2C, cpr 65536, reference 1, PLL 10 Hz | AS5048B on the joint output, read by `moteus_interface` at startup |
| `aux2.pins.0/1.mode` | 13 | aux2 pins 0/1 are I2C SCL/SDA |
| `aux2.i2c.*` | 400 kHz, mode 1, pullup 0 | Fast-mode I2C, internal pull-ups off |
| `aux2.i2c.devices.N` | type 1, address 64, poll 1000 µs | AS5048B at 0x40, polled at 1 kHz |
| calibration | `--cal-bw-hz 400` → `servo.pid_dq_hz` 400 | Current loop bandwidth |

### Per axis

| Parameter | Axis 1 (base) | Axis 2 (shoulder) | Axis 3 (elbow) |
|---|---|---|---|
| `id.id` | 1 | 2 | 3 |
| gearbox | 1:6, one belt stage | 1:30 (5 × 6), two stages | 126:11 ≈ 1:11.45, two stages |
| `motor_position.rotor_to_output_ratio` | 0.166667 | 0.033333 | 0.087302 |
| `motor_position.output.sign` | −1 | 1 | 1 |
| `motor_position.sources.1.i2c_device` | 1 | 0 | 0 |
| `servopos.position_min` [rev] | −0.486111 | −0.444444 | −0.333333 |
| `servopos.position_max` [rev] | 0.486111 | 0.055556 | 0.375 |
| `servo.default_velocity_limit` [rev/s] | 0.5997 | 0.4775 | 0.4775 |
| `servo.max_velocity` [rev/s] | 0.5997 | 0.4775 | 0.4775 |
| `servo.default_accel_limit` [rev/s²] | 12 | 4 | 6 |
| `servo.pid_position.kp` [Nm/rev] | 5000 | 12000 | 6000 |
| `servo.pid_position.ki` [Nm/(rev·s)] | 5000 | 5000 | 5000 |
| `servo.pid_position.kd` [Nm/(rev/s)] | 10 | 40 | 10 |
| `servo.pid_position.ilimit` [Nm] | 7.2 | 36 | 14.5 |

Position and velocity limits in this table are the **target values derived from the
current URDF**; the dumps do not match them yet, see the next section.

## Known differences to the target values

The dumps in this folder (last updated 2026-07-15) still contain older limits. Remove
this section once everything below is done.

### TODO

- [ ] Axis 1 (`--target 1 --console`):
  - [ ] `conf set servopos.position_min -0.486111`
  - [ ] `conf set servopos.position_max 0.486111`
  - [ ] `conf set servo.default_velocity_limit 0.5997`
  - [ ] `conf set servo.max_velocity 0.5997`
  - [ ] `conf set servo.max_velocity_derate 0.5`
  - [ ] `conf write`
- [ ] Axis 2 (`--target 2 --console`):
  - [ ] `conf set servo.default_velocity_limit 0.4775`
  - [ ] `conf set servo.max_velocity 0.4775`
  - [ ] `conf set servo.max_velocity_derate 0.5`
  - [ ] `conf write`
- [ ] Axis 3 (`--target 3 --console`):
  - [ ] `conf set servopos.position_max 0.375`
  - [ ] `conf set servo.default_velocity_limit 0.4775`
  - [ ] `conf set servo.max_velocity 0.4775`
  - [ ] `conf set servo.max_velocity_derate 0.5`
  - [ ] Optional: raise `servo.default_accel_limit` above 6 rev/s² (see
    [Acceleration limits](#acceleration-limits))
  - [ ] `conf write`
- [ ] Test each axis: move to both URDF position limits and at URDF velocity (teleop is
  capped at 2.5 rad/s for all joints) and check that the joint tracks without hitting
  the moteus limits.
- [ ] Re-dump all three controllers into this folder (step 8 of the
  [setup procedure](#setup-procedure)) and commit.
- [ ] Document why axis 3 uses `ilimit 14.5` (see [Position PID](#position-pid)).
- [ ] Remove this section and the "target values" note in the per-axis table.

### Differences

| Parameter | Axis | In dump | Target | Reason |
|---|---|---|---|---|
| `servopos.position_min` | 1 | −0.42 | −0.486111 | URDF −175° |
| `servopos.position_max` | 1 | 0.42 | 0.486111 | URDF +175° |
| `servopos.position_max` | 3 | 0.361111 | 0.375 | URDF +135° |
| `servo.default_velocity_limit` | 1 / 2 / 3 | 1.5 / 3 / 3 | 0.5997 / 0.4775 / 0.4775 | 1.2 × URDF velocity |
| `servo.max_velocity` | 1 / 2 / 3 | 4 / 4 / 4 | 0.5997 / 0.4775 / 0.4775 | 1.2 × URDF velocity |
| `servo.max_velocity_derate` | all | 2 | 0.5 | see [Velocity limits](#velocity-limits) |

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

`motor_position.output.sign` makes the positive moteus direction match the positive
joint direction in the URDF (only axis 1 needs −1).

### Limits come from the URDF

All position and velocity limits are derived from the joint `<limit>` tags in
`robotarm_description/urdf/robotarm_geometry.xacro`. After changing the URDF, recompute
them:

- `servopos.position_min/max = lower/upper / 2π` (no headroom: these are hard stops in the
  controller, and the planners already stay within the URDF).
- `servo.default_velocity_limit = servo.max_velocity = 1.2 · velocity / 2π`.

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
  about 1.1 rev/s (axis 1) and 0.98 rev/s (axis 2/3).

### Acceleration limits

`servo.default_accel_limit` (12 / 4 / 6 rev/s² ≈ 75 / 25 / 38 rad/s²) is deliberately
loose: an upper bound, not a tuned value. The URDF has no acceleration limits, and the
ROS controllers already command smooth trajectories (teleop: 4 rad/s²), so in normal
operation this limit never acts. It only matters when moteus' internal set point has to
jump or catch up, e.g. after a large position step.

Rough rigid-body estimate with the inertias from the report (outstretched arm, 0.5 kg
payload, no belt losses or motor inertia):

| Axis | Limit | Inertia | Torque needed | Available | Reachable |
|---|---|---|---|---|---|
| 1 | 75 rad/s² | ≈ 0.75 kg·m² | ≈ 57 Nm | 6 Nm design / 8 Nm URDF | no, ≈ 8–10 rad/s² |
| 2 | 25 rad/s² | ≈ 0.73 kg·m² | ≈ 18 Nm + 18.4 Nm gravity | 30 Nm design / 35 Nm URDF | about at the limit |
| 3 | 38 rad/s² | ≈ 0.09 kg·m² | ≈ 3.3 Nm + ~5.5 Nm gravity | 9 Nm design / 15 Nm URDF | yes |

On axis 1 and 2 the motor saturates before the limit is reached. If the set point jumps,
these axes therefore move at full torque and the integral term winds up to `ilimit`,
which can cause some overshoot instead of a clean ramp. This is accepted. Axis 3 is the
only axis where the limit actually shapes the motion and may be raised.

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
`ilimit` only bounds the integral term, it is not a torque limit.

History (from the config commits):

| Date | Axis 1 kp / kd | Axis 2 kp / kd | Axis 3 kp / kd |
|---|---|---|---|
| 2026-05-29, first setup | 1000 / 5 | 1000 / 15 | 1000 / 15 |
| 2026-06-02, tuning | 3000 / 5 | 12000 / 40 | 6000 / 10 |
| 2026-07-15, fw v1 | 5000 / 10 | 12000 / 40 | 6000 / 10 |

Axis 2 is the stiffest because it holds the whole arm against gravity. The single-axis
tuning is only a starting point; final tuning has to be done on the assembled arm.
