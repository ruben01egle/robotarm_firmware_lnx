# X4108 wrist motor testing (actuators 5 & 6): results

Record of the torque and thermal tests on the original X4108 wrist motors. The thermal result led to replacing them with the T-Motor MN5008 KV170 (see [wrist_motor_selection.md](wrist_motor_selection.md)). Tests on the new motors are in [mn5008_testing.md](mn5008_testing.md).

## Context

| Item | Value |
|---|---|
| Wrist | Belt-driven differential, 2 motors share pitch load |
| Motor | X4108 380KV outrunner, 89 g, 24N22P, stator 41x8 mm |
| Driver | mjbots moteus r4.11 (cooled) |
| Planned gear ratio | 1:5.5 |
| Actuator 5 | `motor.resistance_ohm 0.1018`, `motor.Kv 392.3` → Kt ≈ 8.27/Kv = 0.0211 Nm/A |
| Actuator 6 | `motor.resistance_ohm 0.1167`, `motor.Kv 348.4` → Kt ≈ 0.0237 Nm/A |
| Motor temp protection | Disabled (`servo.enable_motor_temperature 0`, no thermistor) |
| Expected holding current | ~11 A per motor (1 kg at full reach, static) → ~19–22 W copper loss |

## Torque test

Lever 50 mm on the motor shaft, pressing on a scale. τ_meas = m · 9.81 · 0.05.

| Motor | Commanded [Nm] | Scale [kg] | τ_meas [Nm] | q_A [A] | τ_meas / q_A [Nm/A] |
|---|---|---|---|---|---|
| 5 | 0.1 | 0.24 | 0.118 | 5 | 0.024 |
| 5 | 0.2 | 0.48 | 0.235 | 9 | 0.026 |
| 5 | 0.5 | 1.10 | 0.540 | 24 | 0.023 |
| 5 | 0.6 | 1.17 | 0.574 | 29 | 0.020 |
| 6 | 0.1 | 0.32 | 0.157 | – | – |
| 6 | 0.2 | 0.50 | 0.245 | 8 | 0.031 |
| 6 | 0.5 | 1.05 | 0.515 | 21 | 0.025 |
| 6 | 0.6 | 1.15 | 0.564 | 25 | 0.023 |
| 6 | 0.75 | 1.25 | 0.613 | 32 | 0.019 |

Findings:
- The slope between the middle points (≈ 0.020–0.021 Nm/A) agrees with Kt from Kv.
- τ_meas sits ~0.03–0.05 Nm above the commanded torque at low load. This is most likely a tare offset (lever weight).
- Torque saturates above ~25 A (`motor.rotation_current_cutoff_A 10000`, saturation model off).
- The holding point of ~0.25 Nm per motor needs ~10–12 A.
- The configs had `rotor_to_output_ratio 0.25`, but the measured currents only fit with a ratio of 1. The active ratio during the test is unclear.
- The two motors differ by ~12 % in Kv, so the same current gives noticeably different torque on each side of the differential.

## Thermal test

Method: pure d-current heating, winding resistance from a 5 A → 15 A step, as described in [mn5008_testing.md](mn5008_testing.md#method). Mount temperature was read with a thermal camera.

### Cold reference

| Motor | Date | T_cold [°C] | d_A1 [A] | d_V1 [V] | d_A2 [A] | d_V2 [V] | R_cold [Ω] | Config R [Ω] | Screenshot |
|---|---|---|---|---|---|---|---|---|---|
| 5 | 2026-10-16 | 23 | 5.0 | 0.63 | 15.0 | 1.67 | 0.104 | 0.1018 | `x4108_thermal/motor5_cold_ref.png` |

### Run 1: motor 5, 8 A, no cooling (2026-10-16)

Screenshots are in `x4108_thermal/run1/`.

| time [min] | d_A1 [A] | d_V1 [V] | d_A2 [A] | d_V2 [V] | R [Ω] | T_winding [°C] | mount [°C] | notes |
|---|---|---|---|---|---|---|---|---|
| 0 | 5.0 | 0.635 | 15.0 | 1.68 | 0.1045 | 24 | 25 | start; motor effectively cold |
| 1 | 5.0 | 0.70 | 15.1 | 1.84 | 0.1125 | 44 | 26 | |
| 2 | 5.0 | 0.73 | 15.0 | 1.91 | 0.118 | 58 | 28 | |
| 3 | 5.0 | 0.75 | 15.0 | 1.99 | 0.124 | 72 | 42 | still rising ~14 K/min |
| 4 | 5.0 | 0.785 | 15.0 | 2.05 | 0.1265 | 79 | 58 | 8 A level d_V suggests ~83 °C |
| 5 | 5.0 | 0.80 | 15.0 | 2.13 | 0.133 | ~90 (±5) | 63 | stopped: winding > 80 °C, mount > 60 °C |

Cooldown: about 5 min after stop, the thermal camera hotspot on the motor was 60 °C.

An earlier 11 A attempt was aborted after ~45 s: the winding had reached ~45 °C, with a 60 °C hotspot on the camera.

## Conclusion

- **No steady state at 8 A (~10 W) without cooling.** The winding rose ~11–14 K/min, and the limits were reached after ~4–5 min.
- **The PETG mount is the stator's main heat path.** The mount followed the winding closely (58 °C at 79 °C winding).
- A rough single-body fit (dT/dt = a − ΔT/τ) gives a ≈ 22 K/min and τ ≈ 5–7 min. That puts the continuous capacity without cooling at ~5 A ≈ 4 W ≈ 0.1 Nm per motor (±30 %).
- Holding needs ~0.25 Nm per motor ≈ 11 A ≈ 20 W, about 5× the capacity. **The X4108 is not viable for the wrist.** It was replaced by the MN5008 KV170.
