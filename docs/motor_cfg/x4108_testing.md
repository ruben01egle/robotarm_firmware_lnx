# X4108 wrist motor testing (actuators 5 & 6)

## 1. Purpose & context

Find out whether the wrist motors can hold the expected continuous current without overheating, given that the PETG motor mounts are the thermal limit. Also record the torque-constant test.

| Item | Value |
|---|---|
| Wrist | Belt-driven differential, 2 motors share pitch load |
| Motor | X4108 380KV outrunner, 89 g, 24N22P, stator 41x8 mm |
| Driver | mjbots moteus r4.11 (cooled) |
| Planned gear ratio | 1:5.5 (config currently `motor_position.rotor_to_output_ratio 0.25` = 1:4) |
| Actuator 5 | `motor.resistance_ohm 0.1018`, `motor.Kv 392.3` → Kt ≈ 8.27/Kv = 0.0211 Nm/A |
| Actuator 6 | `motor.resistance_ohm 0.1167`, `motor.Kv 348.4` → Kt ≈ 0.0237 Nm/A |
| Limits | `servo.max_current_A 40`, `servo.fault_temperature 75` (board) |
| Motor temp protection | **Disabled** (`servo.enable_motor_temperature 0`, no thermistor) |
| Expected holding current | ~11 A per motor (1 kg at full reach, static) |
| Copper loss at 11 A | ≈ 1.5 · R · I² ≈ 18–21 W per motor |

## 2. Torque test (done)

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
- τ_meas sits ~0.03–0.05 Nm above the commanded torque at low load, which points to a tare offset (lever weight). Re-check one point with the scale zeroed while the lever rests on it.
- Torque saturates above ~25 A (`motor.rotation_current_cutoff_A 10000`, so the saturation model is off).
- The holding point of ~0.25 Nm per motor needs ~10–12 A, which confirms the 11 A estimate.
- If these commands were in output units, the 0.25 ratio would give ~4× lower currents than measured. Confirm which `rotor_to_output_ratio` was active during the test.

## 3. Thermal test procedure

Heat with **pure positive d-current**. This produces the same copper loss as holding torque, but no torque. The rotor may stay free; it locks to the field.

1. Let the motor sit unpowered until it is at room temperature. Note the ambient temperature `T_cold`.
2. In tview, watch:
   - `servo_stats.d_A` (measured d-current)
   - `servo_control.d_V` (applied d-voltage)
   - `servo_stats.filt_fet_temp_C` (board temperature)
   - `servo_stats.fault`, `servo_stats.mode`
3. **R_cold**: before heating, do one measurement step:
   ```
   d dq 5 0
   ```
   Hold for 2–3 s and average `d_A1` and `d_V1` by eye or with the plot.
   ```
   d dq 15 0
   ```
   Hold for only ~1 s and read `d_A2` and `d_V2` right after the step. At 15 A the winding heats noticeably within seconds: `d_V` drifts up ~4 % over 3 s, which is ~10 K. Then calculate R_cold. It should be close to `motor.resistance_ohm`.
4. **Heat** at the test current:
   ```
   d dq 11 0
   ```
5. **Every 2 min**, run a measurement cycle:
   ```
   d dq 5 0
   d dq 15 0
   d dq 11 0
   ```
   Hold 5 A for 2–3 s, but 15 A for only ~1 s, reading `d_V2` right after the step. Then go back to 11 A straight away. Keep each cycle under ~10 s so the heating isn't disturbed. Also measure the mount surface temperature (IR or thermocouple) right next to the stator.
6. **Stop** when either of these happens:
   - Steady state: T_winding changes by < 1 K over 5 min (expect 20–40 min).
   - A limit is hit: T_winding ≥ 80 °C, mount ≥ 60 °C, board near 75 °C, or any fault.
   ```
   d stop
   ```
7. Repeat for each cooling configuration (no fans / fans / fans + duct) and for both motors. Let the motor cool to ambient between runs.

## 4. Formulas

| Quantity | Formula |
|---|---|
| Resistance | R = (d_V2 − d_V1) / (d_A2 − d_A1) |
| Mean winding temperature | T = (R / R_cold) · (234.5 + T_cold) − 234.5 (+1 % R ≈ +2.5 K) |
| Temperature rise | ΔT = T_steady − T_ambient |
| Max continuous current | I_max = I_test · √(ΔT_allowed / ΔT_measured), ΔT_allowed = T_limit − T_ambient |
| Continuous output torque per motor | τ_out ≈ I_max · Kt · 5.5 · 0.9 |

Notes:
- The two-current slope cancels the constant inverter dead-time voltage offset.
- The calculated temperature is the winding mean. With a fixed electrical angle, the hottest phase can be 5–10 K or more above it.
- The I_max extrapolation ignores R rising with temperature, so it is slightly optimistic. Apply ~10 % margin.
- Use the mount temperature with T_limit = 60 °C for the PETG limit, and the winding temperature with T_limit = 80 °C. The lower resulting I_max applies.

## 5. Cold reference (per motor)

R_cold is measured once on a motor at room temperature. Every run calculates T_winding against it, even if the run starts warm.

| Motor | Date | T_cold [°C] | d_A1 [A] | d_V1 [V] | d_A2 [A] | d_V2 [V] | R_cold [Ω] | Config R [Ω] | Screenshot |
|---|---|---|---|---|---|---|---|---|---|
| 5 | 2026-10-16 | 23 | 5.0 | 0.63 | 15.0 | 1.67 | 0.104 | 0.1018 | `x4108_thermal/motor5_cold_ref.png` |
| 6 | | | | | | | | 0.1167 | |

## 6. Test run header (copy per run)

| Field | Value |
|---|---|
| Date | 2026-10-16 |
| Motor (5 / 6) | 5 |
| Cooling (no fans / fans / fans + duct) | no fans |
| Screenshots | `x4108_thermal/run1/` |
| Test current [A] | 8 |
| Ambient [°C] | 23 |
| R_cold [Ω] | 0.104 (cold reference, section 5) |
| Start condition | Motor still warm from previous tests, not at ambient |
| Notes | |

## 7. Measurement table (copy per run)

| time [min] | d_A1 [A] | d_V1 [V] | d_A2 [A] | d_V2 [V] | R [Ω] | T_winding [°C] | mount [°C] | board [°C] | notes |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 5.0 | 0.635 | 15.0 | 1.68 | 0.1045 | 24 | 25 | | start; motor was effectively cold |
| 1 | 5.0 | 0.70 | 15.1 | 1.84 | 0.1125 | 44 | 26 | | |
| 2 | 5.0 | 0.73 | 15.0 | 1.91 | 0.118 | 58 | 28 | | |
| 3 | 5.0 | 0.75 | 15.0 | 1.99 | 0.124 | 72 | 42 | | still rising ~14 K/min, no flattening |
| 4 | 5.0 | 0.785 | 15.0 | 2.05 | 0.1265 | 79 | 58 | | 8 A level d_V 1.12 → 1.155 V suggests ~83 °C |
| 5 | 5.0 | 0.80 | 15.0 | 2.13 | 0.133 | ~90 (±5) | 63 | | Run stopped: winding > 80 °C, mount > 60 °C. 15 A onset reads 95 °C, 8 A level 1.19 V reads ~89 °C |

Run result: no steady state. At 8 A (~10 W) with no fans, the winding rises ~11–14 K/min, slowing only slightly. The mount follows the winding closely (58 °C at 79 °C winding), so the PETG mount is the stator's main heat path. Limits were reached after ~4–5 min: winding 80 °C at ~4 min, mount 60 °C at ~4.5 min.

Cooldown: about 5 min after stop, the thermal camera hotspot on the motor was 60 °C.

Rough single-body fit of the heating curve (dT/dt = a − ΔT/τ): a ≈ 22 K/min and τ ≈ 5–7 min, so ΔT_ss at 8 A ≈ 110–150 K. That puts I_max (no fans) ≈ 5 A, which is ~0.1 Nm per motor. Uncertainty is ~±30 %; verify with a 5 A run.
| | | | | | | | | | |
| | | | | | | | | | |
| | | | | | | | | | |
| | | | | | | | | | |
| | | | | | | | | | |
| | | | | | | | | | |
| | | | | | | | | | |
| | | | | | | | | | |
| | | | | | | | | | |
| | | | | | | | | | |

## 8. Results summary

| Motor | Cooling | Test current [A] | ΔT winding steady [K] | ΔT mount steady [K] | I_max extrapolated [A] | Continuous output torque [Nm] |
|---|---|---|---|---|---|---|
| 5 | no fans | | | | | |
| 5 | fans | | | | | |
| 5 | fans + duct | | | | | |
| 6 | no fans | | | | | |
| 6 | fans | | | | | |
| 6 | fans + duct | | | | | |

## 9. Safety

- Use **positive d-current only**. Negative d-current weakens the magnet field and has no benefit here.
- Abort at T_winding ≥ 80 °C, mount ≥ 60 °C, or on any `servo_stats.fault`. Keep `d stop` typed and ready.
- Motor overtemperature protection is disabled. You are the protection, so don't leave the test unattended.
- The board faults at 75 °C (`servo.fault_temperature`). Watch `filt_fet_temp_C`, especially in runs without fans.
