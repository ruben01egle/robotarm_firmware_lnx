# MN5008 wrist motor testing (actuators 5 & 6)

Thermal test of the T-Motor MN5008 KV170 wrist motors (2026-10-22). Background and motor selection are in [wrist_motor_selection.md](wrist_motor_selection.md); the previous X4108 results are in [x4108_testing.md](x4108_testing.md).

## Result

| | No cooling (run 1) | **35 mm fan (run 2)** |
|---|---|---|
| Load | 4.5 A per motor ≈ 2.5 Nm wrist pitch (upper target) | same |
| Winding | 59 °C at 9 min, still rising | **~32 °C steady** (ΔT ≈ 9 K) |
| PETG mount | 60 °C (limit) at 9 min, still rising | **31 °C steady** (ΔT ≈ 8 K) |
| Continuous current per motor (extrapolated) | ~4.0–4.5 A | **~9–10 A** (mount-limited) |
| Continuous wrist pitch torque | ~2.3–2.5 Nm | **~5 Nm** (~4.5 Nm with 10 % margin) |

- **Without cooling, the upper target of 2.5 Nm is borderline.** The mount reaches the 60 °C PETG limit, with no reserve for roll load.
- **With a 35 mm fan, the wrist holds 2.5 Nm with the mount at ~31 °C.** The fan cuts the temperature rise by ~3–4×. That leaves about 2× reserve for combined pitch and roll loads.
- **Decision:** MN5008 KV170 + 1:6 belt + 35 mm fan per motor. No further test runs planned.
- The extrapolation for run 2 is based on a small ΔT (8–9 K) with ±3 K reading error per point. Treat the ~5 Nm as ±15–25 %.

## Context

| Item | Value |
|---|---|
| Wrist | Belt-driven differential, 2 motors share pitch load |
| Motor | T-Motor Antigravity MN5008 KV170, 128 g, 24N28P |
| Driver | mjbots moteus r4.11 (cooled) |
| Gear ratio | 1:6 belt |
| Actuator 5 | `motor.resistance_ohm 0.1506`, `motor.Kv 156.4` → Kt ≈ 8.27/Kv = 0.0529 Nm/A, Km ≈ 0.096 Nm/√W |
| Actuator 6 | `motor.resistance_ohm 0.1442`, `motor.Kv 155.5` → Kt ≈ 0.0532 Nm/A, Km ≈ 0.099 Nm/√W |
| Motor temp protection | Disabled (`servo.enable_motor_temperature 0`, no thermistor) |
| Ambient | 23 °C |

### Load cases (1:6, η = 0.9, both motors share pitch)

| Wrist torque | Motor torque | Current | Copper loss per motor |
|---|---|---|---|
| 1.5 Nm (minimum target) | 0.139 Nm | 2.6 A | 1.5 W |
| 2.0 Nm | 0.185 Nm | 3.5 A | 2.6 W |
| 2.5 Nm (upper target) | 0.231 Nm | 4.4 A | 4.3 W |
| 2.5 Nm pitch + 1 Nm roll (heavier motor) | 0.324 Nm | 6.1 A | ~8.5 W |

In the differential, each motor carries (τ_pitch ± τ_roll) / 2, so roll load lands on one motor only.

## Method

Heating with **pure positive d-current**: the same copper loss as holding torque, but no torque, so the rotor needs no blocking. The winding temperature comes from the resistance increase.

1. With the motor at room temperature, plot `servo_stats.d_A` and `servo_control.d_V` in tview.
2. Heat at the test current: `d dq 4.5 0`.
3. Measure R every 2–3 min: read `d_A1`/`d_V1` as the mean at 4.5 A, step to `d dq 15 0` for ~1 s and read `d_A2`/`d_V2` right after the step, then go back to `d dq 4.5 0`. Take a screenshot and read the PETG mount temperature with a thermal camera.
4. The first measurement (t = 0) on the cold motor gives R_cold.
5. Stop at steady state (< 1 K change over 5 min) or at a limit (winding 80 °C, mount 60 °C, any fault): `d stop`.

| Quantity | Formula |
|---|---|
| Resistance | R = (d_V2 − d_V1) / (d_A2 − d_A1) |
| Mean winding temperature | T = (R / R_cold) · (234.5 + T_cold) − 234.5 (+1 % R ≈ +2.5 K) |
| Max continuous current | I_max = I_test · √(ΔT_allowed / ΔT_measured), ΔT_allowed = T_limit − T_ambient |
| Continuous wrist pitch torque | τ_wrist ≈ 2 · I_max · Kt · 6 · 0.9 |

Notes:
- The two-current slope cancels the inverter dead-time voltage offset.
- The calculated temperature is a mean weighted towards the phase carrying most of the current. With a fixed electrical angle, the hottest phase can be 10–15 K above it.
- Limits: mount 60 °C (PETG) and winding 80 °C. The lower resulting I_max applies. Here the mount is the limit.
- The I_max extrapolation ignores R rising with temperature, so it is slightly optimistic. Apply ~10 % margin.

Lessons from these runs:
- The cold reference must use the same two currents and the same direct step as the run. A first motor 6 reference with a stop between 4.5 A and 15 A read 3 % high (0.1501 Ω) and was discarded.
- Keep the 15 A step to ~1 s and measure every 2–3 min, not every minute. Each 15 A step adds ~56 W. In run 1 (1–2.6 s every minute) the steps added ~1 W on average, ~25 % on top of the test load, so run 1 is harsher than the real holding case.
- Always aim the thermal camera at the same spot on the PETG mount, next to the motor base.

## Cold reference

| Motor | T_cold [°C] | d_A1 [A] | d_V1 [V] | d_A2 [A] | d_V2 [V] | R_cold [Ω] | Config R [Ω] | Screenshot |
|---|---|---|---|---|---|---|---|---|
| 5 | 23 | 4.4 | 0.745 | 15.1 | 2.31 | 0.1463 | 0.1506 | `mn5008_thermal/motor5_cold_ref.png` |
| 6 | 23 | 4.48 | 0.75 | 15.0 | 2.28 | 0.1456 | 0.1442 | `mn5008_thermal/motor6_cold_ref.png` (= run 2 t0) |

Both motors are within 0.5 % of each other when cold. The moteus calibration values differ by 4 %, most likely because motor 5 was warmer during calibration.

## Run 1: motor 5, 4.5 A, no cooling

Screenshots are in `mn5008_thermal/run1/`.

| time [min] | d_A1 [A] | d_V1 [V] | d_A2 [A] | d_V2 [V] | R [Ω] | T_winding [°C] | mount [°C] | notes |
|---|---|---|---|---|---|---|---|---|
| 0 | 4.47 | 0.777 | 15.1 | 2.32 | 0.1452 | ~23 | 25 | |
| 0.9 | 4.52 | 0.803 | 15.0 | 2.41 | 0.1532 | 35 | 27 | 15 A held ~2.6 s |
| 1.9 | 4.47 | 0.83 | 15.0 | 2.47 | 0.1552 | 39 | 29 | |
| 2.9 | 4.46 | 0.85 | 14.9 | 2.49 | 0.1571 | 42 | 40 | |
| 3.9 | 4.46 | 0.86 | 15.07 | 2.53 | 0.1574 | 42 | 46 | |
| 4.9 | 4.46 | 0.865 | 15.0 | 2.53 | 0.158 | 44 | 51 | 15 A held ~1.8 s |
| 5.9 | 4.49 | 0.87 | 15.0 | 2.58 | 0.1627 | 52 | 56 | |
| 7.0 | 4.49 | 0.90 | 15.0 | 2.64 | 0.1656 | 57 | 54 | |
| 7.9 | 4.44 | 0.90 | 15.0 | 2.66 | 0.1665 | 59 | 56 | |
| 8.9 | 4.52 | 0.90 | 15.2 | 2.68 | 0.1667 | 59 | 60 | stopped: mount at 60 °C limit |

- The winding was flattening from ~7 min; the 4.5 A d_V level stayed at 0.90 V.
- The mount was still rising ~2 K/min. Estimated steady state is ~62–68 °C, or ~55–60 °C for the real holding case without the extra step heat.
- The mount read about equal to the step-method winding mean, which is physically unlikely. Either the step method reads the winding low, or the camera spot partly covered the motor base. The 4.5 A level method gives ~70 °C. The mount reading is what limits the result either way.

## Run 2: motor 6, 4.5 A, 35 mm fan

Screenshots are in `mn5008_thermal/run2/`. Motor 6 was used because motor 5 was still warm. The two motors are nearly identical, so runs 1 and 2 compare the cooling, not the motor.

| time [min] | d_A1 [A] | d_V1 [V] | d_A2 [A] | d_V2 [V] | R [Ω] | T_winding [°C] | mount [°C] | notes |
|---|---|---|---|---|---|---|---|---|
| 0 | 4.48 | 0.75 | 15.0 | 2.28 | 0.1456 | 23 | 22 | used as R_cold |
| 2.2 | 4.47 | 0.755 | 15.04 | 2.33 | 0.149 | 29 | 28 | |
| 4.2 | 4.51 | 0.76 | 15.0 | 2.35 | 0.1516 | 34 | 29 | |
| 6.3 | 4.47 | 0.77 | 15.02 | 2.37 | 0.1517 | 34 | 30 | |
| 8.4 | 4.47 | 0.775 | 15.1 | 2.37 | 0.150 | 31 | 31 | |
| 10.4 | 4.44 | 0.775 | 15.02 | 2.355 | 0.1493 | 30 | 31 | steady |

- Steady after ~8–10 min: mount unchanged over the last 2 min, 4.5 A d_V level flat since 8 min.
- Winding ~32 °C (mean of the last four points, ΔT ≈ 9 K). Mount 31 °C (ΔT ≈ 8 K).
- In the first ~2 min, run 2 heated only slightly slower than run 1 once the extra step heat in run 1 is accounted for. The fan's effect shows once the motor is well above ambient; from ~4 min the runs clearly separate (6 min: 34 vs 52 °C winding, 30 vs 56 °C mount).

## Safety notes for future tests

- Use **positive d-current only**.
- Abort at winding ≥ 80 °C, mount ≥ 60 °C, or on any `servo_stats.fault`. Keep `d stop` ready.
- Motor overtemperature protection is disabled, so don't leave a test unattended.
