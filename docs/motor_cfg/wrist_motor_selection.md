# Wrist motor selection: X4108 → T-Motor MN5008 KV170

## Summary

The original X4108 380KV wrist motors could not hold the wrist load continuously. The heat they need to produce is about 4–5× what the PETG-mounted setup can remove.

**Decision: T-Motor Antigravity MN5008 KV170, 1:6 belt ratio (up from 1:5.5), and a 35 mm fan per motor.** It costs +78 g per motor pair and cuts holding heat by about 4×. Tests confirmed it (see [mn5008_testing.md](mn5008_testing.md)):

| | Continuous wrist pitch torque |
|---|---|
| Target range | 1.5–2.5 Nm |
| X4108, no cooling | ~1.1 Nm |
| MN5008, no cooling | ~2.3–2.5 Nm (mount at the 60 °C PETG limit) |
| **MN5008, 35 mm fan** | **~5 Nm** (mount at ~31 °C when holding 2.5 Nm) |

## Problem: X4108 thermal test

See [x4108_testing.md](x4108_testing.md) for the full measurement.

| Item | Value |
|---|---|
| Test | Motor 5, 8 A pure d-current, no fans, PETG mount |
| Winding temperature | 24 → ~90 °C in 5 min, still rising ~11–14 K/min |
| Mount temperature | 25 → 63 °C, follows the winding closely (PETG mount is the stator's main heat path) |
| Steady state | Not reached; estimated ΔT_ss at 8 A ≈ 110–150 K |
| Estimated continuous capacity | ~5 A ≈ 4 W copper loss ≈ 0.1 Nm per motor (±30 %) |
| Required for holding (1 kg at full reach) | ~0.25 Nm per motor ≈ 11 A ≈ 19–22 W |

## Constraints

- Belt drive is mandatory, and no extra gear stage is possible.
- Maximum ratio is 1:6, limited by belt geometry and space.
- Weight budget is about +80 g for the motor pair.
- No room for a 40 mm fan.

## Selection metric

Heat for a given torque is set by the motor constant Km = Kt / √R_LL:

- Kt = 8.27 / Kv [Nm/A] (moteus convention; matches the X4108 torque test within a few %)
- R_LL = line-to-line resistance (2 × moteus `motor.resistance_ohm`)
- Copper loss P = 0.75 · R_LL · I² = 0.75 · (τ / Km)²
- Heat also scales with 1/ratio², so 1:5.5 → 1:6 cuts it by ~16 % at no cost.

Required motor torque for 2.5 Nm wrist pitch at 1:6: 0.25 Nm · 5.5 / 6 = **0.229 Nm**.

## Candidates

Datasheet values, except where marked.

| Motor | Weight | Kv | R_LL [Ω] | Km [Nm/√W] | Km/g [10⁻³] | Current at 0.229 Nm | Heat at 1:6 |
|---|---|---|---|---|---|---|---|
| X4108 380KV (previous) | 89 g | 392 (calibrated) | ~0.21 (calibrated) | 0.046 | 0.52 | ~10.9 A | ~19 W |
| MN4006 KV380 | ~66 g | 380 | 0.194 | 0.050 | 0.75 | ~10.5 A | ~16 W |
| MN5006 KV300 | 108 g | 300 | 0.125 | 0.078 | 0.72 | ~8.3 A | ~6.5 W |
| **MN5008 KV170** | **128 g** | 170 | 0.27 | **0.094** | 0.73 | **~4.7 A** | **~4.5 W** |

Findings:
- Within the T-Motor MN family, Km per gram is roughly constant, so Km has to be bought with mass. The X4108 is ~30 % weaker per gram.
- Heat scales with 1/Km², so roughly with 1/mass². The right choice is the lightest motor whose Km meets the requirement.
- Allowed dissipation without cooling was estimated at ~4–6 W, which needs Km ≳ 0.09–0.1. Only the MN5008 meets this. With a fan, the MN5006 would also have worked, but the MN5008 fits the weight budget and gives more reserve.
- The MN5008 uses the full weight budget: 2 × (128 − 89) g = +78 g.

## MN5008 KV170: datasheet vs. calibration

| Item | Datasheet | Actuator 5 | Actuator 6 |
|---|---|---|---|
| Weight | 128 g incl. cable | | |
| Configuration | 24N28P (14 pole pairs) | `motor.poles 28` | `motor.poles 28` |
| Kv | 170 | 156.4 | 155.5 |
| Kt [Nm/A] | 0.049 | 0.0529 | 0.0532 |
| R_LL [Ω] | 0.27 | 0.301 (calibration), 0.293 (cold measurement) | 0.288 (calibration), 0.291 (cold measurement) |
| Km [Nm/√W] | 0.094 | 0.096 | 0.099 |
| Holding 2.5 Nm at 1:6 | ~4.5 W | 4.4 A, ~4.3 W | 4.3 A, ~4.1 W |

- Both motors are slightly better than the datasheet: the lower Kv more than compensates for the slightly higher resistance.
- The pair is well matched (Kv within 0.5 %, cold R within 0.5 %). The X4108 pair differed by ~12 % in Kv.
- Other datasheet values: Ø55.6 × 32 mm, 6 mm shaft, copper class H (180 °C), magnets class E (150 °C), peak current 15 A for 180 s. Some retailers list 720 mΩ; this is a copy error.
- Speed at 24 V: ~3,700 rpm no-load → ~600 rpm output at 1:6.

## Cooling

A 35 mm fan per motor, aimed at the bell openings. At the 2.5 Nm holding load it cut the temperature rise by ~3–4× (mount 31 °C instead of ≥ 60 °C).

Other options considered:
- **Aluminium foil** between motor and mount: no measurable effect. At ~0.02 mm it is too thin to spread heat sideways.
- **0.5–1 mm aluminium plate** between motor and mount, exposed to air: estimated ~2× cooling capacity at ~9–17 g per motor. Not needed with the fan.
- **30 vs. 40 mm fan:** a 30 mm fan gives roughly 60–85 % of a 40 mm fan's effect, depending on airflow. Free-air flow (CFM) matters more than static pressure for an unducted fan.

## Open points

| Point | Action |
|---|---|
| Configs still have `motor_position.rotor_to_output_ratio 0.25` (1:4) | Set 0.16667 once the 1:6 belt is fitted |
| Configs still have `servo.max_current_A 40` (~350 W copper loss at 40 A) | Set 15 A: ~0.8 Nm per motor, ~40 % above the worst expected load and equal to T-Motor's peak rating |
| Motor overtemperature protection is disabled (no thermistor) | Glue an NTC to the stator and enable `servo.enable_motor_temperature`, or add I²t limiting in software |
| +78 g at the end of the arm and larger rotor inertia | Re-check upstream joint holding torques and the wrist rotor inertia in [docs/dynamics](../dynamics/README.md) |
| Torque test (lever and scale) not repeated on the MN5008 | Optional: confirms Kt and saturation |

## Sources

- [T-Motor store: MN5008 KV170 (specs and test data)](https://store.tmotor.com/product/mn5008-kv170-motor-antigravity-type.html)
- [T-Motor UAV: Antigravity MN5008](https://uav-en.tmotor.com/2020/Antigravity_0422/340.html)
- [RobotShop: MN5008 KV170](https://www.robotshop.com/products/tmotor-uav-brushless-motor-antigravity-mn5008-kv170)
- [MAB Robotics: MN5008 for robotics](https://www.mabrobotics.pl/product-page/mn5008-ip35-antigravity-type-6-12s-uav-motor-t-motor)
- [Rehabilitation robot using MN5008 (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC11452858/)
- [GetFPV: MN5006](https://www.getfpv.com/t-motor-antigravity-mn5006-motor.html), [LIG Power: MN5006 KV300](https://www.ligpower.com/product/mn5006-kv300-motor-antigravity-type.html)
- [RobotShop: MN4006 KV380](https://uk.robotshop.com/products/ligpower-mn4006-kv380-antigravity-ultralight-efficient-uav-motor-2pcs-set)
- [Source Robotics: motor constants explained](https://source-robotics.com/blogs/blog/motor-constants-kv-kt-ke-km-explained)
