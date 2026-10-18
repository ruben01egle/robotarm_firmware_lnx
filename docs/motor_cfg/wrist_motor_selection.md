# Wrist motor selection: X4108 → T-Motor MN5008 KV170

## Summary

The current X4108 380KV wrist motors cannot hold the wrist load continuously. The heat they need to produce is about 4–5× what the PETG-mounted setup can remove.

Recommendation: **T-Motor Antigravity MN5008 KV170**, with the belt ratio raised from **1:5.5 to 1:6**, plus a small fan for margin. It costs +78 g per motor pair and cuts holding heat by about 4×.

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

## Selection metric

Heat for a given torque is set by the motor constant Km = Kt / √R_LL:

- Kt = 8.27 / Kv [Nm/A] (moteus convention; matches the X4108 torque test within a few %)
- R_LL = line-to-line resistance (2 × moteus `motor.resistance_ohm`)
- Copper loss P = 0.75 · R_LL · I² = 0.75 · (τ / Km)²

Required motor torque at 1:6: 0.25 Nm · 5.5 / 6 = **0.229 Nm**.

## Candidates

| Motor | Weight | Kv | R_LL [Ω] | Km [Nm/√W] | Km/g [10⁻³] | Current at 0.229 Nm | Heat at 1:6 |
|---|---|---|---|---|---|---|---|
| X4108 380KV (current) | 89 g | 392 (measured) | ~0.21 (measured) | 0.046 | 0.52 | ~10.9 A | ~19 W |
| MN4006 KV380 | ~66 g | 380 | 0.194 | 0.050 | 0.75 | ~10.5 A | ~16 W |
| MN5006 KV300 | 108 g | 300 | 0.125 | 0.078 | 0.72 | ~8.3 A | ~6.5 W |
| **MN5008 KV170** | **128 g** | 170 | 0.27 | **0.094** | 0.73 | **~4.7 A** | **~4.5 W** |

Findings:
- Within the T-Motor MN family, Km per gram is roughly constant, so Km has to be bought with mass. The X4108 is ~30 % weaker per gram.
- Heat scales with 1/Km², so roughly with 1/mass². The right choice is the lightest motor whose Km meets the requirement.
- Allowed dissipation without cooling is ~4–6 W, which needs Km ≳ 0.09–0.1. Only the MN5008 meets this. With a fan (~10–12 W allowed), the MN5006 would also work.
- The MN5008 uses the full weight budget: 2 × (128 − 89) g = +78 g.
- 1:5.5 → 1:6 cuts heat by ~16 % at no cost.

## MN5008 KV170 data

From the official T-Motor datasheet:

| Item | Value |
|---|---|
| Size | Ø55.6 × 32 mm, 6 mm shaft |
| Weight | 128 g incl. cable |
| Configuration | 24N28P (14 pole pairs) |
| Internal resistance | 270 mΩ (some retailers list 720 mΩ; this is a copy error) |
| Insulation | Copper class H (180 °C), magnets class E (150 °C) |
| Kt (from Kv) | ~0.049 Nm/A |
| Speed at 24 V | ~3,700 rpm no-load → ~600 rpm output at 1:6 |

Plausibility check against T-Motor's bench table: at 0.87 Nm, 6,000 rpm, 687 W, total loss is ~144 W. Kt and R predict ~65 W copper loss; the rest is iron and ESC loss at speed. This is consistent.

No published robotics stall or thermal data for this motor was found. The datasheet temperatures (63–89 °C) are with propeller airflow and do not transfer to a stalled wrist.

## Risks and open points

| Risk | Mitigation |
|---|---|
| No-cooling margin is thin (4.5 W vs ~4–6 W, ±30 %) | Plan a small fan; an aluminium heat path between stator and PETG helps further |
| Datasheet R and Kv not yet verified | moteus calibration on the first motor |
| Motor overtemperature protection is disabled (no thermistor) | Glue an NTC to the stator and enable `servo.enable_motor_temperature`, or add I²t limiting in software |
| Larger diameter (55.6 vs ~46 mm) | New mounts and belt layout |
| +78 g at the end of the arm | Within budget; re-check upstream joint holding torques |

## Validation plan

1. Order **one** MN5008 KV170 and calibrate it with moteus. Check R_LL ≈ 0.27 Ω and Kv ≈ 170.
2. Measure a cold reference and run the d-current test from [x4108_testing.md](x4108_testing.md) at **~4.7 A, no cooling**.
3. Pass criteria: steady state with winding < 80 °C and mount < 60 °C. If it passes, order the second motor. If it fails, repeat with a fan.
4. Repeat the lever/scale torque test to confirm Kt and saturation.

## Sources

- [T-Motor store: MN5008 KV170 (specs and test data)](https://store.tmotor.com/product/mn5008-kv170-motor-antigravity-type.html)
- [T-Motor UAV: Antigravity MN5008](https://uav-en.tmotor.com/2020/Antigravity_0422/340.html)
- [RobotShop: MN5008 KV170](https://www.robotshop.com/products/tmotor-uav-brushless-motor-antigravity-mn5008-kv170)
- [MAB Robotics: MN5008 for robotics](https://www.mabrobotics.pl/product-page/mn5008-ip35-antigravity-type-6-12s-uav-motor-t-motor)
- [Rehabilitation robot using MN5008 (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC11452858/)
- [GetFPV: MN5006](https://www.getfpv.com/t-motor-antigravity-mn5006-motor.html), [LIG Power: MN5006 KV300](https://www.ligpower.com/product/mn5006-kv300-motor-antigravity-type.html)
- [RobotShop: MN4006 KV380](https://uk.robotshop.com/products/ligpower-mn4006-kv380-antigravity-ultralight-efficient-uav-motor-2pcs-set)
- [Source Robotics: motor constants explained](https://source-robotics.com/blogs/blog/motor-constants-kv-kt-ke-km-explained)
