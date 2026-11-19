# Tuning logs

Joint state logs (500 Hz, position, velocity, effort per joint) from tuning sessions on the arm, recorded and analyzed
with [joint_state_log.py](joint_state_log.py). `pvc_first_debug/` holds the first position-velocity controller logs.

## Impedance logs, 2026-10-30

`impedance_*.csv`, pushed by hand in impedance mode while the joint damping cap was being developed (see
[cartesian impedance, section 9](../cartesian_impedance/README.md#9-capping-the-joint-damping)).

**Which config each log was recorded with was not written down.** That was an oversight, and a bad one: the logs
are the only record of the discarded variants, and now they can only be matched afterwards. The table below is
reconstructed from the order of the tests as remembered, the start times and the oscillations in the data. Name
future logs after the config, or put a note with the parameters and the controller version next to them.

| Step | Log | Start (CET) | Variant | Seen in the data | Assignment |
|---|---|---|---|---|---|
| 1 | `impedance_fail.csv` | 20:51 | no cap, `torque_rise_time` 0.03 | axis4–6 at 26 Hz, runaway until the end of the log | certain: the 26 Hz is the signature of the active rate limit |
| 2 | `impedance_test1.csv` | 21:16 | no cap, `torque_rise_time` 0.003 | small bursts: axis1 90 Hz, axis4 48 Hz, axis5 44 Hz | certain (order and time) |
| 2 | `impedance_test2.csv` | 21:18 | no cap, `torque_rise_time` 0.003 | axis4 at 42 Hz (≈ 5–13 s), then axis5 at 39 Hz, runaway until the end of the log | certain (order and time) |
| 3 | not recorded | – | diagonal cap on the wrist joints only, axis1–3 uncapped | axis1 still oscillated (remembered) | – |
| 4 | `impedance_test3.csv` | 22:44 | cap that also counts the couplings (cross entries), axis1 still uncapped | wrist at 5.6 Hz for ~15 s (too little damping), at the same time axis1 at 86 Hz (too much) | likely: both signatures fit this variant |
| 5 | `impedance_test4.csv` | 23:07 | diagonal cap incl. axis1, the final version (commit `4415d7f`, 5 min later) | almost stable, only small bursts (axis5 38 Hz, axis4 48 Hz) | likely |

Frequencies are the spectral peak of the velocity during the bursts. Step 3 had problems on axis1, which is why
axis1 got a cap. Step 4 is the variant that pushed the wrist damping below the lower limit `K_q,ii·T`, which is why
it was dropped.

## Plotting

```bash
python3 docs/tuning/joint_state_log.py analyze docs/tuning/impedance_test2.csv --joints axis4 axis5 --fmax 150 --highpass 20 --no-show
```
`--highpass 20` removes the hand motion (below ~2 Hz), otherwise it dominates the
spectrum over a whole log. `--start` / `--end` select a time window in s from the first sample.
