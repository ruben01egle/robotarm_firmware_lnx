# Runtime PID gain scaling — design notes

Working notes from an investigation into adding `kp_scale`/`kd_scale`/`ilimit_scale`
to the per-cycle CAN-FD command frame in `MoteusInterface.cpp`, and the control-theory
and ros2_control-architecture questions that came out of it. Nothing here is implemented
yet — this is a record of what was found and decided (or left open) so the work can be
picked up later without re-deriving it.

Source of truth for all byte-math below: `WriteCombiner` in
`build/moteus_interface/_deps/moteus-src/lib/cpp/mjbots/moteus/moteus_multiplex.h` and
`PositionMode::Make` in `.../moteus_protocol.h`, plus `RoundUpDlc` in
`.../moteus_transport.h` and its byte-identical reimplementation in this package's own
`TransportUSB.cpp`.

## 1. Why this came up

The servo loop is maxing out the controllers' capabilities. The idea was to add two more
fields (`kp_scale`, `kd_scale`) to the per-cycle CAN-FD command, to allow live PID gain
adjustment. Before touching any code, the goal was to fully understand what that costs
on the wire, because the CAN-FD frame is padded up to fixed DLC brackets
(`8,12,16,20,24,32,48,64` bytes) — the actual transmitted size is never just "bytes
written", it's whichever bracket the write rounds up into.

## 2. Current baseline frame size (as of this writing, no scales added)

`write_format` in `on_configure()` (`MoteusInterface.cpp:530-545`) sends
`position/velocity/feedforward_torque` as `kFloat`, everything else `kIgnore`.

| Piece | Bytes |
|---|---|
| Mode set (`kMode`) | 3 |
| position+velocity+torque group (3× `kFloat`, one combined write, 2-byte header + 3×4) | 14 |
| Appended query (`read_format`: pos/vel/torque as Int16, fault as Int8) | 4 |
| **Actual** | **21** |
| **RoundUpDlc(21)** | **24** |

3 bytes of headroom exist in the current 24-byte bracket before the next jump (to 32).

## 3. Adding kp_scale + kd_scale — byte cost

`kCommandKpScale`/`kCommandKdScale` (registers `0x023`/`0x024`) are contiguous with
`feedforward_torque` (`0x022`) but a **different resolution**, so `WriteCombiner` can't
merge them into the existing float group — they always cost a second header.

- **Adding only one of the two** (Int8): +3 bytes (2-byte header + 1 byte value) →
  21+3 = 24 bytes → **still fits the current 24 bracket, zero DLC growth.**
- **Adding both** (Int8 each): +4 bytes → 21+4 = 25 bytes → **rounds up to 32** (+8 bytes
  actual wire size, +33%, driven almost entirely by crossing the DLC boundary, not by
  the 2 logical payload bytes).

## 4. The fix: downgrade position/velocity/torque from Float to Int16

`WriteCombiner` groups by resolution equality among *consecutive* registers — reordering
fields doesn't help, only matching resolutions does. Downgrading the float trio to Int16
shrinks that group from 14 bytes (2-byte header + 3×4) to 8 bytes (2-byte header + 3×2),
freeing enough room to add the scale registers without leaving the current bracket:

| Format | Actual bytes | DLC |
|---|---|---|
| Current (float, no scales) | 21 | 24 |
| Float + kp/kd (Int8) | 25 | 32 |
| **Int16 trio + kp/kd (Int8)** | 19 | 20 |
| **Int16 trio + kp/kd/ilimit (all Int8)** | 22 | **24** |

The last row is the one that matters: with Int16 position/velocity/torque, you can add
**all three** scale registers (kp_scale, kd_scale, ilimit_scale) and still land in the
same 24-byte bracket you're using today with plain floats and no scales at all.
(`ilimit_scale` is register `0x02b`, not contiguous with kp/kd — 6 registers of gap —
so it costs its own 3-byte group: 2-byte header + 1 byte value.)

**Checked and rejected:** merging kp_scale/kd_scale into the float group by also making
them Int16 (all 5 registers, one group). This does *not* help — going to one group saves
a header byte, but promoting kp/kd from Int8 to Int16 costs 2 more value bytes. Net: 20
bytes vs. 19 for the split version, and both round up to the same 20-byte DLC bracket
anyway. Also no accuracy upside: kp_scale/kd_scale use the `kPwm` mapping, and Int8
already gives 1/127 ≈ 0.79% steps across the 0–1 scale range, which is already fine for
a multiplier. **Keep kp/kd/ilimit as Int8, don't merge them with the position group.**

## 5. Precision impact of the Int16 downgrade

Int16 fixed-point scales (fixed regardless of range): position 0.0001 rev (0.036°/LSB),
velocity 0.00025 rev/s (0.09°/s per LSB), torque 0.01 Nm/LSB.

- Joints never exceed ±300° → well within the Int16 representable range (±32767×0.0001
  rev ≈ ±1180°), no saturation risk, ~25% utilization even at the extreme.
- 0.036° position quantization is below typical mechanical repeatability; translated to
  end-effector error at plausible link lengths it's ~0.06mm@0.1m to ~0.6mm@1m — noise
  floor for anything short of sub-tenth-mm precision work.
- Importantly: `read_format` (state feedback) is **already** Int16 at these exact
  resolutions (`MoteusInterface.cpp:548-550`). Downgrading the write side just matches a
  precision ceiling the control loop already lives with on the read side — it isn't a
  new limitation.

## 6. Differential wrist — confirmed no saturation risk from Int16

`DifferentialTransmission.cpp:31-47`: `combine(j1, j2) → a = j1+j2, b = j1-j2`, unity
ratio, no gearing scalar in this transmission. That means the number to check isn't the
per-joint limit, it's the worst-case **sum** of both joints' extremes landing on one
actuator.

- Joint1 ±115°, Joint2 ±175° → worst case either actuator sees up to **±290°**
  (both joints at same-signed extremes for `a`, opposite-signed for `b`).
- 290° ≈ 0.806 rev, vs. Int16's ±3.2767 rev ceiling → **~24.6% utilization at the
  absolute worst case**, roughly 4× headroom over the mechanical limits. Safe.

## 7. The integral term — how it actually behaves, and why torque mode already zeros `ilimit_scale`

From `fw/pid.h:136-159`:
```cpp
to_update_i = state_->error * config_->ki * period_s;   // ki: NOT scaled by kp_scale/kd_scale
state_->integral += to_update_i;
ilimit = config_->ilimit * apply_options.ilimit_scale;    // clamp width only
clamp state_->integral to [-ilimit, ilimit];
command = pd + integral;                                  // I rides through unconditionally
```

Key facts:
- **`kp_scale`/`kd_scale` only gate P and D.** The integral term is added into the output
  regardless of those two scales — zeroing kp/kd alone does *not* silence the I term.
- There is **no `ki_scale`** register. `ki` itself is a static config value
  (`servo.pid_position.ki`), not part of the per-cycle protocol.
- `ilimit_scale=0` forces `ilimit=0`, which clamps `state_->integral` back to exactly 0
  **every cycle** — not just capping growth, but actively zeroing any residual value
  (including leftover integral from a previous control mode), with no explicit reset
  command needed.
- In pure torque mode (`position=NaN, velocity=0`), `control_position_raw` is captured
  once and then frozen (`bldc_servo_position.h:317-341` — trajectory update is skipped
  since velocity=0 means `trajectory_done=true` from the start). If the arm then moves
  under torque control, position error grows against that stale snapshot, and `ki` would
  integrate that growing error into a phantom "return to where torque mode was engaged"
  torque — exactly what `ilimit_scale=0` prevents.
- **Already correctly done**: `MoteusInterface.cpp:801-813` (TORQUE_CONTROL branch)
  already sets `kp_scale=0, kd_scale=0, ilimit_scale=0` together. This matches the
  firmware's own pattern for fully killing PID contribution
  (`bldc_servo_control.h:1391-1394`, the position-bounds-violation fault path). No
  change needed here — just confirmed as correct.

## 8. "Velocity mode" (kp_scale=0, velocity commanded, feedforward-heavy) — why it oscillated

Observed: setting `kp_scale=0` while relying on the internally-generated trajectory (a
continuously-advancing `control_position_raw`, built from the commanded velocity — see
`bldc_servo_position.h:200-291`, `DoVelocityModeLimits`/`DoVelocityAndAccelLimits`)
caused growing-amplitude oscillation until the E-stop was hit.

Two candidate mechanisms found in the firmware, not mutually exclusive:

1. **Deadband** (`bldc_servo_control.h:83-86`, `Threshold()`): if
   `|actual_velocity - velocity_command|` is within `±velocity_threshold`,
   `measured_velocity` snaps to `velocity_command` exactly, i.e. `error_rate = 0` and D
   contributes **nothing**, regardless of how far position has drifted. Outside that
   band D switches on abruptly (a hard relay, not a smooth blend). **Caveat**:
   `velocity_threshold` defaults to `0.0f` (`bldc_servo_structs.h:547`), and at exactly 0
   the deadband condition can never be true — so this mechanism is a no-op *unless*
   `servo.velocity_threshold` has been explicitly configured nonzero. **Not yet checked
   against the actual device config** — do this before treating it as the cause.

2. **`kd` was never validated as a standalone loop gain** (more likely the real cause,
   independent of the config check above): `kd` is tuned as the *damping companion* to a
   specific `kp` (`kd ≈ 2ζ√(kp·J)` — only meaningful relative to the natural frequency
   `kp` defines). Zero `kp`, and `kd` becomes the *sole* feedback gain in what's now a
   bare velocity-proportional loop, applied at full "companion" strength against a
   velocity signal that has its own estimator lag (`motor_position.h:1238`,
   `status.velocity = status.integral + filter.kp * error` — a tracking-filter output,
   not a raw finite difference). That extra phase lag, normally a minor perturbation
   when `kp` dominates the loop's bandwidth, can erode phase margin enough to make the
   loop unstable once it's carrying the full gain alone — small errors amplify each
   cycle instead of damping, growing until saturation/fault. Matches the observed
   symptom.

3. **Unexamined confound: was `ilimit_scale` also zero during this test?** The report was
   "when I accidentally set only kp_scale to zero" — which implies `ilimit_scale` was
   probably left at its default (1.0), not also zeroed. If so, `ki` (see §7) was live the
   whole time, integrating position error against the *continuously advancing* trajectory
   (unlike frozen-setpoint torque mode). A live integral fighting a live, moving reference
   — with the mechanical plant unable to track it well in the absence of P — is a classic
   windup-driven runaway, and arguably a better match for "amplitude grows continuously
   until E-stop" than a linear instability alone. **This was never isolated from
   mechanism 2 above** — both may have contributed, and it's not known which dominated.

**Practical implication, not yet implemented**: don't set `kp_scale` literally to 0 for
a soft/feedforward-heavy controller. Use a **small nonzero** `kp_scale` so the loop keeps
a well-defined natural frequency for `kd` to damp against, and reduce `kd_scale`
proportionally (roughly with `√kp_scale`) rather than leaving it at full strength against
a much weaker spring.

## 9. Pure torque mode (position=NaN, velocity=0, ilimit_scale=0) — damping is host-rate-independent

Checked whether the D-term damping in this configuration is limited by the (slower) host
control loop rate.

- `control_position_raw` freezes on entry (no accel/velocity limit ⇒ `trajectory_done`
  true from the start ⇒ `UpdateTrajectory` never runs).
- D term = `kd_scale · kd · (measured_velocity − 0)` — `measured_velocity` comes from the
  firmware's own internal velocity estimator, refreshed **every internal servo cycle**,
  independent of CAN frame arrival rate.
- **Conclusion: the damping is real-time and host-rate-independent.** Only the
  feedforward torque *value* is host-rate-limited (it's a bare held value between CAN
  writes, no interpolation) — the damping reacting to live motion is not.
- **Caveat that does depend on host rate**: the watchdog. Every received command reloads
  a countdown (`status_.timeout_s`, `bldc_servo.cc:890-893`) that ticks down every
  internal servo cycle; hitting zero drops the controller into `kPositionTimeout`
  (`bldc_servo_control.h:1670-1673`). The host must keep sending *something* — even with
  every value unchanged — at least as often as the configured timeout
  (`default_timeout_s`, since `write_format` currently leaves `watchdog_timeout` as
  `kIgnore`). This is a keepalive-rate requirement, not a control-fidelity one.

## 10. Low-stiffness, lightly-damped torque control — why & how the "velocity=0" variant works, and what it does *not* settle

This ties §7, §8, and §9 together. The controller being aimed for — mostly feedforward,
with a bit of damping, no spring-like stiffness — is built as: `position=NaN,
velocity=0` (constant, not a live trajectory), `kp_scale=0`, `kd_scale` **kept nonzero**
(unlike the currently-implemented all-zero TORQUE_CONTROL branch, which zeros `kd` too),
`ilimit_scale=0`.

### Why it's still technically "a version of velocity control"

It goes through the exact same `PositionMode`/`Mode::kPosition` command as "velocity
mode" in §8 — the *only* difference is the `velocity` field is held at the constant `0`
instead of some live nonzero value. That single difference has a real structural
consequence, per `bldc_servo_position.h:200-341`:

- With `velocity ≠ 0` (§8): `velocity_limit`/`accel_limit` logic and `UpdateTrajectory`
  actively run every cycle, continuously advancing `control_position_raw` — the reference
  the P/I terms chase is a moving target, generated by a trajectory planner with its own
  clamps and discontinuities (hitting `velocity_limit`, crossing zero, etc).
- With `velocity = 0` (§9/§10): `isnan(velocity_limit) && isnan(accel_limit)` is true from
  the first cycle ⇒ `trajectory_done = true` immediately ⇒ `UpdateTrajectory` **never
  runs** ⇒ `control_position_raw` is captured once and stays frozen. No planner, no
  clamps, no discontinuities — just a static reference sitting wherever the joint was
  when the mode was entered.

### How the "mostly feedforward + light damping" behavior actually comes out

Every cycle still runs the same `PID::Apply()` (`fw/pid.h:108`) used everywhere else in
`Mode::kPosition`:
- `error = actual_position − frozen_reference` (position domain) — `kp_scale=0` means
  this never produces torque, no matter how far the joint drifts from that frozen point.
- `to_update_i = error · ki · period_s`, but `ilimit_scale=0` clamps `state_->integral`
  to exactly 0 every cycle (§7) — no windup either, regardless of how large `error` gets.
- `error_rate = measured_velocity − 0` (velocity domain) — `kd_scale ≠ 0` here means this
  **does** produce torque: `kd_scale · kd · measured_velocity`, opposing whatever speed
  the rotor currently has.
- Net: `torque = feedforward_torque + inertia_feedforward − kd_scale·kd·measured_velocity`.
  No term anywhere in that sum depends on *position*. The only closed-loop action is a
  viscous brake reacting to motion itself — which is exactly "feedforward-heavy, no
  stiffness, some damping."

This is also why the runtime-tunable-stiffness work (§12/§14 below) is motivated in the
first place: getting here safely means never actually setting `kp_scale` to a literal,
permanent 0 baked into a fixed `write_format` (see below) — you want the ability to dial
`kp_scale` per task, per joint, and back away from 0 if a specific joint/load combination
turns out to need a bit of restoring stiffness after all.

### What this section does *not* settle — read before relying on it

The kd-alone instability argument from §8 (mechanism 2: `kd` tuned as `kp`'s companion,
unstable once it's the sole gain against a laggy velocity estimate) is a statement about
the **D-loop's own closed-loop poles** — `torque = −kd·(measured_velocity − desired_rate)`
fed through the same plant and the same estimator lag. That loop's stability does not
obviously depend on whether `desired_rate` is the constant `0` or some live moving value:
the reference value shifts what's being tracked, not the feedback loop's pole locations.
**This means mechanism 2 from §8 has not been ruled out for this "torque + kd damping"
variant either** — it was reasoned about for §8's failure, but never re-examined for
whether it also threatens this configuration. What *is* different and *does* remove one
risk cleanly: `ilimit_scale=0` here removes the windup confound (mechanism 3 in §8)
entirely, and the frozen (non-advancing, non-clamped) reference removes the trajectory
planner's own discontinuities. Whether that's sufficient on its own, or whether `kd`
alone is still marginal against the velocity estimator's lag, **has not been tested and
should be treated as an open risk, not a resolved one**, before relying on this
configuration at any real torque/velocity level.

## 11. The damping term also resists *intended* motion — is that recoverable?

Follow-on concern from §10: if feedforward torque is relied on to be accurate (e.g. for
positioning the arm), the same `kd_scale·kd·measured_velocity` damping that's supposed to
only resist disturbances also resists motion caused *deliberately* by that feedforward
torque. The D-term has no way to tell the two apart.

### Why: the reference velocity is degenerate

`error_rate = measured_velocity − desired_rate`, and `desired_rate` is pinned at the
constant `0` (that's what keeps `control_position_raw` frozen and avoids reopening §8's
unresolved trajectory-generator risk). With `desired_rate` always `0`, *any* motion —
wanted or not — looks identical to the firmware: some nonzero `measured_velocity`,
damped proportionally to speed, full stop. There's no signal available to the D-term that
distinguishes "moving because I commanded it" from "moving because something disturbed
it."

### Option A: command a real desired velocity instead of 0 — not recommended yet

Feeding an intended velocity profile into the `velocity` field would make
`error_rate = measured_velocity − desired_velocity`, so the D-term would only react to
*deviation* from the plan rather than fighting all motion. But this is exactly "true
velocity mode" from §8/§10 — it reactivates `UpdateTrajectory`/live `control_position_raw`,
and reopens the still-unverified kd-alone stability question from §10. **Don't reach for
this until that's tested.**

### Option B: keep velocity=0, cancel the damping on the feedforward side instead

`kd_scale·kd·measured_velocity` is fully deterministic and computable ahead of time if the
intended velocity is known (it's your own planned motion). Instead of asking the firmware
to cancel the damping internally, pre-add the expected loss into the feedforward torque
sent from the host:

```
feedforward_torque_to_send = desired_physical_torque + kd_scale · kd · intended_velocity
```

This keeps `velocity=0` in the CAN message — `control_position_raw` stays frozen,
`UpdateTrajectory` never runs, none of §8/§10's unresolved risk gets reopened — while
recovering most of the torque accuracy lost to damping during genuinely intended motion.
During disturbances or transients where actual velocity doesn't match the plan, the
mismatch is only partially compensated — arguably correct behavior, since that residual
*is* the disturbance still worth damping.

**Cost**: needs the actuator's native `kd` (and gear ratio, for joint↔actuator
conversion) known on the host side to compute the compensation — the same requirement §14
below already identified for `kp`/stiffness, just extended to `kd`. If native `kp`/ratio
end up sourced from URDF params per §14's decision, native `kd` should live alongside
them, not as a separate mechanism.

**Not yet decided**: whether to actually implement this compensation, or accept the
simpler "some accuracy loss during intended motion" tradeoff as the cost of the
low-stiffness design. Also not yet worked out: how sensitive the compensation is to
`measured_velocity`'s own estimator lag (§8, `motor_position.h:1238`) — the cancellation
is only as good as how quickly and accurately the firmware's velocity estimate tracks the
actual motion during transients.

## 12. Runtime override architecture (service call) — proposed, not implemented

Idea: expose `kp_scale`/`kd_scale`/`ilimit_scale` as a low-rate override settable via a
ROS service (or dynamic parameter), rather than as full realtime `CommandInterface`s that
some controller would have to publish every cycle — justified because these change
rarely relative to the position/velocity/effort commands, and because §10 above is the
concrete motivating use case: a fixed, permanent `kp_scale=0` in `write_format` can't be
walked back per task/joint if it turns out to be marginal, so it needs to be a live,
tunable value rather than a compile-time constant.

Things settled:
- **Realtime-safety**: a service callback runs on a non-realtime executor thread; the
  handoff into the value(s) `make_cyclic_commands()` reads must be lock-free (e.g.
  `std::atomic<double>` per actuator per scale), never a blocking mutex.
- **Reset-on-mode-change hook**: `perform_command_mode_switch()`
  (`MoteusInterface.cpp:390-408`) is the natural place to reset overrides back to
  defaults — it already fires exactly when ros2_control's controller_manager commits an
  interface claim/release transition, which is the same lifecycle point the
  `ControlMode` (STANDARD/TORQUE_CONTROL) decision in `make_cyclic_commands()` already
  keys off of.

**Not yet decided**: what should scope the reset — any interface change at all, or
specifically the effort-only ⇄ not-effort-only boundary (the same split
`make_cyclic_commands()` already uses for `ControlMode`)? Resetting on every toggle risks
snapping back to full stiffness on an unrelated interface change (e.g. briefly disabling
velocity while staying in the same soft regime).

## 13. Interface-claiming safety gap — found, not yet fixed

Question raised: if a controller claims only velocity+effort (leaving position
unclaimed, to get NaN-position torque/soft control), what stops some *other* controller
from claiming the position interface on that same joint?

**Finding: nothing does, at the ros2_control framework level.** The controller_manager's
conflict check is per-interface, not per-joint — position/velocity/effort are
independent claimable resources. `export_command_interfaces()`
(`MoteusInterface.cpp:293`) always exports all three for every joint regardless of
current intent, so an unclaimed position interface is fair game for any other loaded
controller. If that happens, `apply_interfaces_to_joint_flags()` flips `pos_active_`, and
`make_cyclic_commands()`'s mode decision
(`joint.effort_active_ && !pos_active_ && !vel_active_`) silently reverts to `STANDARD`
— no error, no log, just a quiet loss of the whole soft-mode design.

### Proposed policy (discussed, not implemented)

All-or-nothing claiming per joint: reject any new interface claim if *any* sibling
interface on that joint is currently active and not being dropped in the same
transition; drops are always allowed unconditionally; to gain any interfaces, the caller
must first have the full set free, then claim everything it needs in one atomic call.

- **Fits `prepare_command_mode_switch()` naturally**: it's the veto phase (return
  `ERROR` before anything commits), and `controller_manager::switch_controller()` already
  calls it once with the *complete* start/stop delta for a transition — so "claim
  everything in one call" is already how the framework operates; the new part is just a
  same-joint-conflict check inside it.
- **`prepare_command_mode_switch` only gets interface names, not controller identity** —
  it can't distinguish "a different rogue controller" from "the same controller
  reshaping its own claim." The all-or-nothing framing sidesteps this by applying
  uniformly regardless of who's asking (good — avoids needing identity tracking).
- **Real cost, confirmed, not yet accepted/rejected by the user**: this also blocks the
  *existing* STANDARD ⇄ TORQUE_CONTROL partial-switch pattern (currently
  `apply_interfaces_to_joint_flags()` allows e.g. toggling effort while velocity stays
  claimed). Under strict all-or-nothing, every such transition would need a full
  release-then-reclaim of position+velocity+effort together, introducing a brief
  fully-unclaimed gap (probably harmless — `MakeStop()` already runs for inactive joints
  — but a new transient state that didn't exist before). **Open question: is this
  tightening acceptable, given it changes how the interface's own mode-switch pattern
  works today?**
- **Atomic unit should be the transmission, not a single joint**: `DifferentialTransmission::validate_mode_switch()`
  (`DifferentialTransmission.cpp:49-54`) already requires joint1/joint2 to agree on
  active state. A per-single-joint claiming policy could disagree with that (joint1 free,
  joint2 — its differential partner — not). The new policy's atomic unit needs to be the
  whole transmission (both joints + both actuators), not one joint at a time.

## 14. Physical-stiffness → per-actuator kp_scale conversion — decided: compute it in the transmission layer

Question: if the service call sets an *actual stiffness* (physical units) rather than a
raw scale, and gear ratios/native `kp` differ per actuator, should that conversion happen
inside the interface, or be left to the caller?

**Decided: compute it inside the transmission layer** (extending
`IdentityTransmission`/`DifferentialTransmission` with a stiffness-conversion analogous
to the existing `joint_to_actuator()`/`actuator_to_joint()`), not in the service handler
or left to the caller. Reasoning:

- The whole point of the `Transmission` abstraction is to hide actuator-space mixing
  (gear ratios, the differential's `combine()` math) from anything above joint-space.
  Stiffness is the same class of quantity (a torque-per-unit-error gain) as effort, which
  already goes through this layer every cycle.
- For the differential wrist specifically, a joint-space stiffness request doesn't map to
  one actuator at all: `combine(j1, j2) → a=j1+j2, b=j1-j2` means a pure joint1 stiffness
  request has to become a `kp_scale` on **both** actuators, and the joint1 torque you
  actually get back is `(kp_scale_a·kp_a + kp_scale_b·kp_b)·e_j1` after
  `actuator_to_joint()`'s effort recombination. Pushing this math to the caller means
  duplicating (and risking getting wrong) logic the transmission already owns.

### Constraint that applies regardless of where the math lives

`kp_scale` is Int8-encoded via the `kPwm` mapping, which **saturates at exactly 1.0** —
it can only ever scale a joint's stiffness *down* from whatever the actuator's currently
configured native `kp` provides. A "set an actual stiffness" API needs to either clamp
and report when a request exceeds what's achievable, or rely on native `kp` always being
configured high at commissioning time so runtime requests only ever soften. **Not yet
decided which of these the API should do.**

### Source of native kp / gear ratio: decided — URDF parameters

Considered reading native `kp`/ratio live from the device at `on_configure()` (accurate
to what's actually flashed, but a new config-query code path, and a stale snapshot if
someone re-tunes via `moteus_tool` later without restarting the interface) vs. declaring
them as URDF/transmission parameters, the same pattern already used for `can_id`
(`MoteusInterface.cpp:112`) and `encoder_offset` (`MoteusInterface.cpp:66`).

**Decided: URDF parameters.** Simpler, no new device-read plumbing, consistent with the
existing per-actuator config pattern. Trade-off accepted: this is a second source of
truth that must be kept in sync with whatever's actually configured on the board via
`moteus_tool` — same staleness risk as the device-read option, just relocated to a place
that matches how the rest of this package already handles per-actuator config.

## 15. Open items / not yet decided, going into implementation

- **Whether the kd-alone instability risk from §8 also threatens the §10 "torque + kd
  damping" variant** — reasoned to be structurally the same D-loop regardless of whether
  `desired_rate` is 0 or moving, so not obviously safe just because it looks calmer.
  Never empirically tested. Do this before relying on nonzero `kd_scale` with `kp_scale=0`
  at any real torque level.
- **Whether §8's oscillation was actually caused by mechanism 2 (kd-alone) or mechanism 3
  (integral windup, if `ilimit_scale` was left at its default nonzero value during that
  test)** — not isolated at the time, and it matters for how cautious to be about §10.
- **Whether to implement the feedforward damping-compensation from §11**, and how
  sensitive it ends up being to the velocity estimator's own lag during transients — not
  yet prototyped.
- Reset-on-mode-change scope for runtime overrides (§12): any interface change vs. the
  effort-only/not boundary specifically.
- Whether to accept the all-or-nothing claiming policy's cost to the existing
  STANDARD ⇄ TORQUE_CONTROL partial-switch pattern (§13), or find a narrower policy that
  only blocks *cross-controller* conflicts.
- Whether the stiffness API clamps silently or errors when a requested stiffness exceeds
  what native `kp` can provide (§14).
- Whether native `kd` should be sourced from URDF params alongside native `kp` (§14) to
  support the §11 compensation, if it's implemented.
- Whether `velocity_threshold` is actually configured nonzero on the current hardware —
  check before relying on the deadband explanation in §8.
- The write-format resolution downgrade (§4) and the three scale registers are analysis
  only — no code changes have been made yet.
