# moteus_interface

A [ros2_control](https://control.ros.org/) `SystemInterface` hardware plugin for [mjbots moteus](https://mjbots.com/products/moteus-r4-11) brushless controllers, communicating over the official [mjbots fdcanusb](https://mjbots.com/products/fdcanusb) USB-to-CAN-FD adapter.

## Table of Contents

- [Overview](#overview)
- [Intended Use / Status](#intended-use--status)
- [Features](#features)
- [Installation / Build](#installation--build)
- [Usage](#usage)
- [Configuration Reference](#configuration-reference)
- [Behavior / How It Works](#behavior--how-it-works)
  - [Configuration and homing sequence](#configuration-and-homing-sequence)
  - [The read/write cycle](#the-readwrite-cycle)
  - [Execution modes: strict_sequential vs. pipelined](#execution-modes-strict_sequential-vs-pipelined)
  - [Reply-latency jitter and the effective frequency ceiling](#reply-latency-jitter-and-the-effective-frequency-ceiling)
  - [Watchdog and fault handling](#watchdog-and-fault-handling)
  - [Non-standard secondary-encoder homing](#non-standard-secondary-encoder-homing)
- [Custom Transport Layer (Experimental)](#custom-transport-layer-experimental)
- [Known Limitations / Roadmap](#known-limitations--roadmap)
- [Contributing](#contributing)
- [License & Acknowledgments](#license--acknowledgments)

## Overview

`moteus_interface` is a `hardware_interface::SystemInterface` plugin that lets [ros2_control](https://control.ros.org/) drive one or more [mjbots moteus](https://github.com/mjbots/moteus) brushless controllers as regular joints, exposing `position`/`velocity`/`effort` command and state interfaces per joint.

It exists because there is no official or widely-adopted ros2_control hardware interface for moteus controllers. This package was built to drive a multi-axis robot arm, using moteus's own C++ client library (`mjbots/moteus`) for constructing and parsing the moteus register-protocol frames, paired with a **from-scratch transport layer** written specifically for this project (see [Custom Transport Layer](#custom-transport-layer-experimental)).

## Intended Use / Status

This is an **experimental, community-maintained package**. It is **not affiliated with, endorsed by, or supported by mjbots**.

Honest state of things:
- It runs the author's own robot arm in day-to-day use, over USB via the official mjbots fdcanusb adapter.
- `package.xml` still has placeholder `TODO` values for the package `<description>` and `<license>` — the licensing situation is explicitly called out in [License & Acknowledgments](#license--acknowledgments).
- Only one transport implementation exists today, and it is written specifically for the official mjbots fdcanusb adapter (see [Custom Transport Layer](#custom-transport-layer-experimental)) — it is not a generic fdcanusb-protocol implementation and has not been tested against other CAN-FD adapters or clones. A `Transport` interface exists so other transports (e.g. native SocketCAN) could be added, but none currently ship in this package.
- The controller-side homing approach used here is a project-specific workaround, not a general recommendation — see [Non-standard secondary-encoder homing](#non-standard-secondary-encoder-homing).

Treat this as a working reference implementation to build on or adapt, not a polished, drop-in library.

## Features

- Implements `hardware_interface::SystemInterface` and is exported as the pluginlib plugin `moteus_interface/MoteusInterface`.
- Per-joint `position` + `velocity` + `effort` command and state interfaces (exactly these three, in this order, are required on every joint).
- Two selectable execution modes trading off latency vs. determinism: `strict_sequential` (blocking write+read each cycle) and `pipelined` (fire-and-forget write, read back on the next cycle).
- Automatic control-mode selection per joint per cycle: standard position/velocity/feed-forward-torque mode, or a torque-only mode (feedforward torque with position-mode PD gains zeroed) when only the effort command interface is claimed.
- A software communication watchdog per joint (consecutive-failure counter + exponential moving-average failure rate) that catches missed/degraded replies at the ros2_control level.
- Optional per-cycle transport timing instrumentation (`TransportTiming`) that can be dumped to CSV for offline latency analysis.
- A one-time, non-standard startup routine that reads a secondary/output encoder register and uses it to directly set the moteus internal absolute position estimate (`MakeOutputExact`), bypassing the standard moteus homing flow — see [Non-standard secondary-encoder homing](#non-standard-secondary-encoder-homing).
- Custom, from-scratch USB transport (`TransportUSB`) built specifically for the official mjbots fdcanusb adapter, designed for more deterministic, real-time-friendly cyclic operation than the stock moteus transport — see [Custom Transport Layer](#custom-transport-layer-experimental).

## Installation / Build

### Dependencies

From `package.xml` / `CMakeLists.txt`:

- ROS 2 (tested against a `rclcpp` / `rclcpp_lifecycle` / `hardware_interface` / `pluginlib` distribution providing `ament_cmake`).
- `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle` (declared `<depend>`s).
- The [mjbots/moteus](https://github.com/mjbots/moteus) C++ client library (`moteus::cpp`), pulled automatically at CMake configure time via `FetchContent` — pinned to commit `d38a25326b47077e0e5645492e9b7cb1b84d5f8e` (tagged `cpp/v1.0.0` in that repo). You do **not** need to install moteus separately; CMake fetches and builds it as part of this package's build.
- Network access at build time (for the `FetchContent` fetch of moteus), unless it's already been fetched into your colcon `build/` directory.

### Hardware

- **The official [mjbots fdcanusb](https://mjbots.com/products/fdcanusb) USB-to-CAN-FD adapter** (default device path `/dev/fdcanusb`). The custom transport in this package (see [Custom Transport Layer](#custom-transport-layer-experimental)) is written and tested specifically against this device — it is not a generic fdcanusb-protocol client and has not been validated against other CAN-FD adapters, clones, or UART-based alternatives.
- Read/write permission on that serial device for the user running `ros2_control_node` (e.g. via a udev rule granting access, or membership in the group that owns the device — mjbots documents this for the fdcanusb; this repository doesn't ship its own udev rule).

### Build

```bash
# from your colcon workspace root
colcon build --packages-select moteus_interface
source install/setup.bash
```

Building `moteus_interface` alone will transitively fetch and build the vendored moteus C++ library the first time.

## Usage

Declare the hardware plugin inside a `<ros2_control>` tag in your URDF/xacro. Each joint needs exactly three command interfaces (`position`, `velocity`, `effort`) and three state interfaces (`position`, `velocity`, `effort`), plus the `encoder_offset` parameter; each joint is paired with one (or, for a differential joint pair, two) actuator(s) via a `<transmission>`, which is where `can_id` is declared per actuator (and, for a `differential` transmission, `joint1_encoder_actuator`/`joint2_encoder_actuator` — see [Configuration Reference](#configuration-reference)):

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="my_arm_ros2_control" params="name use_hardware:=false">
    <ros2_control name="${name}" type="system">
      <hardware>
        <xacro:if value="${use_hardware}">
          <plugin>moteus_interface/MoteusInterface</plugin>
        </xacro:if>
        <xacro:unless value="${use_hardware}">
          <plugin>mock_components/GenericSystem</plugin>
          <param name="mock_interfaces">true</param>
        </xacro:unless>
      </hardware>

      <joint name="axis1">
        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <command_interface name="effort"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
        <param name="encoder_offset">0.0</param>
      </joint>

      <transmission name="axis1_transmission">
        <plugin>identity</plugin>
        <joint name="axis1"/>
        <actuator name="axis1_motor"/>
        <param name="axis1_motor.can_id">1</param>
      </transmission>

      <!-- additional joints follow the same pattern, one per moteus controller -->
    </ros2_control>
  </xacro:macro>
</robot>
```

Pairing `<xacro:if value="${use_hardware}">` with a `mock_components/GenericSystem` fallback (as shown above) lets you develop/test against simulated joints without real hardware attached, and switch to real hardware with a single xacro argument.

Controller-manager YAML (trimmed and generalized from a real working configuration):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500
    lock_memory: true
    thread_priority: 99

    hardware_components_initial_state:
      inactive:
        - moteus_hardware_system

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    # ... your motion/trajectory controller(s) here ...

moteus_hardware_system:
  ros__parameters:
    execution_mode: "pipelined"       # "pipelined" or "strict_sequential"
    timeout_transport_us: 1000
    time_transport: false

    transport:
      usb_device: "/dev/fdcanusb"

joint_state_broadcaster:
  ros__parameters:
    publish_rate: 50
```

Launch the usual way, e.g.:

```bash
ros2 launch controller_manager ros2_control_node --ros-args \
  --params-file /path/to/controllers.yaml

# or, from a launch file that starts ros2_control_node with your robot_description
# and controller config, then spawns controllers:
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner your_controller
```

The hardware component name in your controller YAML (`moteus_hardware_system` above) must match the `name` given to the `<ros2_control>` tag in your URDF/xacro.

## Configuration Reference

### Per-joint URDF/xacro `<param>`s

| Name | Type | Default | Required | Meaning |
|---|---|---|:---:|---|
| `encoder_offset` | double (radians) | — | **Yes** | Field-calibration trim, in joint space: if the measured joint angle is off by some amount, enter the correction here rather than re-homing hardware directly. Applied when re-zeroing the controller(s)' internal absolute position from the secondary encoder register(s) at `on_configure()`. See [Non-standard secondary-encoder homing](#non-standard-secondary-encoder-homing). |

Read in `on_init()`; the hardware component fails to initialize (`CallbackReturn::ERROR`) if this is missing on any joint.

### Per-actuator URDF/xacro `<transmission>` `<param>`s

| Name | Type | Default | Required | Meaning |
|---|---|---|:---:|---|
| `<actuator_name>.can_id` | int | — | **Yes** | The moteus controller's CAN arbitration ID for this actuator. Used to route outgoing command frames and to match incoming reply frames back to the actuator by their CAN source address. |

Read in `on_init()` from each `<transmission>`'s parameters, one per `<actuator>` it declares; the hardware component fails to initialize if any actuator is missing its `can_id`.

### Per-`differential`-transmission URDF/xacro `<param>`s

| Name | Type | Default | Required | Meaning |
|---|---|---|:---:|---|
| `joint1_encoder_actuator` | string | — | **Yes** (differential only) | Names which of this transmission's two `<actuator>`s the first joint's dedicated absolute secondary encoder is physically wired through. |
| `joint2_encoder_actuator` | string | — | **Yes** (differential only) | Names which of this transmission's two `<actuator>`s the second joint's dedicated absolute secondary encoder is physically wired through. Must name the transmission's *other* actuator from `joint1_encoder_actuator`. |

Only relevant for a `differential` transmission, where either joint's dedicated encoder could physically be wired to either actuator's board and the mapping can't be derived structurally. An `identity` transmission has exactly one joint and one actuator, so there's nothing to disambiguate and no param is needed. Read in `on_init()` from the `<transmission>`'s own parameters (not from either `<joint>`); the hardware component fails to initialize if either is missing on a `differential` transmission, or if together they don't name that transmission's two actuators one each.

### Node ROS parameters (declared in `on_configure()`)

| Name | Type | Default | Required | Meaning |
|---|---|---|:---:|---|
| `execution_mode` | string | `"auto"` | No | `"auto"` or `"strict_sequential"` selects blocking write-then-read-per-cycle behavior. `"pipelined"` selects fire-and-forget writes with replies collected on the following cycle. Any other value fails configuration. |
| `time_transport` | bool | `false` | No | Enables `TransportTiming` instrumentation on `on_activate()`; a CSV timing log is written on `on_deactivate()`. |
| `timeout_transport_us` | uint32 | `0` | Conditionally | Read/cycle timeout budget in microseconds. **Must be explicitly set to a non-zero value when `execution_mode` is `strict_sequential`** (configuration fails otherwise). In `pipelined` mode it may be left at `0`, but the code logs a warning that this can be problematic depending on the transport. |

### Transport parameters (declared by `TransportUSB` in `on_configure()`)

| Name | Type | Default | Required | Meaning |
|---|---|---|:---:|---|
| `transport.usb_device` | string | `"/dev/fdcanusb"` | No | Path to the serial device backing the fdcanusb-protocol USB-CAN adapter. |

> Only `can_id`, `encoder_offset`, and (for `differential` transmissions) `joint1_encoder_actuator`/`joint2_encoder_actuator` are verified as strictly required — everything else has a code-level default. All defaults above are taken directly from `MoteusInterface::read_ros_parameters()` and `TransportUSB::declare_and_read_parameters()`.

## Behavior / How It Works

### Configuration and homing sequence

On `on_configure()`, the interface:

1. Reads the ROS parameters above and instantiates `TransportUSB`, then calls its `declare_and_read_parameters()` and `initialize()` (which opens and configures the serial device: raw/non-canonical mode, low-latency ASYNC flag, no flow control).
2. Builds fixed wire formats: commands are sent as `position`/`velocity`/`feedforward_torque` floats (everything else on the command side — kp/kd scale, torque/velocity/accel limits, voltage/current overrides, position bounds — is left at the controller's own defaults and not overridden per cycle). Query replies are read back as `int16` position/velocity/torque plus an `int8` fault code; everything else is ignored, except one extra register added specifically for the startup homing step: `kEncoder1Position` (secondary/output encoder), read as a float.
3. Sends an initial `MakeStop()` to every joint (with the extra encoder register attached) via a blocking `cycle()` (10 ms timeout) to get a first telemetry snapshot, then runs the watchdog in **strict** mode — any joint that doesn't answer or that reports a fault aborts configuration.
4. For each actuator, reads back its raw `kEncoder1Position` extra register value and stages it; once every actuator has a value staged, each transmission's `home()` picks out the raw reading(s) belonging to its own joint(s), offset-corrects them (`remainder(encoder_value - encoder_offset/2π, 1.0)`) and combines them into actuator-space command(s), and `MakeOutputExact` is issued per actuator to directly overwrite that moteus controller's internal absolute output position — this is the non-standard homing step described in detail below.
5. Cycles those `OutputExact` commands (10 ms timeout), re-checks the watchdog in strict mode.
6. Sends a final `MakeStop()` to every joint before returning success, so the hardware is guaranteed to be in a safe, stopped state before the first `read()`/`write()` call from the controller manager. In `pipelined` mode this is a fire-and-forget write followed by a 5 ms sleep (to make sure a reply is buffered before the first `read()`); in `strict_sequential` mode it's a blocking 5 ms-timeout cycle.

If anything in this sequence throws or fails, `on_configure()` returns `CallbackReturn::ERROR` and the hardware component does not become usable.

### The read/write cycle

Each joint tracks which of its three command interfaces (`position`, `velocity`, `effort`) is currently claimed by an active controller (`perform_command_mode_switch()`). Every cycle, `make_cyclic_commands()`:

- Rotates a per-cycle `send_order_` array by one position, so the order in which joints' frames are placed in the outgoing buffer shifts every cycle rather than always sending the same joint first — this exists specifically to spread out reply-timeout risk evenly across joints; see [Reply-latency jitter and the effective frequency ceiling](#reply-latency-jitter-and-the-effective-frequency-ceiling).
- Picks a control mode per joint:
  - **Standard mode** (default, or whenever position/velocity is active): sends `position`, `velocity`, and `feedforward_torque` as a `PositionMode::Command`, in each case falling back to "inactive"/neutral values (`NaN` position, `0` velocity, `0` torque) for any interface not currently claimed or whose command value is `NaN`. Position and velocity are converted from radians / rad·s⁻¹ to the motor revolutions / rev·s⁻¹ units moteus expects (`value / 2π`).
  - **Torque-only mode** (only when the `effort` interface is active and neither `position` nor `velocity` is): still sent as a `PositionMode::Command`, but with position/velocity ignored on the wire and `kp_scale`/`kd_scale`/`ilimit_scale` all forced to `0` — i.e. the position-mode PD/I gains are zeroed out so only the feedforward torque term drives the motor.

**strict_sequential**: `write()` builds commands and performs a single blocking `transport_->cycle()` (write, then block for replies up to `timeout_transport_us`); `read()` then just parses whatever `cycle()` already collected.

**pipelined**: command building and the transport write happen inside `read()` (fire-and-forget `transport_->write()`, immediately followed on the *next* `read()` call by `transport_->read()` collecting the replies from the previous cycle's write, bounded by a fixed 200 µs internal timeout, `pipelined_read_timeout_us`). `write()` in this mode is a no-op.

That 200 µs figure isn't a real wait budget in the sense of "how long a reply is allowed to take" — replies should already be sitting in the OS's read buffer by the time this `read()` runs, since they were sent in response to the *previous* cycle's write. It exists to let the cycle slightly overshoot: at typical update rates there's usually some slack left in a cycle's period, and that slack can absorb an occasionally-late reply (see [Reply-latency jitter and the effective frequency ceiling](#reply-latency-jitter-and-the-effective-frequency-ceiling)) and let the loop catch back up on the next iteration, rather than dropping the reply outright the instant the nominal period elapses. It's an empirically chosen constant based on commonly observed cycle timings, not a value derived from any particular update rate or hardware configuration — it may need tuning for setups with different timing characteristics.

While the hardware component is loaded but not yet **active**, `read()` still runs every cycle and repeatedly sends `MakeStop()` to every joint (instead of live commands), so an inactive-but-configured interface keeps the hardware safely stopped.

### Execution modes: strict_sequential vs. pipelined

| Mode | Write | Read | Timeout behavior |
|---|---|---|---|
| `strict_sequential` (default / `"auto"`) | Blocking write, waits for all replies before returning | Reads whatever the preceding blocking cycle already collected | `timeout_transport_us` **must** be set explicitly (>0); the full write+read cycle is bounded by it |
| `pipelined` | Fire-and-forget; does not wait for replies | Reads back the previous cycle's replies, bounded by a fixed ~200 µs overshoot allowance | `timeout_transport_us` may be `0`, with a logged warning that this can be problematic depending on the transport |

`strict_sequential` is simpler and deterministic per-cycle but pays the full round-trip latency inline every cycle. `pipelined` overlaps the round-trip with other work at the cost of state (commands and telemetry are always one cycle removed from each other) and requires care around startup ordering (hence the extra flush/sleep at the end of `on_configure()`).

### Reply-latency jitter and the effective frequency ceiling

The moteus controller runs on a single-core MCU that has to service its own control-loop interrupt (running at its configured PWM/control frequency) alongside CAN-FD frame handling. CAN traffic only gets processed with whatever CPU headroom is left over from that interrupt load, so reply latency isn't constant: a controller that normally replies in roughly 200–300 µs will occasionally take 600–800 µs if it happens to be busy servicing its own control loop right when a command arrives. This is a property of the moteus controller itself, not of this package's transport.

On a multi-joint bus, whichever joint's command is sent **last** in a given cycle has the least slack left before the shared read timeout — it's the one most likely to occasionally miss the reply window and register as a dropped reply that cycle. The `send_order_` rotation described above exists specifically to counter this: instead of always disadvantaging the same joint, the "sent last, least slack" position moves around the joint list every cycle, distributing this random tail latency evenly across all joints rather than concentrating it on one.

This is also why the software watchdog below tolerates a short run of misses rather than failing on the very first one: an isolated single-cycle miss caused by this kind of scheduling jitter on the controller side is expected and not itself a sign of a real communication problem — only a sustained or repeated pattern of it is.

**Practical implication:** today, the effective frequency ceiling for a given setup is generally set by the moteus controllers' own reply-latency jitter under their control-loop interrupt load, not by this package's transport layer. Pushing `update_rate` and/or tightening `timeout_transport_us` too close to a controller's typical reply time makes these jitter-driven dropouts increasingly frequent. One way to claw back CPU headroom on the controller side is to lower the moteus PWM frequency (which is also its control-loop frequency) — fewer interrupts per second leaves more CPU time free to service CAN traffic promptly, at the cost of that controller's own control-loop bandwidth.

### Watchdog and fault handling

This package implements two independent layers of fault detection at the ros2_control level (the moteus controllers also run their own firmware-side command watchdog, left at its default configuration and not touched by this plugin):

1. **Immediate hardware fault check** (`read()`): if a joint's parsed `Query::Result.fault` code is non-zero, `read()` immediately logs `RCLCPP_FATAL` and returns `hardware_interface::return_type::ERROR` for that cycle — no filtering or grace period.
2. **Software communication watchdog** (`MoteusInterface::watchdog()` / `Joint::update_status()`), run once per `read()` call for every joint:
   - Each joint tracks `consecutive_failures_` (reset to 0 on any successful reply, incremented on any missed reply) and an exponentially-weighted `failure_rate_` (`α = 0.01`, i.e. roughly a 100-cycle time constant), where each missed reply counts as `1.0` and each success as `0.0` in the filter.
   - A joint is considered to be **in an error state** if `consecutive_failures_ >= 3` **or** the filtered `failure_rate_ > 0.10` (10%).
   - In *strict* mode (used only during `on_configure()`), a single missed reply from any joint is fatal and aborts configuration.
   - In normal (non-strict) operation, a single missed reply is only logged as a warning, but the error-state thresholds above still apply — occasional missed replies are tolerated, sustained or bursty communication loss is not, and either condition returns `hardware_interface::return_type::ERROR` from `read()`.

On any `ERROR` return, or on `on_shutdown()`/`on_error()`, the interface calls into `on_deactivate()`, which sends a final `MakeStop()` to every joint as an emergency stop before giving up control.

### Non-standard secondary-encoder homing

> **This is a project-specific workaround, not a general recommendation.** If your setup uses a standard moteus homing flow (index search, absolute encoder wired as the primary commutation encoder, `d rezero`, etc.), you can remove this step entirely and go straight from the initial `MakeStop()` telemetry cycle to normal operation.

Because of restrictions in the author's hardware/mounting setup, this interface does not rely on the moteus controller's own built-in homing procedure. Each joint has its own dedicated absolute secondary encoder, wired into one of its actuator's moteus boards — not into the motor's own commutation encoder — so correcting a joint's measured position is a matter of entering a plain joint-space trim (`encoder_offset`), not reasoning about individual motors. Which actuator's board a given joint's encoder is wired through only matters for a `differential` transmission (see `joint1_encoder_actuator`/`joint2_encoder_actuator` in [Configuration Reference](#configuration-reference)) — a 1:1 (`identity`) joint has only one actuator to begin with, so there's nothing to name. Instead, during every `on_configure()` call, it:

1. Queries an **extra register**, `kEncoder1Position` — moteus's secondary/output encoder position — by adding it to the `Query::Format` as an extra float field on the initial `MakeStop()` telemetry request, for every actuator.
2. For every actuator, reads the raw value back out of its own reply `extra[]` array (failing configuration outright if the register isn't present in the reply, which would indicate a query-format mismatch) and stages it. No offset correction or joint-space math happens yet — this step is purely per-actuator.
3. Each transmission's `home()` then picks out, from the actuators it owns, whichever raw reading(s) belong to its own joint(s) — trivial for a 1:1 (`identity`) joint (its one actuator), or resolved via `joint1_encoder_actuator`/`joint2_encoder_actuator` for a differential pair — offset-corrects each with `remainder(encoder1_value - encoder_offset / (2π), 1.0)`, and combines them into the corresponding actuator-space value(s): a straight passthrough for `identity`, or the same sum/difference relationship `joint_to_actuator()` already uses for a differential pair.
4. Sends each resulting actuator-space value via `MakeOutputExact`, which tells that moteus controller to directly overwrite its internal absolute output position estimate with this value, rather than deriving it from an index search or the primary encoder's own calibrated absolute reading.

This happens on *every* `on_configure()` — not just once at first power-up — so every joint's secondary encoder must be an absolute (not incremental) sensor for this to make sense: the interface re-derives absolute position from it every time the hardware component configures, rather than trusting the moteus controller(s) to retain or recompute it independently.

If you don't need this — e.g. you have a standard absolute commutation encoder setup that moteus can home on its own — this whole step (and the `kEncoder1Position` extra-register plumbing around it) can simply be deleted from `on_configure()`, leaving the ordinary post-`MakeStop()` configuration flow.

## Custom Transport Layer (Experimental)

> **Experimental.** `TransportUSB` is a complete, from-scratch rewrite of the transport layer — it does not share code, architecture, or ongoing maintenance with the upstream moteus C++ transport. Do not expect a line-level diff between the two to reveal much; that's expected. It is also written specifically and only for the **official mjbots fdcanusb adapter** — it is not a general-purpose fdcanusb-protocol implementation, and other CAN-FD adapters, clones, or UART-mode devices are neither supported nor tested.

### Why rewrite it

The stock `mjbots/moteus` C++ transport (`lib/cpp/mjbots/moteus/moteus_transport.h`) is designed around a **background event-loop thread**: every `Transport` implementation (`Fdcanusb`, `Socketcan`) owns a `ThreadedEventLoop` that runs on its own `std::thread`, and `Cycle()`/`BlockingCycle()` post work onto that thread and block a caller-side condition variable waiting for a callback. Timing is governed by a set of independent wait budgets (`min_ok_wait_ns`, `min_rcv_wait_ns`, `rx_extra_wait_ns`, `final_wait_ns`) that accumulate and extend dynamically as data trickles in.

That model is well suited to general-purpose, potentially multi-transport, asynchronous use (the reference `tview`/`moteus_tool` tooling, arbitrary numbers of devices, mixed transports) but is a poor fit for a single ros2_control cycle running on its own real-time-managed thread: an extra background thread, a mutex/condition-variable handoff, and dynamically-extending wait windows all add scheduling and latency variance that a periodic RT-managed control loop would rather avoid.

`TransportUSB` in this package instead:

- Runs entirely **synchronously on the calling thread** — no background thread, no condition variables, no `Post()`/callback machinery. `write()`, `read()`, and `cycle()` are plain blocking calls invoked directly from `read()`/`write()` in the hardware interface, which is itself already being driven by ros2_control's own real-time-aware update loop.
- Uses a **single, caller-supplied timeout budget** per `read()`/`cycle()` call (`timeout_us`) rather than the stock transport's multiple independently-extending wait windows — the caller (the hardware interface) decides up front how much time a cycle is allowed to take, which maps naturally onto a fixed-rate control loop's period budget.
- Uses **fixed-size, pre-allocated `char` buffers** (4 KB TX/RX) and byte-level hex encode/decode helpers (a precomputed hex lookup table) instead of `snprintf`-based formatting, avoiding per-frame heap allocation and `printf`-family overhead on the hot path.
- Sets `ASYNC_LOW_LATENCY` on the serial port and switches it into raw/non-canonical mode explicitly, and opens it non-blocking, polling with `ppoll()` against a single deadline rather than the layered-timeout retry model above.
- Adds first-class **timing instrumentation** (`TransportTiming`) as a plain, always-present member — a zero-cost-when-disabled set of `clock_gettime(CLOCK_MONOTONIC_RAW)` markers around encode / write-syscall / round-trip / drain phases of each cycle, dumpable to CSV — which the stock transport has no equivalent of.

### What carried over / what's different

| Aspect | Stock `mjbots::moteus` transport | This package's `TransportUSB` |
|---|---|---|
| Threading | Dedicated background `ThreadedEventLoop` thread per transport, callback-based completion | Fully synchronous, runs on the caller's thread |
| Timing model | Multiple independently-extending wait windows (`min_ok_wait_ns`, `rx_extra_wait_ns`, `final_wait_ns`, …) | One fixed timeout per `read()`/`cycle()` call, deadline computed once up front |
| Retry / UART support | Built-in per-frame retry logic, CRC-8 checksums, UART auto-detect vs. fdcanusb | None — no retries, no checksums, no UART fallback; a single always-fdcanusb-protocol code path |
| Async API | `Cycle()` (async, callback) + `BlockingCycle()` (blocking wrapper) | Only a blocking API: `write()`, `read()`, `cycle()` |
| Transport selection | `TransportFactory`/`TransportRegistry` auto-detects fdcanusb vs. SocketCAN vs. UART at runtime | Single hardcoded transport (`TransportUSB`) instantiated directly by `MoteusInterface`; a `Transport` base class exists for future alternatives but none ship today |
| Line/frame parsing | `Tokenizer`-based line parsing, checksum validation, `OK`/`ERR` handling for UART retry logic | Minimal `rcv <id> <data> [flags]` line parser only; no `OK`/`ERR` handling, no checksums |
| Wire protocol on the bus | fdcanusb ASCII protocol (`can send <id> <hexdata> [B/b] [F/f]` / `rcv <id> <hexdata> [flags]`) | Same fdcanusb ASCII protocol, targeting only the official mjbots fdcanusb device — this part is intentionally compatible |
| Instrumentation | None built in | `TransportTiming`: per-stage microsecond timing, optional, dumpable to CSV |

The one thing that is **not** different is the wire format: `TransportUSB` speaks the same fdcanusb line protocol as the stock transport (`can send <hex-id> <hex-payload> [B|b] [F|f]` out, `rcv <hex-id> <hex-payload> [flags]` in), including the same DLC round-up padding scheme (`0x50` padding bytes) — so it talks to the same official fdcanusb hardware, just without going through mjbots's own client-side transport code to do it.

### Divergence from upstream

`TransportUSB` was originally forked from, and inspired by, the moteus firmware/transport code around the **v1.0** era (the vendored client library in this package's `CMakeLists.txt` is pinned to commit `d38a25326b47077e0e5645492e9b7cb1b84d5f8e`, tagged `cpp/v1.0.0`). Since then it has diverged completely and is maintained **independently** — it does **not** track upstream moteus transport changes, bug fixes, or protocol extensions going forward.

Implications:

- If a future moteus firmware or fdcanusb protocol revision changes the ASCII line format, checksum requirements, or adds new flags, this transport will need to be updated by hand — it will not pick up such changes by bumping the vendored moteus dependency.
- Only the register/command-construction side of the vendored moteus library is used here (`Controller::Make*`, `Query::Parse`, register/format definitions) — never its `Transport`/`Fdcanusb`/`Socketcan` classes. Bumping the `FetchContent` pin to a newer moteus release only affects the register protocol layer, not this transport.
- Only the official mjbots fdcanusb adapter is supported today; there is no SocketCAN (native Linux CAN interface) implementation in this package, unlike the stock library, and no support for other USB-CAN adapters or clones.

## Known Limitations / Roadmap

- `package.xml` still carries placeholder `TODO` text for `<description>` and `<license>`.
- Only the official mjbots fdcanusb adapter is supported; no SocketCAN transport, and no support for other USB-CAN adapters.
- The practical frequency ceiling today comes from the moteus controllers' own reply-latency jitter under their control-loop interrupt load (see [Reply-latency jitter and the effective frequency ceiling](#reply-latency-jitter-and-the-effective-frequency-ceiling)), not from this package's transport — running near that edge increases sporadic, watchdog-tolerated reply dropouts.
- The secondary-encoder homing step in `on_configure()` is a setup-specific workaround (see above), not a general moteus homing solution — anyone without the author's aux-encoder wiring should remove it.
- The custom transport has no retry logic and no checksum support — it assumes a clean link to the official fdcanusb device.
- The transport does not track upstream moteus protocol/firmware changes going forward (see [Divergence from upstream](#divergence-from-upstream)).

## Contributing

This package is shared in case it's useful to others in the mjbots/moteus and ros2_control communities — issues, questions, and pull requests are welcome. If you run into a problem, please include your moteus firmware version, the transport/execution-mode configuration you're using, and (if relevant) a `time_transport`-enabled timing CSV.

## License & Acknowledgments

**This package does not currently have a license file.** `package.xml` still declares a placeholder (`TODO: License declaration`) license, and no `LICENSE` file exists in this repository at the time of writing — until that's resolved, treat the code as all-rights-reserved by default.

This package depends on and is built against the [mjbots/moteus](https://github.com/mjbots/moteus) project's C++ client library, which is licensed under the [Apache License 2.0](https://github.com/mjbots/moteus/blob/main/LICENSE), © mjbots Robotic Systems, LLC. This package uses that library's register/command-construction and parsing code (`Controller`, `Query`, protocol/register definitions); the transport layer described above is an independent rewrite and is not derived from mjbots's transport implementation. moteus is a product of mjbots Robotic Systems, LLC — this package is an independent, unofficial integration and is not affiliated with or endorsed by mjbots.
