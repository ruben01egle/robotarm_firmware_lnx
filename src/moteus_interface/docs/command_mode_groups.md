# Command-Mode Groups for the Actuator Hardware Interface (ros2_control)

This document describes how the hardware interface enforces **valid combinations of command interfaces per joint**, so that controllers can only claim interface sets that correspond to a real operating mode of the motor driver. It is written as an implementation spec.

## Problem

ros2_control guarantees that each individual command interface is claimed by at most one controller. It does **not** know which *combinations* of interfaces on the same joint are meaningful.

For our actuators, the set of claimed interfaces determines the CAN operating mode sent to the driver:

- A controller that claims only torque relies on the driver being in **pure torque mode**. If another controller additionally claims position or velocity, the driver would switch to a position/velocity mode, and the torque value would silently change its meaning (e.g. become a feedforward term).
- On the other hand, combining position/velocity from one controller with a torque feedforward from another (e.g. gravity compensation) is a legitimate and desired use case.
- Velocity damping in torque mode must run on the motor driver itself (its control loop is much faster than the ros2_control loop), so it has to be a distinct driver mode, not something computed in a controller.

The hardware interface cannot see *which controller* claims an interface. Therefore the **semantics are encoded in the interface names**: an interface name describes what the value does at the actuator, not which controller writes it.

## Command interfaces per joint

Every joint exports six command interfaces, organised in three **mode groups**:

| Group      | Interfaces                            | Driver mode                                        | Rule                               |
|------------|---------------------------------------|----------------------------------------------------|------------------------------------|
| `PV`       | `position`, `velocity`, `torque_ff`   | Position / velocity control with torque feedforward | Any non-empty subset is allowed    |
| `TORQUE`   | `torque`                              | Pure torque control                                | Single member                      |
| `DAMPED`   | `velocity_damped`, `torque_damped`    | Torque control with driver-side velocity damping    | Both interfaces or none            |

Additional rules:

- All active command interfaces of one joint must belong to **the same group**. Mixing groups is always rejected.
- The **empty set** (no command interface claimed) is valid and maps to the driver's idle mode (defined per actuator: hold / freewheel / brake).
- The driver supports every subset of the `PV` group, so no further restriction is needed inside that group.

Example URDF snippet:

```xml
<ros2_control name="actuators" type="system">
  <hardware>
    <plugin>my_package/MyActuatorSystem</plugin>
  </hardware>
  <joint name="joint1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="torque_ff"/>
    <command_interface name="torque"/>
    <command_interface name="velocity_damped"/>
    <command_interface name="torque_damped"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

Controllers select their mode purely by the interface names they claim (most controllers take the interface name as a parameter), for example:

- Trajectory controller: `joint1/position`, `joint1/velocity`
- Gravity compensation: `joint1/torque_ff`
- Pure torque controller: `joint1/torque`
- Damped torque controller: `joint1/torque_damped`, `joint1/velocity_damped`

## Validation logic

Validation is **stateless with respect to switch order**: only the resulting target set of active interfaces per joint is checked. It does not matter in which order controllers were activated.

```cpp
enum class Group { NONE, PV, TORQUE, DAMPED };

Group group_of(const std::string & iface)
{
  if (iface == "position" || iface == "velocity" || iface == "torque_ff") return Group::PV;
  if (iface == "torque") return Group::TORQUE;
  if (iface == "velocity_damped" || iface == "torque_damped") return Group::DAMPED;
  return Group::NONE;  // unknown -> reject
}

bool is_allowed(const std::set<std::string> & ifaces)
{
  if (ifaces.empty()) return true;                 // idle mode
  const Group g = group_of(*ifaces.begin());
  if (g == Group::NONE) return false;
  for (const auto & i : ifaces) {
    if (group_of(i) != g) return false;            // no mixing of groups
  }
  if (g == Group::DAMPED) return ifaces.size() == 2;  // both or none
  return true;                                     // PV: any subset, TORQUE: single
}
```

Switching between groups in a single controller switch (stop controller A, start controller B) is allowed, because the target state is valid.

## Mode switch implementation

### State

- `active_[joint]`: set of currently active interface types (e.g. `{"position", "torque_ff"}`), committed state.
- `pending_[joint]`: target state computed in `prepare_command_mode_switch`, not yet committed.
- `mode_[joint]`: resulting driver mode (`IDLE`, `PV`, `TORQUE`, `DAMPED`), used by `write()`.

On `on_activate()` all joints start with an empty `active_` set and `IDLE` mode.

### `prepare_command_mode_switch(start_interfaces, stop_interfaces)`

Runs in the non-realtime context.

1. **Important:** `start_interfaces` and `stop_interfaces` contain only the interfaces that change in *this* switch, not all currently claimed ones. The full state must be tracked in `active_`.
2. Copy `active_` to a working set.
3. Parse each full interface name (`"<joint>/<interface>"`). Match the joint by known joint names of this component (joint names may contain `/`, so match against the list of joint names rather than splitting naively). Ignore interfaces that do not belong to this hardware component.
4. Remove all `stop_interfaces`, then add all `start_interfaces`.
5. Run `is_allowed()` for every joint. If any joint fails, log an error and return `return_type::ERROR`. The controller manager then rejects the whole switch.
6. On success, store the working set in `pending_` and return `OK`.

Do **not** commit to `active_` here: another hardware component may still reject the switch, in which case `perform_command_mode_switch` is never called. The next `prepare` call simply overwrites `pending_`.

### `perform_command_mode_switch(start_interfaces, stop_interfaces)`

Depending on the ROS 2 distribution this may be called from the **realtime loop**. Check the controller_manager version in use; assume realtime unless verified otherwise.

- No blocking calls, no memory allocation, no direct CAN I/O.
- Commit `pending_` to `active_` and derive `mode_[joint]` from the group of each joint.
- Mark joints whose mode changed, so `write()` sends the CAN mode-change message on the next cycle.
- Reset command values of newly started interfaces to safe defaults to avoid jumps: `position` = current measured position, `velocity` / `torque` / `torque_ff` / `torque_damped` / `velocity_damped` = 0 (or NaN, if `write()` treats NaN as "no command").

### `write()`

- If a joint's mode changed, send the mode-change sequence to the driver first.
- Then send the commands that match `mode_[joint]`. Interfaces that are not active in the current mode are not sent.
- In `IDLE` mode, send the configured idle behaviour.

## Logging

When rejecting a switch, log a clear message including the joint, the requested target set and the reason, e.g.:

```
[MyActuatorSystem] Rejecting mode switch on joint1: target {torque, position}
mixes groups TORQUE and PV.
```

The controller manager only reports that the switch failed, so this message is the main debugging aid.

## Test cases

Each test starts from the given active set and applies a switch.

| Active before         | Start                               | Stop                    | Expected |
|-----------------------|-------------------------------------|-------------------------|----------|
| `{}`                  | `torque`                            |                         | OK       |
| `{torque}`            | `position`                          |                         | ERROR    |
| `{}`                  | `position`, `velocity`              |                         | OK       |
| `{position, velocity}`| `torque_ff`                         |                         | OK       |
| `{}`                  | `torque_ff`                         |                         | OK       |
| `{position}`          | `torque`                            |                         | ERROR    |
| `{}`                  | `torque_damped`                     |                         | ERROR    |
| `{}`                  | `torque_damped`, `velocity_damped`  |                         | OK       |
| `{torque_damped, velocity_damped}` |                        | `velocity_damped`       | ERROR    |
| `{torque_damped, velocity_damped}` |                        | both                    | OK (idle)|
| `{torque}`            | `position`, `velocity`              | `torque`                | OK       |
| `{position, torque_ff}` | `torque_damped`, `velocity_damped` | `position`, `torque_ff` | OK       |
| any                   | interface of another component      |                         | ignored  |

Additionally test that a rejected `prepare` leaves `active_` unchanged, and that a successful `prepare` followed by a new `prepare` (without `perform`) evaluates against the old committed state.