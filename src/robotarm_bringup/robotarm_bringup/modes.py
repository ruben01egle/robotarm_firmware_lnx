"""
Operating modes of the arm and how they map onto ros2_control.

The single place that knows which controllers make up a mode. Pure functions without ROS, so the
table, the mode detection and the switch planning are testable on their own. The controller names
must match config/controllers.yaml (checked by test/test_modes.py).
"""

from dataclasses import dataclass

from robotarm_interface.msg import RobotarmMode


@dataclass(frozen=True)
class ModeSpec:
    hardware_active: bool
    controllers: frozenset


@dataclass(frozen=True)
class SwitchPlan:
    activate_hardware: bool      # before the switch
    deactivate: list
    activate: list
    deactivate_hardware: bool    # after the switch


FF = 'ff_controller'
IMPEDANCE = 'cartesian_impedance_controller'

# ff_controller stays active in every position controlled mode, only its upstream changes. Its own
# restart would ramp the feedforward in from 0 again, so hold_controller is the upstream in ARMED.
MODES = {
    RobotarmMode.LIMP: ModeSpec(False, frozenset()),
    # the drives hold on their own PID (moteus_interface idle mode), also the end of a fault
    RobotarmMode.STOPPED: ModeSpec(True, frozenset()),
    RobotarmMode.ARMED: ModeSpec(True, frozenset({FF, 'hold_controller'})),
    RobotarmMode.JOINT_STREAM: ModeSpec(True, frozenset({FF, 'joint_stream_controller'})),
    RobotarmMode.CARTESIAN_JOG: ModeSpec(True, frozenset({FF, 'cartesian_jog_controller'})),
    RobotarmMode.TRAJECTORY: ModeSpec(True, frozenset({FF, 'joint_trajectory_controller'})),
    RobotarmMode.GRAVITY_COMP: ModeSpec(True, frozenset({'gc_controller'})),
    RobotarmMode.IMPEDANCE: ModeSpec(True, frozenset({IMPEDANCE})),
    RobotarmMode.IMPEDANCE_JOINT_STREAM: ModeSpec(
        True, frozenset({IMPEDANCE, 'joint_stream_controller_impedance'})),
    RobotarmMode.IMPEDANCE_JOG: ModeSpec(
        True, frozenset({IMPEDANCE, 'cartesian_jog_controller_impedance'})),
}

# controllers outside of this set (joint_state_broadcaster) are never touched or looked at
MANAGED_CONTROLLERS = frozenset().union(*(spec.controllers for spec in MODES.values()))

MODE_NAMES = {
    value: name for name, value in vars(RobotarmMode).items()
    if name.isupper() and isinstance(value, int)
}


def mode_name(mode):
    return MODE_NAMES.get(mode, f'<invalid {mode}>')


def detect_mode(hardware_state, active_controllers):
    """
    Mode of the actual ros2_control state.

    hardware_state is the lifecycle label of the hardware component, None if it couldn't be read.
    Anything that isn't exactly one mode is FAULT, e.g. the remains of a controller that stopped on
    an error.
    """
    if hardware_state is None:
        return RobotarmMode.UNKNOWN
    if hardware_state == 'finalized':
        return RobotarmMode.EMERGENCY_OFF
    managed = frozenset(active_controllers) & MANAGED_CONTROLLERS
    if hardware_state not in ('active', 'inactive'):
        return RobotarmMode.FAULT
    hardware_active = hardware_state == 'active'
    for mode, spec in MODES.items():
        if spec.hardware_active == hardware_active and spec.controllers == managed:
            return mode
    return RobotarmMode.FAULT


def missing_controllers(loaded_controllers, mode):
    """Return the controllers of a mode that are not loaded and configured, sorted."""
    return sorted(MODES[mode].controllers - frozenset(loaded_controllers))


def available_modes(hardware_state, loaded_controllers):
    """
    Modes a request can switch to now, sorted.

    loaded_controllers are the controllers that are configured (inactive or active). A mode needs
    all of its controllers loaded and the hardware usable (active or inactive). EMERGENCY_OFF is
    available until the hardware is finalized, also when the state can't be read.
    """
    if hardware_state == 'finalized':
        return []
    modes = [RobotarmMode.EMERGENCY_OFF]
    if hardware_state in ('active', 'inactive'):
        modes += [m for m in MODES if not missing_controllers(loaded_controllers, m)]
    return sorted(modes)


def plan_switch(hardware_state, active_controllers, target):
    """
    Plan the steps from the actual state to the target mode, None if it can't be requested.

    All controllers change in one strict switch request, so a mode never exists half switched and
    a controller that stays (ff_controller between position modes) is not restarted. The hardware
    is activated before and deactivated after the switch, controllers only run on active hardware.
    """
    spec = MODES.get(target)
    if spec is None:
        return None
    managed = frozenset(active_controllers) & MANAGED_CONTROLLERS
    hardware_active = hardware_state == 'active'
    return SwitchPlan(
        activate_hardware=spec.hardware_active and not hardware_active,
        deactivate=sorted(managed - spec.controllers),
        activate=sorted(spec.controllers - managed),
        deactivate_hardware=not spec.hardware_active and hardware_active,
    )
