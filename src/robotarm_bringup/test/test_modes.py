"""Mode table, mode detection and switch planning of robotarm_bringup.modes."""

import os

import pytest
from robotarm_bringup.modes import available_modes
from robotarm_bringup.modes import detect_mode
from robotarm_bringup.modes import MANAGED_CONTROLLERS
from robotarm_bringup.modes import missing_controllers
from robotarm_bringup.modes import MODES
from robotarm_bringup.modes import plan_switch
from robotarm_interface.msg import RobotarmMode as M
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'controllers.yaml')
LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'controllers.launch.py')


def active_of(mode):
    """Hardware state and active controllers of a mode, plus the broadcaster that is always on."""
    spec = MODES[mode]
    return ('active' if spec.hardware_active else 'inactive',
            set(spec.controllers) | {'joint_state_broadcaster'})


# ---------------------------------------------------------------------------------------------
# table vs. config


def test_every_managed_controller_is_configured_and_spawned():
    with open(CONFIG) as f:
        configured = yaml.safe_load(f)['controller_manager']['ros__parameters']
    with open(LAUNCH) as f:
        launch = f.read()
    for name in MANAGED_CONTROLLERS:
        assert name in configured, f'{name} missing in controllers.yaml'
        assert f"'{name}'" in launch, f'{name} not spawned in controllers.launch.py'


def test_modes_are_distinguishable():
    seen = {}
    for mode, spec in MODES.items():
        key = (spec.hardware_active, spec.controllers)
        assert key not in seen, f'{mode} and {seen[key]} are the same state'
        seen[key] = mode


def test_modes_without_a_controller_set_are_not_planned():
    # FAULT and UNKNOWN are reported only, EMERGENCY_OFF is handled by the node itself
    for mode in (M.FAULT, M.UNKNOWN, M.EMERGENCY_OFF):
        assert mode not in MODES
        assert plan_switch('active', set(), mode) is None


# ---------------------------------------------------------------------------------------------
# detection


@pytest.mark.parametrize('mode', list(MODES))
def test_every_mode_is_detected(mode):
    assert detect_mode(*active_of(mode)) == mode


def test_unreadable_state_is_unknown():
    assert detect_mode(None, set()) == M.UNKNOWN


def test_hardware_active_without_controllers_is_stopped():
    # the drives hold on their own PID without feedforward, also the end of a fault cleanup
    assert detect_mode('active', {'joint_state_broadcaster'}) == M.STOPPED


def test_remains_of_a_stopped_chain_are_fault():
    # ff_controller stopped on an error, its upstream is left over
    assert detect_mode('active', {'cartesian_jog_controller'}) == M.FAULT


def test_hardware_in_error_state_is_fault():
    assert detect_mode('unconfigured', set()) == M.FAULT


def test_finalized_hardware_is_emergency_off():
    # also with controllers left active, the drives are stopped either way
    assert detect_mode('finalized', set()) == M.EMERGENCY_OFF
    assert detect_mode('finalized', MODES[M.ARMED].controllers) == M.EMERGENCY_OFF


def test_unmanaged_controllers_are_ignored():
    hardware_state, active = active_of(M.ARMED)
    assert detect_mode(hardware_state, active | {'some_broadcaster'}) == M.ARMED


# ---------------------------------------------------------------------------------------------
# availability


def test_all_modes_available_with_all_controllers_loaded():
    expected = sorted(list(MODES) + [M.EMERGENCY_OFF])
    assert available_modes('inactive', MANAGED_CONTROLLERS) == expected
    assert available_modes('active', MANAGED_CONTROLLERS) == expected


def test_mode_unavailable_while_a_controller_is_not_loaded():
    loaded = MANAGED_CONTROLLERS - {'joint_trajectory_controller'}
    available = available_modes('active', loaded)
    assert M.TRAJECTORY not in available
    assert M.ARMED in available and M.LIMP in available
    assert missing_controllers(loaded, M.TRAJECTORY) == ['joint_trajectory_controller']


def test_ff_missing_disables_all_position_modes():
    available = available_modes('active', MANAGED_CONTROLLERS - {'ff_controller'})
    for mode in (M.ARMED, M.JOINT_STREAM, M.CARTESIAN_JOG, M.TRAJECTORY):
        assert mode not in available
    assert M.GRAVITY_COMP in available


def test_only_emergency_off_without_usable_hardware():
    assert available_modes(None, set()) == [M.EMERGENCY_OFF]
    assert available_modes('unconfigured', MANAGED_CONTROLLERS) == [M.EMERGENCY_OFF]


def test_nothing_available_after_emergency_off():
    assert available_modes('finalized', MANAGED_CONTROLLERS) == []


def test_stop_from_motion_deactivates_everything_keeps_power():
    plan = plan_switch(*active_of(M.CARTESIAN_JOG), M.STOPPED)
    assert plan.deactivate == ['cartesian_jog_controller', 'ff_controller']
    assert plan.activate == []
    assert not plan.activate_hardware and not plan.deactivate_hardware


# ---------------------------------------------------------------------------------------------
# planning


def test_arm_from_limp_activates_hardware_before_ff_and_hold():
    plan = plan_switch(*active_of(M.LIMP), M.ARMED)
    assert plan.activate_hardware and not plan.deactivate_hardware
    assert plan.deactivate == []
    assert plan.activate == ['ff_controller', 'hold_controller']


@pytest.mark.parametrize('a', [M.ARMED, M.JOINT_STREAM, M.CARTESIAN_JOG, M.TRAJECTORY])
@pytest.mark.parametrize('b', [M.ARMED, M.JOINT_STREAM, M.CARTESIAN_JOG, M.TRAJECTORY])
def test_position_modes_only_swap_the_upstream(a, b):
    # ff_controller must never be in a switch between position modes, a restart ramps it from 0
    plan = plan_switch(*active_of(a), b)
    assert 'ff_controller' not in plan.activate + plan.deactivate
    assert len(plan.activate) == len(plan.deactivate) == (0 if a == b else 1)
    assert not plan.activate_hardware and not plan.deactivate_hardware


def test_position_to_effort_switches_in_one_request():
    plan = plan_switch(*active_of(M.CARTESIAN_JOG), M.IMPEDANCE_JOG)
    assert plan.deactivate == ['cartesian_jog_controller', 'ff_controller']
    assert plan.activate == [
        'cartesian_impedance_controller', 'cartesian_jog_controller_impedance']


def test_limp_deactivates_everything_then_the_hardware():
    plan = plan_switch(*active_of(M.TRAJECTORY), M.LIMP)
    assert plan.deactivate == ['ff_controller', 'joint_trajectory_controller']
    assert plan.activate == []
    assert plan.deactivate_hardware and not plan.activate_hardware


def test_broadcaster_is_never_touched():
    for a in MODES:
        for b in MODES:
            plan = plan_switch(*active_of(a), b)
            assert 'joint_state_broadcaster' not in plan.activate + plan.deactivate


def test_fault_recovers_to_any_mode():
    # leftovers of a stopped chain are deactivated in the same request
    plan = plan_switch('active', {'cartesian_jog_controller'}, M.ARMED)
    assert plan.deactivate == ['cartesian_jog_controller']
    assert plan.activate == ['ff_controller', 'hold_controller']
