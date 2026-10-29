"""
Mode manager: the only client of the controller manager's switch services.

Clients request a mode (~/set_mode, robotarm_interface/srv/SetMode) and read the actual mode, the
modes available now and whether a switch is running (~/state, robotarm_interface/msg/ModeState,
latched). Which controllers and hardware states make up a mode is defined in modes.py.

The actual state is polled from the controller manager, so ~/state also reflects changes nobody
requested. A state that is no mode (a controller stopped on an error) is reported as FAULT and the
remaining managed controllers are deactivated, which is STOPPED: the drives hold on their own PID.

EMERGENCY_OFF finalizes the hardware (the drives are powered off) and is never rejected because of
a running switch, it aborts it. Afterwards every request is rejected, only a restart recovers.
"""

import threading

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import ListHardwareComponents
from controller_manager_msgs.srv import SetHardwareComponentState
from controller_manager_msgs.srv import SwitchController
from lifecycle_msgs.msg import State
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.task import Future
from robotarm_bringup.modes import available_modes
from robotarm_bringup.modes import detect_mode
from robotarm_bringup.modes import FF
from robotarm_bringup.modes import missing_controllers
from robotarm_bringup.modes import mode_name
from robotarm_bringup.modes import MODES
from robotarm_bringup.modes import plan_switch
from robotarm_interface.msg import ModeState
from robotarm_interface.msg import RobotarmMode
from robotarm_interface.srv import SetMode


class ModeError(Exception):
    pass


class ModeManager(Node):

    def __init__(self):
        super().__init__('mode_manager')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('hardware_component', 'moteus_hardware_system')
        self.declare_parameter('poll_period', 0.2)       # s, until an unrequested change is seen
        self.declare_parameter('service_timeout', 5.0)   # s, per controller manager call

        cm = self.get_parameter('controller_manager').value
        self._hardware = self.get_parameter('hardware_component').value
        self._timeout = self.get_parameter('service_timeout').value

        self._cb = ReentrantCallbackGroup()
        self._list_controllers = self.create_client(
            ListControllers, f'{cm}/list_controllers', callback_group=self._cb)
        self._list_hardware = self.create_client(
            ListHardwareComponents, f'{cm}/list_hardware_components', callback_group=self._cb)
        self._set_hardware = self.create_client(
            SetHardwareComponentState, f'{cm}/set_hardware_component_state',
            callback_group=self._cb)
        self._switch = self.create_client(
            SwitchController, f'{cm}/switch_controller', callback_group=self._cb)
        # controller nodes live in the controller manager's namespace
        self._ff_parameters = self.create_client(
            SetParameters, f'{cm.rsplit("/", 1)[0]}/{FF}/set_parameters', callback_group=self._cb)

        # one switch at a time (request or fault cleanup), a request while another one runs is
        # rejected rather than queued. EMERGENCY_OFF doesn't take it. The poll only reads and never
        # blocks a request
        self._switch_lock = threading.Lock()
        self._poll_lock = threading.Lock()
        self._emergency = False

        self._mode = RobotarmMode.UNKNOWN
        self._available = tuple(available_modes(None, set()))
        self._switching = False
        self._published = None
        self._state_pub = self.create_publisher(
            ModeState, '~/state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(SetMode, '~/set_mode', self._on_set_mode, callback_group=self._cb)
        self.create_timer(
            self.get_parameter('poll_period').value, self._on_poll, callback_group=self._cb)
        self._publish()

    # ------------------------------------------------------------------------------------------
    # callbacks

    async def _on_set_mode(self, request, response):
        target = request.target.mode
        if target == RobotarmMode.EMERGENCY_OFF:
            return await self._emergency_off(response)
        if self._emergency:
            return self._reject(response, 'emergency off, restart the firmware')
        if not self._switch_lock.acquire(blocking=False):
            return self._reject(response, 'another mode switch is in progress')
        self._publish(switching=True)
        try:
            await self._transition(target)
            response.success = True
            response.message = ''
        except ModeError as e:
            self.get_logger().error(f'switch to {mode_name(target)} failed: {e}')
            response.success = False
            response.message = str(e)
        finally:
            await self._refresh()
            response.mode.mode = self._mode
            self._publish(switching=False)
            self._switch_lock.release()
        return response

    async def _on_poll(self):
        if not self._poll_lock.acquire(blocking=False):
            return
        try:
            state = await self._read_state_or_none()
            # a request that started meanwhile publishes its own, newer state
            if self._switch_lock.locked():
                return
            self._publish_state(state)
            if self._mode != RobotarmMode.FAULT or self._emergency:
                return
            if not self._switch_lock.acquire(blocking=False):
                return
            try:
                state = await self._refresh()   # again under the lock, a request may have fixed it
                if state is not None and self._mode == RobotarmMode.FAULT:
                    await self._clear_fault(*state)
                    await self._refresh()
            except ModeError as e:
                self.get_logger().error(f'fault cleanup failed: {e}', throttle_duration_sec=5.0)
            finally:
                self._switch_lock.release()
        finally:
            self._poll_lock.release()

    def _reject(self, response, message):
        response.success = False
        response.message = message
        response.mode.mode = self._mode
        return response

    # ------------------------------------------------------------------------------------------
    # transitions

    async def _transition(self, target):
        hardware_state, active, loaded = await self._read_state()
        self._check_emergency()
        if target not in available_modes(hardware_state, loaded):
            if target not in MODES:
                raise ModeError(f'{mode_name(target)} can not be requested')
            if hardware_state not in ('active', 'inactive'):
                raise ModeError(f'hardware component is {hardware_state}, restart required')
            raise ModeError(f'controllers not loaded: {missing_controllers(loaded, target)}')
        plan = plan_switch(hardware_state, active, target)
        self.get_logger().info(
            f'{mode_name(detect_mode(hardware_state, active))} -> {mode_name(target)}: '
            f'deactivate {plan.deactivate}, activate {plan.activate}')

        if plan.ff_ramp_start is not None:
            await self._set_ff_ramp_start(plan.ff_ramp_start)
        if plan.activate_hardware:
            await self._set_hardware_state(State.PRIMARY_STATE_ACTIVE, 'active')
            self._check_emergency()
        try:
            await self._switch_controllers(plan.activate, plan.deactivate)
            self._check_emergency()
        except ModeError:
            # back to limp, as before the request: active hardware without controllers would
            # hold without feedforward and load the drive integrators
            if plan.activate_hardware and not self._emergency:
                await self._set_hardware_state(State.PRIMARY_STATE_INACTIVE, 'inactive')
            raise
        if plan.deactivate_hardware:
            await self._set_hardware_state(State.PRIMARY_STATE_INACTIVE, 'inactive')

    async def _emergency_off(self, response):
        # first in every running transition: it aborts at its next step
        self._emergency = True
        self.get_logger().error('EMERGENCY OFF requested, finalizing the hardware')
        response.success = True
        response.message = ''
        try:
            await self._set_hardware_state(State.PRIMARY_STATE_FINALIZED, 'finalized')
        except ModeError as e:
            hardware_state = None
            try:
                hardware_state, _, _ = await self._read_state()
            except ModeError:
                pass
            if hardware_state != 'finalized':   # a repeated request is no failure
                self.get_logger().fatal(f'EMERGENCY OFF failed: {e}')
                response.success = False
                response.message = str(e)
        # controllers stay active: finalized hardware rejects every switch, and nothing they
        # compute reaches the drives anymore
        await self._refresh()
        response.mode.mode = self._mode
        return response

    def _check_emergency(self):
        if self._emergency:
            raise ModeError('aborted by emergency off')

    async def _clear_fault(self, hardware_state, active, _loaded):
        plan = plan_switch(hardware_state, active, RobotarmMode.STOPPED)
        if hardware_state != 'active' or not plan.deactivate:
            return
        self.get_logger().error(f'no valid mode, deactivating {plan.deactivate}: STOPPED')
        await self._switch_controllers([], plan.deactivate)

    # ------------------------------------------------------------------------------------------
    # state

    async def _refresh(self):
        """Read the actual state and publish it, returns it or None if unreadable."""
        state = await self._read_state_or_none()
        self._publish_state(state)
        return state

    async def _read_state_or_none(self):
        try:
            return await self._read_state()
        except ModeError as e:
            self.get_logger().warn(f'state not readable: {e}', throttle_duration_sec=5.0)
            return None

    def _publish_state(self, state):
        if state is None:
            self._publish(mode=RobotarmMode.UNKNOWN, available=available_modes(None, set()))
            return
        hardware_state, active, loaded = state
        available = available_modes(hardware_state, loaded)
        if self._emergency:
            available = [m for m in available if m == RobotarmMode.EMERGENCY_OFF]
        self._publish(mode=detect_mode(hardware_state, active), available=available)

    def _publish(self, mode=None, available=None, switching=None):
        if mode is not None:
            if mode != self._mode:
                self.get_logger().info(f'mode: {mode_name(self._mode)} -> {mode_name(mode)}')
            self._mode = mode
        if available is not None:
            self._available = tuple(available)
        if switching is not None:
            self._switching = switching
        state = (self._mode, self._available, self._switching)
        if state == self._published:
            return
        self._published = state
        self._state_pub.publish(ModeState(
            mode=RobotarmMode(mode=self._mode),
            available=[RobotarmMode(mode=m) for m in self._available],
            switching=self._switching))

    # ------------------------------------------------------------------------------------------
    # controller manager

    async def _read_state(self):
        """Hardware state label, active controllers and loaded (configured) controllers."""
        hardware = await self._call(self._list_hardware, ListHardwareComponents.Request())
        hardware_state = next(
            (c.state.label for c in hardware.component if c.name == self._hardware), None)
        if hardware_state is None:
            raise ModeError(f'hardware component {self._hardware} not found')
        controllers = await self._call(self._list_controllers, ListControllers.Request())
        active = {c.name for c in controllers.controller if c.state == 'active'}
        loaded = {c.name for c in controllers.controller if c.state in ('active', 'inactive')}
        return hardware_state, active, loaded

    async def _switch_controllers(
            self, activate, deactivate, strictness=SwitchController.Request.STRICT):
        if not activate and not deactivate:
            return
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = strictness
        req.activate_asap = False
        req.timeout = Duration(sec=int(self._timeout))
        res = await self._call(self._switch, req)
        if not res.ok:
            raise ModeError(f'controller switch rejected: {res.message or "no reason given"}')

    async def _set_hardware_state(self, state_id, label):
        req = SetHardwareComponentState.Request()
        req.name = self._hardware
        req.target_state = State(id=state_id, label=label)
        res = await self._call(self._set_hardware, req)
        if not res.ok:
            raise ModeError(f'hardware component {self._hardware} did not become {label}')

    async def _set_ff_ramp_start(self, ramp_start):
        req = SetParameters.Request()
        req.parameters = [Parameter(name='ramp_start', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=ramp_start))]
        res = await self._call(self._ff_parameters, req)
        if not res.results or not res.results[0].successful:
            raise ModeError(f'{FF} ramp_start {ramp_start} not set')

    async def _call(self, client, request):
        if not client.service_is_ready():
            raise ModeError(f'{client.srv_name} not available')
        result = Future(executor=self.executor)

        def on_done(future):
            if result.done():
                return
            if future.exception() is not None:
                result.set_exception(future.exception())
            else:
                result.set_result(future.result())

        def on_timeout():
            if not result.done():
                result.set_exception(ModeError(f'{client.srv_name} timed out'))

        timer = self.create_timer(self._timeout, on_timeout, callback_group=self._cb)
        try:
            client.call_async(request).add_done_callback(on_done)
            return await result
        finally:
            timer.cancel()
            self.destroy_timer(timer)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
