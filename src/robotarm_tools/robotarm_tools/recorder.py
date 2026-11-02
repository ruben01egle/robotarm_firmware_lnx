"""
Record every message of a joint state broadcaster's dynamic_joint_states to a csv.

Activates the broadcaster through the controller manager (loads and configures it first if needed), records
until stopped and deactivates it again, unless it was already active before. With hw_diagnostics on, the
moteus drive diagnostics are in there as well. See README.md for usage and the csv layout.
"""

import argparse
import csv
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from control_msgs.msg import DynamicJointState
from controller_manager_msgs import srv
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

DEFAULT_BROADCASTER = 'joint_state_broadcaster_hires'


class Recorder:
    """
    Records a broadcaster's dynamic_joint_states to a csv, from start() until stop().

    Uses its own node and executor in a background thread, so it does not interfere with the caller's nodes
    or spinning. Initializes rclpy itself (without signal handlers, Ctrl-C stays a KeyboardInterrupt) if the
    caller has not, and shuts it down again in stop().

        with Recorder('run1.csv') as rec:
            rec.mark('v0.02_up')
            ...
    """

    def __init__(self, output=None, broadcaster=DEFAULT_BROADCASTER,
                 controller_manager='/controller_manager', rate=500.0, use_sim_time=None,
                 node_name='robotarm_recorder', timeout=5.0):
        self.output = Path(output) if output else Path(time.strftime('joint_states_%Y%m%d_%H%M%S.csv'))
        self.broadcaster = broadcaster
        self.controller_manager = controller_manager.rstrip('/')
        self.rate = rate                    # expected broadcaster rate, only for counting missed samples
        self.use_sim_time = use_sim_time    # None: the same as the controller manager, read in start()
        self.node_name = node_name
        self.timeout = timeout

        self.samples = 0
        self.missed = 0

        self._node = None
        self._executor = None
        self._thread = None
        self._spinning = False
        self._owns_rclpy = False
        self._activated = False
        self._sub = None
        self._file = None
        self._writer = None
        self._columns = None
        self._first_stamp = None
        self._last_stamp = None
        self._segment = ''
        self._marks = deque()

    @property
    def topic(self):
        return f'/{self.broadcaster}/dynamic_joint_states'

    @property
    def columns(self):
        """Csv column names after stamp and segment, None until the first message arrived."""
        return None if self._columns is None else [f'{n}_{i}' for n, i in self._columns]

    # ------------------------------------------------------------------------------------------- public

    def start(self):
        if self._node is not None:
            raise RuntimeError('already recording')

        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            # the default handler shuts the context down on Ctrl-C, then the deactivate call in stop() fails
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._node = rclpy.create_node(self.node_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        try:
            self._list = self._client(srv.ListControllers, 'list_controllers')
            self._load = self._client(srv.LoadController, 'load_controller')
            self._configure = self._client(srv.ConfigureController, 'configure_controller')
            self._switch = self._client(srv.SwitchController, 'switch_controller')

            # mark() compares its clock with the stamps of the controller manager
            if self.use_sim_time is None:
                self.use_sim_time = self._controller_manager_uses_sim_time()
            self._node.set_parameters([Parameter('use_sim_time', value=self.use_sim_time)])

            self.output.parent.mkdir(parents=True, exist_ok=True)
            self._file = self.output.open('w', newline='')
            self._writer = csv.writer(self._file)
            self._sub = self._node.create_subscription(
                DynamicJointState, self.topic, self._on_msg,
                QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))

            self._activated = self._activate()
        except BaseException:
            self._teardown()
            raise

        self._spinning = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def mark(self, label):
        """Label all samples from now on (by their control loop stamp) with label, until the next mark."""
        if self._node is None:
            self._segment = str(label)
            return
        self._marks.append((self._node.get_clock().now().nanoseconds * 1e-9, str(label)))

    def stop(self):
        """Stop recording, deactivate the broadcaster if start() activated it. Returns a summary dict."""
        if self._node is None:
            return self.summary()
        self._spinning = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        self._teardown()
        return self.summary()

    def summary(self):
        duration = 0.0 if self._first_stamp is None else self._last_stamp - self._first_stamp
        return {'output': self.output, 'samples': self.samples, 'duration': duration, 'missed': self.missed,
                'columns': self.columns}

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
        return False

    # ------------------------------------------------------------------------------------------- recording

    def _on_msg(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._columns is None:
            # interfaces of the first message, sorted (the broadcaster's order is arbitrary), later ones are
            # matched by name
            self._columns = sorted((name, interface)
                                   for name, values in zip(msg.joint_names, msg.interface_values)
                                   for interface in values.interface_names)
            self._writer.writerow(['stamp', 'segment'] + [f'{n}_{i}' for n, i in self._columns])
            self._first_stamp = stamp

        values = {}
        for name, interface_values in zip(msg.joint_names, msg.interface_values):
            for interface, value in zip(interface_values.interface_names, interface_values.values):
                values[(name, interface)] = value

        while self._marks and stamp >= self._marks[0][0]:
            self._segment = self._marks.popleft()[1]

        if self._last_stamp is not None:
            self.missed += max(0, round((stamp - self._last_stamp) * self.rate) - 1)
        self._last_stamp = stamp

        self._writer.writerow([f'{stamp:.6f}', self._segment]
                              + [values.get(c, math.nan) for c in self._columns])
        self.samples += 1

    def _spin(self):
        try:
            while self._spinning:
                self._executor.spin_once(timeout_sec=0.1)
        except ExternalShutdownException:
            # the caller's rclpy context was shut down (their Ctrl-C handler), nothing left to record
            pass

    # ------------------------------------------------------------------------------------------- broadcaster

    def _client(self, srv_type, name):
        return self._node.create_client(srv_type, f'{self.controller_manager}/{name}')

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f'service {client.srv_name} not available, is the controller manager running?')
        future = client.call_async(request)
        self._executor.spin_until_future_complete(future, timeout_sec=self.timeout)
        if future.result() is None:
            raise RuntimeError(f'service {client.srv_name} timed out')
        return future.result()

    def _controller_manager_uses_sim_time(self):
        client = self._client(GetParameters, 'get_parameters')
        values = self._call(client, GetParameters.Request(names=['use_sim_time'])).values
        return bool(values and values[0].bool_value)

    def _state(self):
        for c in self._call(self._list, srv.ListControllers.Request()).controller:
            if c.name == self.broadcaster:
                return c.state
        return None

    def _set_active(self, active):
        req = srv.SwitchController.Request()
        if active:
            req.activate_controllers = [self.broadcaster]
        else:
            req.deactivate_controllers = [self.broadcaster]
        req.strictness = srv.SwitchController.Request.STRICT
        if not self._call(self._switch, req).ok:
            raise RuntimeError(f'switching {self.broadcaster} to {"active" if active else "inactive"} failed')

    def _activate(self):
        """Return True if this call activated the broadcaster, False if it already was active."""
        state = self._state()
        if state is None:
            if not self._call(self._load, srv.LoadController.Request(name=self.broadcaster)).ok:
                raise RuntimeError(f'loading {self.broadcaster} failed, is it in controllers.yaml?')
            state = self._state()
        if state == 'unconfigured':
            if not self._call(self._configure, srv.ConfigureController.Request(name=self.broadcaster)).ok:
                raise RuntimeError(f'configuring {self.broadcaster} failed')
            state = self._state()
        if state == 'active':
            return False
        self._set_active(True)
        return True

    def _teardown(self):
        # first: the deactivate call below spins the executor, queued messages must not reach the closed file
        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
            self._sub = None
        if self._file is not None:
            self._file.close()
            self._file = None
        if self._activated:
            self._activated = False
            try:
                if not rclpy.ok():
                    raise RuntimeError('rclpy already shut down')
                self._set_active(False)
            except RuntimeError as e:
                print(f'WARNING: {e}, deactivate it by hand: '
                      f'ros2 control set_controller_state {self.broadcaster} inactive', file=sys.stderr)
        self._executor.shutdown()
        self._node.destroy_node()
        self._node = None
        self._executor = None
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()


def record(output=None, duration=0.0, progress=True, **kwargs):
    """
    Record until Ctrl-C or duration (s, 0 = no limit). Blocks, returns the summary dict.

    kwargs go to Recorder.
    """
    rec = Recorder(output, **kwargs)
    rec.start()
    if progress:
        print(f'recording {rec.topic} -> {rec.output}, Ctrl-C to stop')
    t_end = time.monotonic() + duration if duration > 0 else math.inf
    t_report = time.monotonic() + 5.0
    try:
        while time.monotonic() < t_end:
            time.sleep(0.1)
            if progress and time.monotonic() > t_report:
                print(f'  {rec.samples} samples')
                t_report += 5.0
    except KeyboardInterrupt:
        pass
    finally:
        summary = rec.stop()
    if progress:
        print(format_summary(summary))
    return summary


def format_summary(summary):
    text = f'{summary["samples"]} samples, {summary["duration"]:.1f} s written to {summary["output"]}'
    if summary['missed']:
        text += f', {summary["missed"]} samples missed'
    return text


def main(argv=None):
    if argv is None:
        argv = remove_ros_args(sys.argv)[1:]
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('-o', '--output', help='csv path (default: joint_states_<date>_<time>.csv)')
    parser.add_argument('--duration', type=float, default=0.0, help='s, 0 = until Ctrl-C')
    parser.add_argument('--broadcaster', default=DEFAULT_BROADCASTER, help='joint state broadcaster to record')
    parser.add_argument('--controller-manager', default='/controller_manager', help='controller manager node')
    parser.add_argument('--rate', type=float, default=500.0,
                        help='Hz, publish rate of the broadcaster, only for counting missed samples')
    args = parser.parse_args(argv)
    try:
        record(args.output, args.duration, broadcaster=args.broadcaster,
               controller_manager=args.controller_manager, rate=args.rate)
    except RuntimeError as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
