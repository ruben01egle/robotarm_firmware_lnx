"""
Drive the arm through configured single axis sweeps with the joint stream controller and record them.

For each sweep: move to its start pose, then move one axis by a stroke and back, at every configured speed.
Each sweep is recorded to its own csv with the recorder, every move is a segment. A run.json next to the csvs
holds the config, the controller limits and every segment with its file, label and start/end stamps.
See README.md for the config format.
"""

import argparse
import copy
import json
import math
import socket
import sys
import threading
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from robotarm_interface.msg import ModeState, RobotarmMode
from robotarm_interface.srv import SetFloat64, SetMode
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from robotarm_tools.recorder import Recorder

SETTINGS = {                    # top level keys with their defaults
    'approach_speed': 0.3,      # rad/s, moving to the start poses
    'settle_time': 1.0,         # s at rest before each move, recorded
    'position_tolerance': 0.01,  # rad, arrived when every joint is this close to the target ...
    'velocity_tolerance': 0.02,  # rad/s ... and slower than this
    'limit_margin': 0.05,       # rad, poses and strokes must stay this far inside the URDF limits
}
SWEEP_KEYS = {'name', 'pose', 'axis', 'stroke', 'speeds', 'repeat'}
DEFAULT_KEYS = SWEEP_KEYS - {'name', 'pose'}


class ConfigError(Exception):
    pass


class MotionError(Exception):
    pass


# ----------------------------------------------------------------------------------------------- config

def load_config(source):
    """Read and check a config (path or dict). Returns the resolved config, sweeps with all keys filled in."""
    if isinstance(source, dict):
        raw = copy.deepcopy(source)
    else:
        with open(source) as f:
            raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise ConfigError('config must be a mapping')

    unknown = set(raw) - set(SETTINGS) - {'name', 'description', 'poses', 'defaults', 'sweeps'}
    if unknown:
        raise ConfigError(f'unknown keys: {sorted(unknown)}')
    if not raw.get('name'):
        raise ConfigError('missing name')

    config = {'name': str(raw['name']), 'description': str(raw.get('description', ''))}
    for key, default in SETTINGS.items():
        config[key] = _positive(raw.get(key, default), key)

    poses = raw.get('poses') or {}
    if not isinstance(poses, dict) or not poses:
        raise ConfigError('poses must be a non-empty mapping of name: [joint positions]')
    for name, pose in poses.items():
        if not isinstance(pose, list) or not all(_is_number(p) for p in pose):
            raise ConfigError(f'pose {name}: must be a list of joint positions [rad]')
    config['poses'] = {str(k): [float(p) for p in v] for k, v in poses.items()}

    defaults = raw.get('defaults') or {}
    unknown = set(defaults) - DEFAULT_KEYS
    if unknown:
        raise ConfigError(f'defaults: unknown keys {sorted(unknown)}, allowed {sorted(DEFAULT_KEYS)}')

    sweeps = raw.get('sweeps') or []
    if not isinstance(sweeps, list) or not sweeps:
        raise ConfigError('sweeps must be a non-empty list')
    config['sweeps'] = []
    names = set()
    for i, raw_sweep in enumerate(sweeps):
        sweep = {**defaults, **raw_sweep}
        where = f'sweep {sweep.get("name", i + 1)}'
        unknown = set(sweep) - SWEEP_KEYS
        if unknown:
            raise ConfigError(f'{where}: unknown keys {sorted(unknown)}')
        missing = SWEEP_KEYS - set(sweep) - {'repeat'}
        if missing:
            raise ConfigError(f'{where}: missing {sorted(missing)}')
        name = str(sweep['name'])
        if name in names or not name.replace('_', '').replace('-', '').isalnum():
            raise ConfigError(f'{where}: name must be unique and only letters, digits, _ and -')
        names.add(name)
        if sweep['pose'] not in config['poses']:
            raise ConfigError(f'{where}: unknown pose {sweep["pose"]}')
        if not _is_number(sweep['stroke']) or sweep['stroke'] == 0:
            raise ConfigError(f'{where}: stroke must be a non-zero number [rad]')
        speeds = sweep['speeds'] if isinstance(sweep['speeds'], list) else [sweep['speeds']]
        repeat = sweep.get('repeat', 1)
        if not isinstance(repeat, int) or isinstance(repeat, bool) or repeat < 1:
            raise ConfigError(f'{where}: repeat must be an integer >= 1')
        config['sweeps'].append({
            'name': name,
            'pose': str(sweep['pose']),
            'axis': str(sweep['axis']),
            'stroke': float(sweep['stroke']),
            'speeds': [_positive(v, f'{where}: speeds') for v in speeds],
            'repeat': repeat,
        })
    return config


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _positive(value, what):
    if not _is_number(value) or value <= 0:
        raise ConfigError(f'{what} must be a positive number, got {value!r}')
    return float(value)


def select_sweeps(config, names):
    if not names:
        return config
    unknown = set(names) - {s['name'] for s in config['sweeps']}
    if unknown:
        raise ConfigError(f'unknown sweeps: {sorted(unknown)}')
    config = copy.deepcopy(config)
    config['sweeps'] = [s for s in config['sweeps'] if s['name'] in names]
    return config


def moves(sweep):
    """Yield (direction, speed, repeat) of a sweep in execution order."""
    for repeat in range(1, sweep['repeat'] + 1):
        for speed in sweep['speeds']:
            yield 'out', speed, repeat
            yield 'back', speed, repeat


def segment_label(sweep, direction, speed, repeat):
    return f'{sweep["name"]}_v{speed:g}_r{repeat}_{direction}'


def estimate_duration(config):
    """Seconds without the approach moves, which depend on where the arm is."""
    total = 0.0
    for sweep in config['sweeps']:
        for _, speed, _ in moves(sweep):
            total += config['settle_time'] + abs(sweep['stroke']) / speed + 1.0
    return total


def describe(config):
    lines = [f'{config["name"]}: {len(config["sweeps"])} sweeps, about '
             f'{estimate_duration(config) / 60:.0f} min plus the moves to the start poses']
    for sweep in config['sweeps']:
        n = sum(1 for _ in moves(sweep))
        lines.append(f'  {sweep["name"]}: pose {sweep["pose"]}, {sweep["axis"]} {sweep["stroke"]:+g} rad and back, '
                     f'speeds {sweep["speeds"]} rad/s x{sweep["repeat"]}, {n} moves')
    return '\n'.join(lines)


# ----------------------------------------------------------------------------------------------- runner

class MotionRunner:
    """
    Drives the arm through the sweeps of a config and records them.

    Uses its own node and executor in a background thread, initializes rclpy (without signal handlers) if
    the caller has not. The arm is moved with the joint stream controller (mode JOINT_STREAM through the mode
    manager), the speed set through its scale_speed service. Ends in ARMED, also on an error or Ctrl-C.
    """

    def __init__(self, config, output_dir='.', controller_manager='/controller_manager',
                 mode_manager='/mode_manager', controller='joint_stream_controller',
                 broadcaster='joint_state_broadcaster_hires', timeout=5.0):
        self.config = config
        self.output_dir = Path(output_dir).expanduser()
        self.controller_manager = controller_manager.rstrip('/')
        self.mode_manager = mode_manager.rstrip('/')
        # controller nodes live in the controller manager's namespace
        self.controller = f'{self.controller_manager.rsplit("/", 1)[0]}/{controller}'
        self.broadcaster = broadcaster
        self.timeout = timeout

        self.run_dir = None
        self.manifest = None
        self._lock = threading.Lock()
        self._joint_state = None
        self._mode_state = None
        self._robot_description = None
        self._expect_joint_stream = False

    # ------------------------------------------------------------------------------------------- public

    def run(self, confirm=True):
        """Run all sweeps, returns the run directory (None if not confirmed). Raises on errors, in ARMED then."""
        owns_rclpy = not rclpy.ok()
        if owns_rclpy:
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._start_node()
        try:
            self._connect()
            self._validate()
            print(describe(self.config))
            if confirm and not self._confirm():
                print('aborted, nothing moved')
                return None
            self._create_run_dir()
            moving = False
            try:
                self._set_mode(RobotarmMode.JOINT_STREAM)
                moving = True
                self._expect_joint_stream = True
                for sweep in self.config['sweeps']:
                    self._run_sweep(sweep)
                self.manifest['status'] = 'complete'
            except BaseException as e:
                self.manifest['status'] = 'aborted'
                self.manifest['error'] = f'{type(e).__name__}: {e}'
                raise
            finally:
                if moving:
                    self._stop_motion()
                self.manifest['finished'] = _now()
                self._write_manifest()
                print(f'{self.manifest["status"]}: {len(self.manifest["segments"])} segments in {self.run_dir}')
            return self.run_dir
        finally:
            self._stop_node()
            if owns_rclpy and rclpy.ok():
                rclpy.shutdown()

    # ------------------------------------------------------------------------------------------- steps

    def _connect(self):
        self._wait_for(lambda: self._mode_state is not None,
                       f'{self.mode_manager}/state, is the mode manager running?')
        self._wait_for(lambda: self._joint_state is not None, '/joint_states')
        self._wait_for(lambda: self._robot_description is not None, '/robot_description')
        # segment stamps on the clock of the controller manager (sim time in the simulation)
        values = self._call(self._cm_parameters, GetParameters.Request(names=['use_sim_time'])).values
        self.use_sim_time = bool(values and values[0].bool_value)
        self._node.set_parameters([Parameter('use_sim_time', value=self.use_sim_time)])
        names = ['joints', 'max_velocity', 'max_acceleration', 'max_jerk']
        values = self._call(self._get_parameters, GetParameters.Request(names=names)).values
        if len(values) != len(names):
            raise MotionError(f'could not read the parameters of {self.controller}')
        self.joints = list(values[0].string_array_value)
        self.limits = {
            'max_velocity': values[1].double_value,
            'max_acceleration': values[2].double_value,
            'max_jerk': values[3].double_value,
        }
        if not self.joints or self.limits['max_velocity'] <= 0:
            raise MotionError(f'{self.controller}: no joints or no max_velocity, is it loaded?')

    def _validate(self):
        errors = []
        joint_limits = urdf_joint_limits(self._robot_description)
        margin = self.config['limit_margin']
        v_max = self.limits['max_velocity']
        if self.config['approach_speed'] > v_max:
            errors.append(f'approach_speed {self.config["approach_speed"]} > max_velocity {v_max} of the controller')

        def check_position(where, joint, value):
            lower, upper = joint_limits.get(joint, (-math.inf, math.inf))
            if not lower + margin <= value <= upper - margin:
                errors.append(f'{where}: {joint} = {value:.3f} outside [{lower:.3f}, {upper:.3f}] '
                              f'minus margin {margin}')

        for name, pose in self.config['poses'].items():
            if len(pose) != len(self.joints):
                errors.append(f'pose {name}: {len(pose)} values, the controller has {len(self.joints)} joints '
                              f'{self.joints}')
                continue
            for joint, value in zip(self.joints, pose):
                check_position(f'pose {name}', joint, value)
        for sweep in self.config['sweeps']:
            where = f'sweep {sweep["name"]}'
            if sweep['axis'] not in self.joints:
                errors.append(f'{where}: axis {sweep["axis"]} not in {self.joints}')
                continue
            pose = self.config['poses'][sweep['pose']]
            if len(pose) == len(self.joints):
                start = pose[self.joints.index(sweep['axis'])]
                check_position(f'{where} stroke end', sweep['axis'], start + sweep['stroke'])
            for speed in sweep['speeds']:
                if speed > v_max:
                    errors.append(f'{where}: speed {speed} > max_velocity {v_max} of the controller')
        if errors:
            raise ConfigError('\n  '.join(['config does not fit the arm:'] + errors))

    def _confirm(self):
        print('\n' + '!' * 100)
        print('WARNING: the arm is about to be powered and moved through all poses and strokes.')
        print('Make sure it can move freely everywhere, with nothing and nobody in reach.')
        print('Ctrl-C stops the motion (mode ARMED). Keep the emergency stop at hand.')
        print('!' * 100)
        try:
            return input('type "yes" to start: ').strip().lower() == 'yes'
        except EOFError:
            return False

    def _create_run_dir(self):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = self.output_dir / f'{self.config["name"]}_{stamp}'
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.manifest = {
            'tool': 'robotarm_tools.motion_runner',
            'name': self.config['name'],
            'description': self.config['description'],
            'status': 'running',
            'error': None,
            'started': _now(),
            'finished': None,
            'host': socket.gethostname(),
            'stamps': f'{"sim" if self.use_sim_time else "ROS"} time [s], the same clock as the csv stamp column',
            'controller': {'name': self.controller, 'joints': self.joints, **self.limits},
            'hw_diagnostics': None,
            'config': self.config,
            'files': {},
            'segments': [],
        }
        self._write_manifest()

    def _run_sweep(self, sweep):
        print(f'\n{sweep["name"]}: moving to pose {sweep["pose"]}')
        pose = self.config['poses'][sweep['pose']]
        self._move(pose, self.config['approach_speed'])

        axis = self.joints.index(sweep['axis'])
        file_name = f'{sweep["name"]}.csv'
        rec = Recorder(self.run_dir / file_name, broadcaster=self.broadcaster,
                       controller_manager=self.controller_manager, use_sim_time=self.use_sim_time,
                       node_name='motion_runner_recorder')
        entry = {'sweep': sweep['name'], 'status': 'recording'}
        self.manifest['files'][file_name] = entry
        with rec:
            try:
                rec.mark('settle')
                time.sleep(self.config['settle_time'])
                for direction, speed, repeat in moves(sweep):
                    target = list(pose)
                    if direction == 'out':
                        target[axis] += sweep['stroke']
                    label = segment_label(sweep, direction, speed, repeat)
                    print(f'  {label}')
                    segment = {
                        'label': label, 'file': file_name, 'sweep': sweep['name'], 'axis': sweep['axis'],
                        'pose': sweep['pose'], 'direction': direction, 'speed': speed, 'repeat': repeat,
                        'from': self._positions()[axis], 'to': target[axis],
                        't_start': None, 't_end': None, 'status': 'running',
                    }
                    self.manifest['segments'].append(segment)
                    rec.mark(label)
                    segment['t_start'] = self._ros_now()
                    self._move(target, speed)
                    segment['t_end'] = self._ros_now()
                    segment['status'] = 'complete'
                    rec.mark('settle')
                    self._write_manifest()
                    time.sleep(self.config['settle_time'])
            except BaseException:
                # stop first, closing the recorder (service calls) can wait
                self._stop_motion()
                for segment in self.manifest['segments']:
                    if segment['status'] == 'running':
                        segment['status'] = 'aborted'
                        segment['t_end'] = self._ros_now()
                entry['status'] = 'aborted'
                raise
        summary = rec.summary()
        columns = summary['columns'] or []
        entry.update(status='complete', samples=summary['samples'], missed=summary['missed'], columns=columns)
        if self.manifest['hw_diagnostics'] is None:
            self.manifest['hw_diagnostics'] = any(c.endswith('_rotor_position') for c in columns)
            if not self.manifest['hw_diagnostics']:
                print('  WARNING: no drive diagnostics in the recording, is hw_diagnostics on?')
        if summary['missed']:
            print(f'  WARNING: {summary["missed"]} samples missed')
        self._write_manifest()

    def _move(self, target, speed):
        """Command target [rad, controller joint order] at speed [rad/s], return once arrived and at rest."""
        self._set_scale(min(1.0, speed / self.limits['max_velocity']))
        start = self._positions()
        distance = max(abs(t - s) for t, s in zip(target, start))
        timeout = 1.5 * distance / speed + 10.0
        self._command.publish(Float64MultiArray(data=[float(t) for t in target]))

        t0 = time.monotonic()
        while True:
            time.sleep(0.02)
            self._check_mode()
            q, v = self._positions(), self._velocities()
            if (all(abs(a - b) <= self.config['position_tolerance'] for a, b in zip(q, target))
                    and all(abs(x) <= self.config['velocity_tolerance'] for x in v)):
                return
            if time.monotonic() - t0 > timeout:
                worst = max(range(len(q)), key=lambda i: abs(q[i] - target[i]))
                raise MotionError(f'target not reached within {timeout:.0f} s, {self.joints[worst]} is '
                                  f'{q[worst] - target[worst]:+.3f} rad off')

    def _stop_motion(self):
        self._expect_joint_stream = False
        try:
            self._set_mode(RobotarmMode.ARMED)
        except Exception as e:
            print(f'ERROR: switching to ARMED failed ({e}), STOP THE ARM BY HAND', file=sys.stderr)

    # ------------------------------------------------------------------------------------------- ros

    def _start_node(self):
        self._node = rclpy.create_node('motion_runner')
        node = self._node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(ModeState, f'{self.mode_manager}/state', self._on_mode_state, latched)
        node.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        node.create_subscription(String, '/robot_description', self._on_robot_description, latched)
        self._command = node.create_publisher(Float64MultiArray, f'{self.controller}/commands', 10)
        self._set_mode_client = node.create_client(SetMode, f'{self.mode_manager}/set_mode')
        self._scale_client = node.create_client(SetFloat64, f'{self.controller}/scale_speed')
        self._get_parameters = node.create_client(GetParameters, f'{self.controller}/get_parameters')
        self._cm_parameters = node.create_client(GetParameters, f'{self.controller_manager}/get_parameters')

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)
        self._spinning = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _stop_node(self):
        self._spinning = False
        self._thread.join()
        self._executor.shutdown()
        self._node.destroy_node()

    def _spin(self):
        try:
            while self._spinning:
                self._executor.spin_once(timeout_sec=0.1)
        except ExternalShutdownException:
            pass

    def _on_mode_state(self, msg):
        self._mode_state = msg

    def _check_mode(self):
        """Raise if someone else switched the mode away from JOINT_STREAM during the run."""
        state = self._mode_state
        if self._expect_joint_stream and not state.switching and state.mode.mode != RobotarmMode.JOINT_STREAM:
            raise MotionError(f'mode changed to {state.mode.mode} by someone else, is another client running?')

    def _on_joint_state(self, msg):
        with self._lock:
            self._joint_state = (time.monotonic(), dict(zip(msg.name, zip(msg.position, msg.velocity))))

    def _on_robot_description(self, msg):
        self._robot_description = msg.data

    def _state(self):
        with self._lock:
            received, state = self._joint_state
        if time.monotonic() - received > 0.5:
            raise MotionError('joint states older than 0.5 s')
        return state

    def _positions(self):
        state = self._state()
        return [state[j][0] for j in self.joints]

    def _velocities(self):
        state = self._state()
        return [state[j][1] for j in self.joints]

    def _ros_now(self):
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _wait_for(self, condition, what):
        t_end = time.monotonic() + self.timeout
        while not condition():
            if time.monotonic() > t_end:
                raise MotionError(f'nothing received on {what}')
            time.sleep(0.05)

    def _call(self, client, request, timeout=None):
        timeout = timeout or self.timeout
        if not client.wait_for_service(timeout_sec=timeout):
            raise MotionError(f'service {client.srv_name} not available')
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise MotionError(f'service {client.srv_name} timed out')
        return future.result()

    def _set_mode(self, mode):
        request = SetMode.Request()
        request.target.mode = mode
        # activating the hardware and switching controllers takes a while
        response = self._call(self._set_mode_client, request, timeout=max(self.timeout, 15.0))
        if not response.success:
            raise MotionError(f'set_mode failed: {response.message}')

    def _set_scale(self, scale):
        # every move, not only on a change: a reactivation of the controller resets its scale to 1
        response = self._call(self._scale_client, SetFloat64.Request(data=scale))
        if not response.success:
            raise MotionError(f'scale_speed {scale} rejected: {response.message}')

    def _write_manifest(self):
        path = self.run_dir / 'run.json'
        tmp = path.with_suffix('.json.tmp')
        tmp.write_text(json.dumps(self.manifest, indent=2, default=str))
        tmp.replace(path)


def urdf_joint_limits(urdf):
    """{joint: (lower, upper)} of the limited joints of a URDF string."""
    limits = {}
    for joint in ET.fromstring(urdf).iter('joint'):
        limit = joint.find('limit')
        if joint.get('type') in ('revolute', 'prismatic') and limit is not None:
            limits[joint.get('name')] = (float(limit.get('lower', 0.0)), float(limit.get('upper', 0.0)))
    return limits


def _now():
    return datetime.now().astimezone().isoformat(timespec='seconds')


def run(config, output_dir='.', sweeps=None, confirm=True, **kwargs):
    """Load config (path or dict), run the selected sweeps (all if None). Returns the run directory."""
    resolved = select_sweeps(load_config(config), sweeps)
    return MotionRunner(resolved, output_dir, **kwargs).run(confirm=confirm)


def main(argv=None):
    if argv is None:
        argv = remove_ros_args(sys.argv)[1:]
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('config', help='sweep config (yaml)')
    parser.add_argument('-o', '--output-dir', default='.', help='runs are written to <dir>/<name>_<date>_<time>/')
    parser.add_argument('--sweeps', nargs='+', help='only these sweeps (default: all)')
    parser.add_argument('--dry-run', action='store_true',
                        help='only check the config and print the plan, no ROS needed (limits are not checked)')
    parser.add_argument('--yes', action='store_true', help='skip the confirmation')
    parser.add_argument('--controller-manager', default='/controller_manager')
    parser.add_argument('--mode-manager', default='/mode_manager')
    args = parser.parse_args(argv)
    try:
        if args.dry_run:
            print(describe(select_sweeps(load_config(args.config), args.sweeps)))
            return 0
        run(args.config, args.output_dir, args.sweeps, confirm=not args.yes,
            controller_manager=args.controller_manager, mode_manager=args.mode_manager)
    except (ConfigError, MotionError) as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('stopped by Ctrl-C', file=sys.stderr)
        return 130
    return 0


if __name__ == '__main__':
    sys.exit(main())
