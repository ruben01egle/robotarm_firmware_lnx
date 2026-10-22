"""
Push the simulated arm with a defined wrench, release it and measure the response per joint.

Reports overshoot, damping ratio, frequency and settling time, next to what the gains in
controllers.yaml should give at that pose. Needs the sim (it loads the wrench plugin) and the
gc_controller active, see the README.

    ros2 run robotarm_mujoco push_test [--body stage3] [--force 0 0 -10] [--duration 0.3]
"""

import argparse
import csv
import sys

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
import mujoco
from mujoco_ros2_control_msgs.msg import ExternalWrench
from mujoco_ros2_control_msgs.srv import ApplyExternalWrench, ResetWorld, SetPause
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from robotarm_mujoco import dynamics
from sensor_msgs.msg import JointState

SERVICE = '/external_wrench/apply_wrench'
SIM = '/mujoco_ros2_control_node'
SETTLE_BAND = 0.05      # settling time: last time outside 5% of the deflection at release
MIN_DEFLECTION = 1e-3   # rad, joints that moved less are not evaluated
PEAK_MIN = 0.03         # overshoot below 3% of the deflection at release counts as none
COUPLED = 1.5           # moved more than this times its deflection at release: driven by others
MIN_JOINT_TORQUE = 1e-3  # Nm, below this on the pushed body's own joint the push cannot move it
LIMIT_MARGIN = 0.01     # rad, closer to a position limit counts as resting on it
CONTACT_MARGIN = 0.005  # m, closer to the floor counts as resting on it


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--body', default='stage3', help='MJCF body (stage1..stage6)')
    parser.add_argument('--force', type=float, nargs=3, default=[0.0, 0.0, -10.0],
                        help='N, world frame')
    parser.add_argument('--torque', type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        help='Nm, world frame')
    parser.add_argument('--point', type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        help='m, application point in the body frame')
    parser.add_argument('--duration', type=float, default=0.3, help='s, push length')
    parser.add_argument('--record', type=float, default=3.0, help='s, recorded after the release')
    parser.add_argument('--pose', type=float, nargs=6, metavar='Q',
                        help='rad, reset the sim to this pose, held by the controller')
    parser.add_argument('--tool', default='none', help='tool of the sim, for the expected values')
    parser.add_argument('--params', help='sim parameter yaml (default: installed sim_params.yaml)')
    parser.add_argument('--controllers',
                        help='controllers.yaml with the gains (default: installed bringup)')
    parser.add_argument('--csv', help='write the recording (t, q per joint) to this file')
    return parser.parse_args(argv)


class Recorder(Node):

    def __init__(self):
        super().__init__('push_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.names = None
        self.t, self.q = [], []
        self.create_subscription(JointState, '/joint_states', self.on_state, 100)
        self.wrench = self.create_client(ApplyExternalWrench, SERVICE)
        self.controllers = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.pause = self.create_client(SetPause, SIM + '/set_pause')
        self.reset = self.create_client(ResetWorld, SIM + '/reset_world')

    def on_state(self, msg):
        if self.names is None:
            self.names = list(msg.name)
        order = [msg.name.index(n) for n in self.names]
        self.t.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.q.append([msg.position[i] for i in order])

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_until(self, t_end):
        while rclpy.ok() and self.now() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=5.0):
            sys.exit(f'service {client.srv_name} not available')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()


class Model:
    """The sim's MuJoCo model, for body orientations and the expected response at a pose."""

    def __init__(self, tool, params_file, controllers_file):
        self.m = dynamics.load_model(tool, params_file)
        self.d = mujoco.MjData(self.m)
        self.names = dynamics.joint_names(self.m)
        self.damping, self.stiffness = dynamics.load_gains(controllers_file)

    def set_pose(self, names, q):
        self.q = np.array([q[names.index(n)] for n in self.names])
        self.m_ii = np.diag(dynamics.full_mass_matrix(self.m, self.d, self.q))

    def body_id(self, body):
        body_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, body)
        if body_id < 0:
            sys.exit(f'unknown body {body}')
        return body_id

    def rotation(self, body):
        return self.d.xmat[self.body_id(body)].reshape(3, 3).copy()

    def joint_torques(self, body, point, force, torque):
        """Joint torques of a world frame wrench at a point given in the body frame."""
        b = self.body_id(body)
        p = self.d.xpos[b] + self.rotation(body) @ np.array(point)
        jp, jr = np.zeros((3, self.m.nv)), np.zeros((3, self.m.nv))
        mujoco.mj_jac(self.m, self.d, jp, jr, p, b)
        return jp.T @ np.array(force) + jr.T @ np.array(torque)

    def bodies_on_floor(self, margin):
        """Bodies with a collision geom closer than margin to the floor (resting on it)."""
        m = self.m
        floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
        fromto = np.zeros(6)
        bodies = {m.geom_bodyid[g] for g in range(m.ngeom)
                  if m.geom_bodyid[g] != 0 and (m.geom_contype[g] or m.geom_conaffinity[g])
                  and mujoco.mj_geomDistance(m, self.d, floor, g, margin, fromto) < margin}
        return [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in sorted(bodies)]

    def expected(self):
        """Damping ratio and natural frequency of the hold the gains should give at the pose."""
        out = {}
        for i, n in enumerate(self.names):
            k, d, m_ii = self.stiffness[i], self.damping[i], self.m_ii[i]
            out[n] = ((d / (2 * np.sqrt(k * m_ii)), np.sqrt(k / m_ii) / (2 * np.pi))
                      if k > 0 else (None, None))
        return out


def switch_request(activate, timeout=10.0):
    request = SwitchController.Request()
    if activate:
        request.activate_controllers = [dynamics.CONTROLLER]
    else:
        request.deactivate_controllers = [dynamics.CONTROLLER]
    request.strictness = SwitchController.Request.STRICT
    request.timeout = Duration(sec=int(timeout))
    return request


def reset_to_pose(node, names, pose):
    """
    Reset the sim to pose and let the controller hold it from there.

    The controller anchors its hold at the pose it is activated in, so it is deactivated first and
    activated again at the new pose. A paused sim does not run the controller manager, so the
    activation is requested while paused and completes in the first cycles after the unpause,
    before the limp arm can fall.
    """
    if controller_active(node):
        result = node.call(node.switch, switch_request(False), 15.0)
        if result is None or not result.ok:
            sys.exit('could not deactivate the controller')
    node.call(node.pause, SetPause.Request(paused=True), 5.0)
    request = ResetWorld.Request()
    request.state_overrides.joint_states.name = list(names)
    request.state_overrides.joint_states.position = [float(q) for q in pose]
    result = node.call(node.reset, request, 5.0)
    if result is None or not result.success:
        sys.exit(f'reset_world failed: {result.message if result else "timeout"}')
    activation = node.switch.call_async(switch_request(True))
    node.call(node.pause, SetPause.Request(paused=False), 5.0)
    rclpy.spin_until_future_complete(node, activation, timeout_sec=15.0)
    if activation.result() is None or not activation.result().ok:
        sys.exit('could not activate the controller after the reset')
    node.t.clear()
    node.q.clear()
    node.spin_until(node.now() + 1.0)


def controller_active(node):
    result = node.call(node.controllers, ListControllers.Request(), 5.0)
    controllers = result.controller if result else []
    return any(c.name == dynamics.CONTROLLER and c.state == 'active' for c in controllers)


def wrench_request(args, rotation):
    # the plugin applies force and torque in the body frame, although its description says world
    force, torque = rotation.T @ np.array(args.force), rotation.T @ np.array(args.torque)
    w = ExternalWrench()
    w.wrench.header.frame_id = args.body
    w.wrench.wrench.force.x, w.wrench.wrench.force.y, w.wrench.wrench.force.z = force.tolist()
    w.wrench.wrench.torque.x, w.wrench.wrench.torque.y, w.wrench.wrench.torque.z = torque.tolist()
    w.application_point.x, w.application_point.y, w.application_point.z = args.point
    w.duration = Duration(sec=int(args.duration), nanosec=int(round(args.duration % 1.0 * 1e9)))
    request = ApplyExternalWrench.Request()
    request.wrenches.external_wrenches = [w]
    return request


def free_response(t, e):
    """Overshoot, damping ratio, damped frequency and settling time of the decay e(t) -> 0."""
    # ~10 ms moving average, so sample noise does not count as extrema
    n = max(1, int(round(0.01 / np.median(np.diff(t)))))
    e = np.convolve(np.pad(e, (n // 2, n - 1 - n // 2), mode='edge'), np.ones(n) / n, 'valid')
    e0 = e[0]
    # extrema: sign changes of the slope, the first sample (release) counts as the first peak.
    # Extrema smaller than PEAK_MIN of the deflection at release are noise
    de = np.diff(e)
    idx = [0] + [i + 1 for i in range(len(de) - 1)
                 if de[i] * de[i + 1] < 0 and abs(e[i + 1]) > PEAK_MIN * abs(e0)]
    peaks = [(t[i], e[i]) for i in idx]
    opposite = [p for p in peaks if np.sign(p[1]) != np.sign(e0)]
    overshoot = abs(opposite[0][1]) / abs(e0) if opposite else 0.0

    same = [p for p in peaks[1:] if np.sign(p[1]) == np.sign(e0)]
    if opposite and same:
        # log decrement over one full period, independent of the velocity at release
        delta = np.log(abs(peaks[0][1]) / abs(same[0][1]))
        zeta = delta / np.sqrt(4 * np.pi ** 2 + delta ** 2)
        f_d = 1.0 / (same[0][0] - peaks[0][0])
    elif opposite:
        # half period only, assumes no velocity at release
        ln_os = np.log(overshoot)
        zeta = -ln_os / np.sqrt(np.pi ** 2 + ln_os ** 2)
        f_d = 0.5 / (opposite[0][0] - peaks[0][0])
    else:
        zeta, f_d = None, None

    outside = np.nonzero(np.abs(e) > SETTLE_BAND * abs(e0))[0]
    settle = t[outside[-1]] - t[0] if len(outside) else 0.0
    return overshoot, zeta, f_d, settle


def fmt(value, spec):
    return '–' if value is None else f'{value:{spec}}'


def warn_constrained(model):
    """Warn when the start pose rests on a joint limit or on the floor."""
    lo, hi = model.m.jnt_range[:, 0], model.m.jnt_range[:, 1]
    at_limit = [n for n, q, a, b in zip(model.names, model.q, lo, hi)
                if q - a < LIMIT_MARGIN or b - q < LIMIT_MARGIN]
    on_floor = model.bodies_on_floor(CONTACT_MARGIN)
    if at_limit:
        print(f'warning: {", ".join(at_limit)} at a position limit, it takes part of the push')
    if on_floor:
        print(f'warning: {", ".join(on_floor)} on the floor, it takes part of the push. '
              'Start from a free pose, see the README')


def print_push_torques(model, args):
    """Print the joint torques of the push, and warn when it cannot turn the body's own joint."""
    tau = model.joint_torques(args.body, args.point, args.force, args.torque)
    print('joint torques of the push [Nm]: '
          + ', '.join(f'{n} {t:+.3f}' for n, t in zip(model.names, tau)))
    joint = model.m.body_jntadr[model.body_id(args.body)]
    if joint >= 0 and abs(tau[joint]) < MIN_JOINT_TORQUE:
        print(f'warning: the wrench puts {tau[joint]:.1e} Nm on {model.names[joint]}, it acts '
              'on its axis and cannot move it. Use --point to add a lever or --torque.')


def print_results(names, t, q, t_push, t_release, rate, model, args):
    before = q[t < t_push]
    q_rest = before[-int(0.2 * rate) or -1:].mean(0)
    pushed = q[(t >= t_push) & (t < t_release)]
    after_mask = t >= t_release
    t_after, q_after = t[after_mask], q[after_mask]
    q_final = q_after[t_after > t_after[-1] - 0.3].mean(0)
    model.set_pose(names, q_final)
    exp = model.expected()

    print(f'\npush: {args.body}, force {args.force} N, torque {args.torque} Nm, '
          f'point {args.point} m, {args.duration} s, joint states {rate:.0f} Hz\n')
    print('| joint | max push deflection [rad] | anchor slid [rad] | at release [rad] | overshoot '
          '| ζ | f_d [Hz] | settled 5% [s] | ζ expected | f_n expected [Hz] |')
    print('|---|---|---|---|---|---|---|---|---|---|')
    for i, n in enumerate(names):
        e = q_after[:, i] - q_final[i]
        deflection = np.abs(pushed[:, i] - q_rest[i]).max() if len(pushed) else 0.0
        slid = q_final[i] - q_rest[i]
        zeta_exp, fn_exp = exp.get(n, (None, None))
        head = f'| {n} | {deflection:.4f} | {slid:+.4f} | {e[0]:+.4f} |'
        tail = f' {fmt(zeta_exp, ".2f")} | {fmt(fn_exp, ".1f")} |'
        if abs(e[0]) < MIN_DEFLECTION:
            print(head + ' – | – | – | – |' + tail)
        elif np.abs(e).max() > COUPLED * abs(e[0]):
            print(head + ' coupled | – | – | – |' + tail)
        else:
            overshoot, zeta, f_d, settle = free_response(t_after, e)
            zeta_txt = fmt(zeta, '.2f') if zeta is not None else '≥ 1'
            print(head + f' {overshoot:.0%} | {zeta_txt} | {fmt(f_d, ".1f")} | {settle:.2f} |'
                  + tail)
    print('\nanchor slid != 0: the push exceeded the breakaway torque, the hold moved.'
          '\ncoupled: moved further after the release than it was deflected, dragged along by the'
          '\nother joints or still carried by its momentum, so it has no clean decay to fit.'
          '\nThe joints are coupled, the per joint ζ of a combined motion is an estimate.')


def main(argv=None):
    args = parse_args(argv)
    model = Model(args.tool, args.params, args.controllers)
    rclpy.init()
    node = Recorder()

    if args.pose:
        reset_to_pose(node, model.names, args.pose)
    elif not controller_active(node):
        print(f'warning: {dynamics.CONTROLLER} is not active, the arm is limp')

    node.spin_until(node.now() + 0.5)
    if node.names is None:
        sys.exit('no /joint_states received')
    rate = (len(node.t) - 1) / (node.t[-1] - node.t[0]) if len(node.t) > 1 else 0.0
    if rate < 200:
        print(f'warning: joint states at {rate:.0f} Hz, too coarse to fit the response. '
              'Start the sim with joint_state_rate:=500')

    model.set_pose(node.names, node.q[-1])
    warn_constrained(model)
    print_push_torques(model, args)
    t_push = node.now()
    request = wrench_request(args, model.rotation(args.body))
    result = node.call(node.wrench, request, args.duration + 10.0)
    if result is None or not result.success:
        sys.exit(f'apply_wrench failed: {result.message if result else "timeout"}')
    t_release = t_push + args.duration
    node.spin_until(t_release + args.record)
    node.destroy_node()
    rclpy.shutdown()

    t, q, names = np.array(node.t), np.array(node.q), node.names
    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t'] + names)
            writer.writerows(np.column_stack([t - t_push, q]).tolist())
    print_results(names, t, q, t_push, t_release, rate, model, args)


if __name__ == '__main__':
    main()
