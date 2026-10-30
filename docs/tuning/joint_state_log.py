#!/usr/bin/env python3
"""
Log full rate joint states to a csv and look at the spectrum afterwards, for finding joint oscillations.

record: loads/configures/activates joint_state_broadcaster_hires (controllers.yaml, 500 Hz, not spawned at
startup), writes every message of /joint_state_broadcaster_hires/joint_states to a csv until Ctrl-C or
--duration, then deactivates the broadcaster again. Needs a sourced workspace and a running controller manager.

    python3 docs/tuning/joint_state_log.py record [-o out.csv] [--duration 30]

analyze: reads such a csv, reports sample gaps, resamples to a uniform grid, high-passes (removes the motion
itself), and prints/plots the amplitude spectrum of position, velocity and effort per joint. Needs only numpy,
matplotlib for the plots (without it only the peaks are printed).

    python3 docs/tuning/joint_state_log.py analyze log.csv [--joints axis2 axis3] [--start 5 --end 20]
"""

import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np

BROADCASTER = 'joint_state_broadcaster_hires'
SIGNALS = ['position', 'velocity', 'effort']


# ---------------------------------------------------------------------------------------------------- record

def call(node, client, request, timeout=5.0):
    import rclpy
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f'service {client.srv_name} not available, is the controller manager running?')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.result() is None:
        raise RuntimeError(f'service {client.srv_name} timed out')
    return future.result()


class BroadcasterSwitch:
    """Brings the hires broadcaster to active and back to inactive through the controller manager services."""

    def __init__(self, node, cm):
        from controller_manager_msgs import srv
        self.node = node
        self.srv = srv
        self.list = node.create_client(srv.ListControllers, f'{cm}/list_controllers')
        self.load = node.create_client(srv.LoadController, f'{cm}/load_controller')
        self.configure = node.create_client(srv.ConfigureController, f'{cm}/configure_controller')
        self.switch = node.create_client(srv.SwitchController, f'{cm}/switch_controller')

    def state(self):
        for c in call(self.node, self.list, self.srv.ListControllers.Request()).controller:
            if c.name == BROADCASTER:
                return c.state
        return None

    def set_active(self, active):
        req = self.srv.SwitchController.Request()
        if active:
            req.activate_controllers = [BROADCASTER]
        else:
            req.deactivate_controllers = [BROADCASTER]
        req.strictness = self.srv.SwitchController.Request.STRICT
        if not call(self.node, self.switch, req).ok:
            raise RuntimeError(f'switching {BROADCASTER} to {"active" if active else "inactive"} failed')

    def activate(self):
        """Returns True if this call activated it (and it should be deactivated again), False if it already was."""
        state = self.state()
        if state is None:
            req = self.srv.LoadController.Request(name=BROADCASTER)
            if not call(self.node, self.load, req).ok:
                raise RuntimeError(f'loading {BROADCASTER} failed, is it in controllers.yaml?')
            state = self.state()
        if state == 'unconfigured':
            req = self.srv.ConfigureController.Request(name=BROADCASTER)
            if not call(self.node, self.configure, req).ok:
                raise RuntimeError(f'configuring {BROADCASTER} failed')
            state = self.state()
        if state == 'active':
            return False
        self.set_active(True)
        return True


def record(args):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState

    out = Path(args.output or time.strftime('joint_states_%Y%m%d_%H%M%S.csv'))
    # own Ctrl-C handling: the default handler shuts the context down, then the deactivate call would fail
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('joint_state_log')
    switch = BroadcasterSwitch(node, args.controller_manager)

    rows = 0
    joints = None
    f = out.open('w', newline='')
    writer = csv.writer(f)

    def on_msg(msg):
        nonlocal rows, joints
        if joints is None:
            joints = list(msg.name)
            writer.writerow(['stamp'] + [f'{j}_{s}' for j in joints for s in SIGNALS])
        index = {n: i for i, n in enumerate(msg.name)}
        row = [msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9]
        for j in joints:
            i = index.get(j)
            for values in (msg.position, msg.velocity, msg.effort):
                # interfaces the hardware does not export come as empty arrays
                row.append(values[i] if i is not None and i < len(values) else math.nan)
        writer.writerow(row)
        rows += 1

    topic = f'/{BROADCASTER}/joint_states'
    sub = node.create_subscription(JointState, topic, on_msg,
                                   QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))

    activated = False
    try:
        activated = switch.activate()
        print(f'recording {topic} -> {out}, Ctrl-C to stop')
        t_end = time.monotonic() + args.duration if args.duration > 0 else math.inf
        t_report = time.monotonic() + 5.0
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() > t_report:
                print(f'  {rows} samples')
                t_report += 5.0
                f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        # the deactivate call below spins the node, queued messages must not reach the closed file
        node.destroy_subscription(sub)
        f.close()
        if activated:
            try:
                switch.set_active(False)
            except RuntimeError as e:
                print(f'WARNING: {e}, deactivate it by hand: '
                      f'ros2 control set_controller_state {BROADCASTER} inactive')
        node.destroy_node()
        rclpy.shutdown()
    print(f'{rows} samples written to {out}')
    if rows:
        print(f'analyze with: python3 {Path(__file__).as_posix()} analyze {out}')


# --------------------------------------------------------------------------------------------------- analyze

def load_csv(path):
    with open(path, newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        data = np.array([[float(v) if v != '' else math.nan for v in row] for row in reader])
    if data.size == 0:
        raise SystemExit(f'{path}: no samples')
    joints = list(dict.fromkeys(h.rsplit('_', 1)[0] for h in header[1:]))
    cols = {h: data[:, i] for i, h in enumerate(header)}
    return cols['stamp'], joints, cols


def highpass(x, dt, f_cut):
    """Subtracts a centered moving average of 1/f_cut length, i.e. removes the slow commanded motion."""
    if f_cut <= 0:
        return x - np.mean(x)
    n = max(3, int(round(1.0 / (f_cut * dt))) | 1)
    if n >= len(x):
        return x - np.mean(x)
    pad = n // 2
    # odd reflection continues a ramp through the edges, edge padding would leave a step there
    xp = np.pad(x, pad, mode='reflect', reflect_type='odd')
    return x - np.convolve(xp, np.ones(n) / n, mode='valid')


def spectrum(x, dt):
    """Single sided amplitude spectrum with a Hann window, amplitude of a sine reads as its amplitude."""
    w = np.hanning(len(x))
    amp = 2.0 * np.abs(np.fft.rfft(x * w)) / np.sum(w)
    return np.fft.rfftfreq(len(x), dt), amp


def peaks(freq, amp, f_min, count):
    """The `count` largest local maxima above f_min, as (freq, amp), largest first."""
    i = np.arange(1, len(amp) - 1)
    is_peak = (amp[i] > amp[i - 1]) & (amp[i] >= amp[i + 1]) & (freq[i] >= f_min)
    idx = i[is_peak]
    idx = idx[np.argsort(amp[idx])[::-1][:count]]
    return [(freq[k], amp[k]) for k in idx]


def analyze(args):
    stamp, joints, cols = load_csv(args.csv)
    t = stamp - stamp[0]

    # timing: the realtime publisher drops a message while the previous one is still being sent
    dt_raw = np.diff(t)
    dt = float(np.median(dt_raw))
    gaps = dt_raw > 1.5 * dt
    print(f'{args.csv}: {len(t)} samples over {t[-1]:.2f} s, median dt {dt * 1e3:.2f} ms ({1 / dt:.0f} Hz)')
    print(f'gaps > 1.5 dt: {int(gaps.sum())}, {int(np.round(dt_raw[gaps] / dt).sum() - gaps.sum())} samples '
          f'missing, largest {dt_raw.max() * 1e3:.1f} ms (filled by linear interpolation)')

    t_end = args.end if args.end is not None else t[-1]
    grid = np.arange(args.start, t_end, dt)
    if len(grid) < 16:
        raise SystemExit('time window too short')
    selected = args.joints or joints
    unknown = set(selected) - set(joints)
    if unknown:
        raise SystemExit(f'unknown joints {sorted(unknown)}, the csv has {joints}')
    nyquist = 0.5 / dt
    print(f'window {grid[0]:.2f} – {grid[-1]:.2f} s, resolution {1 / (len(grid) * dt):.2f} Hz, '
          f'Nyquist {nyquist:.0f} Hz, high-pass {args.highpass:g} Hz\n')

    results = {}
    for j in selected:
        for s in SIGNALS:
            raw = cols[f'{j}_{s}']
            ok = np.isfinite(raw)
            if ok.sum() < 16:
                continue
            x = np.interp(grid, t[ok], raw[ok])
            xf = highpass(x, dt, args.highpass)
            freq, amp = spectrum(xf, dt)
            results[(j, s)] = (x, xf, freq, amp)
            unit = {'position': 'rad', 'velocity': 'rad/s', 'effort': 'Nm'}[s]
            found = ', '.join(f'{f:.1f} Hz ({a:.3g})' for f, a in peaks(freq, amp, args.highpass, args.peaks))
            print(f'{j:>6} {s:<8} rms {np.sqrt(np.mean(xf ** 2)):.3g} {unit:<5} peaks: {found}')

    if args.no_plot:
        return
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('\nmatplotlib not installed, no plots (pip install matplotlib)')
        return

    for j in selected:
        sigs = [s for s in SIGNALS if (j, s) in results]
        if not sigs:
            continue
        fig, axes = plt.subplots(len(sigs), 2, figsize=(14, 3 * len(sigs)), squeeze=False)
        fig.suptitle(f'{j} – {Path(args.csv).name}')
        for row, s in enumerate(sigs):
            x, xf, freq, amp = results[(j, s)]
            ax = axes[row][0]
            ax.plot(grid, x, lw=0.8, label='raw')
            ax2 = ax.twinx()
            ax2.plot(grid, xf, lw=0.6, color='tab:orange', label=f'high-pass {args.highpass:g} Hz')
            ax.set_ylabel(s)
            ax.set_xlabel('t [s]')
            ax.legend(loc='upper left', fontsize=8)
            ax2.legend(loc='upper right', fontsize=8)
            ax = axes[row][1]
            ax.semilogy(freq[1:], amp[1:], lw=0.8)
            ax.set_xlim(0, min(args.fmax, nyquist) if args.fmax > 0 else nyquist)
            ax.set_xlabel('f [Hz]')
            ax.set_ylabel(f'{s} amplitude')
            ax.grid(True, which='both', alpha=0.3)
        fig.tight_layout()
        png = Path(args.csv).with_name(f'{Path(args.csv).stem}_{j}.png')
        fig.savefig(png, dpi=120)
        print(f'saved {png}')
    if not args.no_show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)

    p = sub.add_parser('record', help='activate the hires broadcaster and log to csv')
    p.add_argument('-o', '--output', help='csv path (default: joint_states_<date>_<time>.csv)')
    p.add_argument('--duration', type=float, default=0.0, help='s, 0 = until Ctrl-C')
    p.add_argument('--controller-manager', default='/controller_manager', help='controller manager node')
    p.set_defaults(func=record)

    p = sub.add_parser('analyze', help='spectrum of a recorded csv')
    p.add_argument('csv')
    p.add_argument('--joints', nargs='+', help='default: all')
    p.add_argument('--start', type=float, default=0.0, help='s from the first sample')
    p.add_argument('--end', type=float, help='s from the first sample, default: last sample')
    p.add_argument('--highpass', type=float, default=0.5,
                   help='Hz, removes the commanded motion below this, 0 = only the mean')
    p.add_argument('--peaks', type=int, default=3, help='number of spectrum peaks printed per signal')
    p.add_argument('--fmax', type=float, default=0.0, help='Hz, x axis limit of the spectra, 0 = Nyquist')
    p.add_argument('--no-plot', action='store_true', help='only print, no matplotlib')
    p.add_argument('--no-show', action='store_true', help='save the pngs without opening a window')
    p.set_defaults(func=analyze)

    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
