#!/usr/bin/env python3
"""Measure simulation truth/EKF motion while driving the plain sim command topic."""
import argparse
from dataclasses import dataclass
import math
import signal
import statistics
import time

COMMANDS = ((0.0, 0.2), (0.0, 0.4), (0.0, 0.6), (0.0, 0.8), (0.2, 0.3), (0.4, 0.3))


@dataclass(frozen=True)
class Sample:
    stamp: float
    x: float
    y: float
    yaw: float


def summarize(samples, duration, max_gap=0.3):
    """Fit unwrapped heading against source time, and measure planar path speed."""
    if len(samples) < 5 or not math.isfinite(duration) or duration <= 0:
        raise ValueError('Insufficient samples or invalid duration')
    if any(not all(math.isfinite(v) for v in (s.stamp, s.x, s.y, s.yaw)) for s in samples):
        raise ValueError('Non-finite odometry')
    gaps = [b.stamp-a.stamp for a, b in zip(samples, samples[1:])]
    span = samples[-1].stamp-samples[0].stamp
    if min(gaps) <= 0 or max(gaps) > max_gap or span < duration*0.8:
        raise ValueError('Stale, unordered or insufficient odometry coverage')
    headings = [samples[0].yaw]
    for a, b in zip(samples, samples[1:]):
        headings.append(headings[-1]+math.atan2(math.sin(b.yaw-a.yaw), math.cos(b.yaw-a.yaw)))
    ts = [s.stamp-samples[0].stamp for s in samples]
    mt, mh = statistics.mean(ts), statistics.mean(headings)
    rate = sum((t-mt)*(h-mh) for t, h in zip(ts, headings))/sum((t-mt)**2 for t in ts)
    speed = sum(math.hypot(b.x-a.x, b.y-a.y) for a, b in zip(samples, samples[1:]))/span
    return rate, speed


def fit_factor(old_factor, commands_and_rates):
    """Return both parameter conventions, using only valid in-place ratios.

    Fixed wheel-rate odometry: new separation = old * ratio.
    Fortress command-to-wheel separation: new factor ~= old / ratio.
    Neither relation models friction breakaway or speed-dependent scrub.
    """
    if not math.isfinite(old_factor) or old_factor <= 0:
        raise ValueError('Invalid old factor')
    ratios = []
    for linear, angular, measured in commands_and_rates:
        if not all(map(math.isfinite, (linear, angular, measured))):
            raise ValueError('Non-finite measurement')
        if linear != 0:
            continue
        if angular == 0 or measured/angular <= 0:
            raise ValueError('Zero command, no rotation or reversed rotation; do not fit')
        ratios.append(measured/angular)
    if not ratios:
        raise ValueError('No in-place measurements')
    ratio = statistics.mean(ratios)
    return {'mean_ratio': ratio, 'min_ratio': min(ratios), 'max_ratio': max(ratios),
            'fixed_wheel_rate_separation': old_factor*ratio,
            'fortress_command_factor': old_factor/ratio}


def parse_linear_sweep(value):
    """Parse positive finite arc speeds without starting ROS."""
    try:
        speeds = tuple(float(part.strip()) for part in value.split(','))
        if not speeds or any(not math.isfinite(v) or v <= 0 for v in speeds):
            raise ValueError
        return speeds
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Expected comma-separated finite positive speeds") from exc


def sweep_commands(speeds=(), in_place_only=False):
    return COMMANDS[:4] if in_place_only else COMMANDS+tuple((v, 0.3) for v in speeds)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--linear-sweep', type=parse_linear_sweep, default=(),
                        help='Append moving arcs at comma-separated linear speeds (m/s)')
    parser.add_argument('--in-place-only', action='store_true', help='Skip moving arcs')
    parser.add_argument('--settle', type=float, default=2.0)
    parser.add_argument('--sample', type=float, default=3.0)
    parser.add_argument('--old-factor', type=float, default=2.0)
    parser.add_argument('--wall-timeout', type=float, default=45.0, help='Wall seconds per command')
    parser.add_argument('--clearance', type=float, default=0.6, help='Abort below this scan range (m)')
    args = parser.parse_args(argv)
    if any(not math.isfinite(v) or v <= 0 for k, v in vars(args).items() if k not in ('in_place_only', 'linear_sweep')):
        parser.error('All arguments must be finite and positive')
    # ROS imports belong only to the executable entry point.
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    rclpy.init(args=[])
    # Keep the ROS context alive on Ctrl-C so the finally block can send zero.
    previous = signal.getsignal(signal.SIGINT)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    node = Node('yaw_rate_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    samples = {'truth': [], 'ekf': []}
    received = {}
    scan_state = {}

    def odom(msg, key):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if key == 'truth' and (msg.header.frame_id, msg.child_frame_id) != ('sim_world', 'base_footprint_gt'):
            raise ValueError('Expected independent sim_world/base_footprint_gt truth')
        s = Sample(stamp, p.x, p.y, yaw)
        if samples[key] and s.stamp <= samples[key][-1].stamp:
            raise ValueError('Odometry clock reset or duplicate stamp')
        samples[key].append(s)
        samples[key][:] = samples[key][-2000:]
        received[key] = time.monotonic()

    def scan(msg):
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        scan_state.update(received=time.monotonic(), stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                          minimum=min(valid, default=math.inf), valid=bool(valid))

    subscriptions = [node.create_subscription(Odometry, '/sim/ground_truth_odom', lambda m: odom(m, 'truth'), qos_profile_sensor_data),
                     node.create_subscription(Odometry, '/odom', lambda m: odom(m, 'ekf'), qos_profile_sensor_data),
                     node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)]

    def now():
        return node.get_clock().now().nanoseconds*1e-9

    def check_fresh():
        for key in samples:
            if key not in received or time.monotonic()-received[key] > 0.5 or not 0 <= now()-samples[key][-1].stamp <= 0.5:
                raise ValueError(f'Missing/stale {key} odometry')
        if (not scan_state.get('valid') or time.monotonic()-scan_state['received'] > 0.5
                or not 0 <= now()-scan_state['stamp'] <= 0.5):
            raise ValueError('Missing/stale scan')
        if scan_state['minimum'] < args.clearance:
            raise ValueError('Obstacle clearance reached; restart/reposition in sim before retrying')

    def drive(linear, angular, seconds):
        start, wall = now(), time.monotonic()
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        while now()-start < seconds:
            if now() < start or time.monotonic()-wall > args.wall_timeout:
                raise ValueError('Simulation clock reset/stalled or wall timeout')
            check_fresh()
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
        return start

    status = 0
    results = []
    try:
        print('Plain sim only: publishing /cmd_vel directly; no mux/watchdog. No other command producers.')
        deadline = time.monotonic()+15
        while not (all(samples.values()) and scan_state and publisher.get_subscription_count()):
            if time.monotonic() > deadline:
                raise ValueError('Timed out waiting for simulation data/command subscriber')
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        if any(n in node.get_node_names() for n in ('cmd_vel_mux', 'cmd_vel_watchdog')):
            raise ValueError('Mux/watchdog detected: use a fresh plain sim launch')
        print(' cmd_v cmd_w | truth_w truth_speed yaw_ratio speed_ratio | ekf_w ekf_speed yaw_ratio speed_ratio')
        for linear, angular in sweep_commands(args.linear_sweep, args.in_place_only):
            drive(0.0, 0.0, 1.0)
            drive(linear, angular, args.settle)
            start = drive(linear, angular, args.sample)
            rates = [summarize([s for s in samples[key] if start <= s.stamp <= now()], args.sample) for key in ('truth', 'ekf')]
            results.append((linear, angular, rates[0][0]))
            fields = []
            for yaw, speed in rates:
                speed_ratio = f'{speed/linear:.3f}' if linear else 'n/a'
                fields.append(f'{yaw:7.3f} {speed:11.3f} {yaw/angular:9.3f} {speed_ratio:>11}')
            print(f'{linear:6.2f} {angular:5.2f} | '+ ' | '.join(fields), flush=True)
    except KeyboardInterrupt:
        print('Interrupted; publishing zero.')
        status = 130
    except (ValueError, RuntimeError) as exc:
        print(f'ABORT: {exc}; publishing zero.')
        status = 1
    finally:
        # Repeated zeros give the bridge time to forward the stop before teardown.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            for _ in range(10):
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            signal.signal(signal.SIGINT, previous)
    if len([r for r in results if r[0] == 0]) == 4:
        try:
            print('Fit (four in-place rows): '+str(fit_factor(args.old_factor, results)))
            print('Use fortress_command_factor as an unverified trial; fixed_wheel_rate_separation is NOT the xacro correction.')
            print('Friction breakaway and moving skid steering invalidate a single-factor model; confirm every row before changing defaults.')
        except ValueError as exc:
            print(f'No valid fit: {exc}')
            status = status or 1
    return status


if __name__ == '__main__':
    raise SystemExit(main())
