"""Record an exclusive guided zone sequence, or compare saved reports."""
import argparse
import json
from pathlib import Path
import sys
import time

from .cli import positive_seconds, wait_until
from .nav_compare_logic import comparison_table, goal_metrics


def main():
    import rclpy
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import remove_ros_args
    from nav_msgs.msg import Odometry
    from .node import ZoneNavigator

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stack', choices=['apt', 'fork'])
    parser.add_argument('--output', type=Path)
    parser.add_argument('--reports', nargs=2, type=Path, metavar=('APT_JSON', 'FORK_JSON'))
    parser.add_argument('--case-timeout', type=positive_seconds, default=180.0)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if args.reports:
        print(comparison_table(*(json.loads(p.read_text()) for p in args.reports)))
        return 0
    if not args.stack or not args.output:
        parser.error('Recording requires --stack and --output')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ZoneNavigator('nav_compare', enable_interfaces=False)
    report = dict(stack=args.stack, truth_topic='/sim/ground_truth_odom',
                  time_basis='simulation truth stamps', goals=[], complete=False)
    samples = []
    latest_wall = [0.0]

    def truth(msg):
        if msg.header.frame_id != 'sim_world':
            return
        stamp = msg.header.stamp
        samples.append((stamp.sec + stamp.nanosec*1e-9,
                        msg.pose.pose.position.x, msg.pose.pose.position.y))
        latest_wall[0] = time.monotonic()

    node.create_subscription(Odometry, '/sim/ground_truth_odom', truth, 10)
    def require(condition, message):
        if not condition:
            raise RuntimeError(message)

    try:
        require(node.get_parameter('use_sim_time').value, 'Simulation time is required')
        params = AsyncParameterClient(node, '/controller_server')
        require(params.wait_for_services(timeout_sec=15.0), 'controller_server unavailable')
        future = params.get_parameters(['controller_plugins'])
        require(wait_until(node, future.done, 5.0), 'Cannot verify running controller profile')
        plugins = future.result().values[0].string_array_value
        require(('Slow' in plugins) if args.stack == 'fork' else list(plugins) == ['FollowPath'],
                'Running stack does not match --stack')
        require(node.client.wait_for_server(timeout_sec=15.0), 'Nav2 unavailable')
        require(wait_until(node, lambda: bool(samples) and time.monotonic()-latest_wall[0] < 0.3,
                           10.0), 'Fresh simulation truth required; no odom fallback')
        for zone in node.zones:
            require(zone.frame_id == 'map', 'Comparison requires map-frame goals')
            samples[:] = samples[-1:]
            row = dict(zone=zone.name, result='ERROR', seconds=None, path_m=None, final_error_m=None)
            report['goals'].append(row)
            node.navigate(zone.name)
            settled = wait_until(node, lambda: not node.tracker.busy, args.case_timeout)
            row['result'] = node.tracker.state.value if settled else 'TIMEOUT'
            if not settled:
                node.cancel()
                require(wait_until(node, lambda: not node.tracker.busy, 10.0),
                        'Cancellation unconfirmed; stop simulation before continuing')
            completed = time.monotonic()
            require(wait_until(node, lambda: latest_wall[0] > completed, 3.0),
                    'Missing post-result truth')
            row.update(goal_metrics(samples, (zone.x, zone.y)))
            print(json.dumps(row), flush=True)
        report['complete'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc)
        print(f'FAIL: {exc}', file=sys.stderr)
    finally:
        if node.tracker.busy:
            node.cancel()
            if not wait_until(node, lambda: not node.tracker.busy, 10.0):
                report['cleanup_error'] = 'Cancellation unconfirmed; stop simulation'
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    return 0 if (report['complete'] and not report.get('cleanup_error')
                 and all(g['result'] == 'SUCCEEDED' for g in report['goals'])) else 1
