"""Run destructive-to-current-navigation checks only in an exclusive simulator session."""
import argparse
import math
from pathlib import Path
import time

from .cli import positive_seconds, wait_until
from .goal_tracker import State
from .zones import Zone
from .nav_check_logic import (navfn_tolerance, store_bounds, unreachable_clearances,
                              final_position_detail)


def main():
    import sys
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import Odometry, OccupancyGrid
    from rclpy.utilities import remove_ros_args
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from ament_index_python.packages import get_package_share_directory
    from .node import ZoneNavigator

    parser = argparse.ArgumentParser(description='Exclusive simulation navigation checks')
    parser.add_argument('--case-timeout', type=positive_seconds, default=120.0)
    parser.add_argument('--tolerance-check', action='store_true')
    parser.add_argument('--max-final-error', type=positive_seconds, default=0.35,
                        help='Maximum zone position error in metres (default: 0.35)')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # Exercise the same adapter without duplicating the persistent command/service endpoints.
    node = ZoneNavigator('nav_action_check', enable_interfaces=False)
    rows = []
    odometry = []
    distances = []
    costmaps = []
    node.progress = lambda distance: distances.append(distance)

    def odom(msg):
        if msg.header.frame_id != 'odom':
            return
        twist = msg.twist.twist
        odometry.append((time.monotonic(), msg.pose.pose.position.x,
                         msg.pose.pose.position.y,
                         max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z))))
        del odometry[:-100]

    node.create_subscription(Odometry, '/odom', odom, 10)
    node.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                             lambda msg: costmaps.__setitem__(slice(None), [msg]),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def require(condition, message):
        if not condition:
            raise RuntimeError(message)

    def settled():
        return not node.tracker.busy

    def wait_settled():
        require(wait_until(node, settled, args.case_timeout),
                f'goal did not settle: {node.tracker.status}')

    def far_zone():
        require(bool(odometry), 'no fresh /odom')
        _, x, y, _ = odometry[-1]
        return max(node.zones[:6], key=lambda z: math.hypot(z.x-x, z.y-y))

    def unknown():
        count = node.send_count
        node.navigate('__unknown_zone_check__')
        require(node.tracker.status.code == 'UNKNOWN_ZONE' and node.send_count == count,
                'unknown zone reached action transport')

    def unreachable():
        world = Path(get_package_share_directory('shopping_robot_gazebo')) / 'worlds/supermarket.sdf'
        config = Path(get_package_share_directory('shopping_robot_navigation')) / 'config/nav2_params.yaml'
        tolerance = navfn_tolerance(config)
        require(bool(costmaps), 'no global costmap')
        grid = costmaps[-1]
        origin = grid.info.origin
        require(grid.header.frame_id == 'map' and grid.info.resolution > 0,
                'expected map-frame global costmap with positive resolution')
        require(origin.orientation.x == origin.orientation.y == origin.orientation.z == 0
                and abs(origin.orientation.w) == 1, 'expected axis-aligned global costmap')
        bounds = (origin.position.x, origin.position.y,
                  origin.position.x + grid.info.width * grid.info.resolution,
                  origin.position.y + grid.info.height * grid.info.resolution)
        target = Zone('__unreachable_check__', (), 50.0, 50.0, 0.0, 'map')
        clearances = unreachable_clearances(target.x, target.y, tolerance, store_bounds(world), bounds)
        node.zones.append(target)
        node.navigate(target.name)
        wait_settled()
        require(node.tracker.state in (State.ABORTED, State.REJECTED),
                f'unreachable goal returned {node.tracker.state.value}')
        return f'NavFn tolerance={tolerance:.3f} m; minimum clearance={min(clearances):.3f} m'

    def canceled():
        distances.clear()
        target = far_zone()
        start_x, start_y = odometry[-1][1:3]
        node.navigate(target.name)
        require(wait_until(node, lambda: node.tracker.state == State.ACTIVE, 10.0), 'not ACTIVE')

        def progressed():
            return (len(distances) >= 2 and max(distances[:-1]) - distances[-1] > 0.15
                    and math.hypot(odometry[-1][1] - start_x, odometry[-1][2] - start_y) > 0.1)

        require(wait_until(node, progressed, args.case_timeout), 'no navigation progress')
        started = time.monotonic()
        node.cancel()

        def stopped():
            samples = [sample for sample in odometry if sample[0] >= started]
            return (len(samples) >= 2 and samples[-1][0] - samples[0][0] >= 0.3
                    and time.monotonic() - samples[-1][0] < 0.3
                    and all(sample[3] < 0.02 for sample in samples[-2:]))

        require(wait_until(node, lambda: settled() and stopped(), 2.0),
                'no final result and fresh near-zero odom twist within 2 s of cancel')
        require(node.tracker.state == State.CANCELED, f'expected CANCELED: {node.tracker.status}')

    def succeeded():
        node.navigate(far_zone().name)
        wait_settled()
        require(node.tracker.state == State.SUCCEEDED, str(node.tracker.status))

    def tolerance_check():
        target = far_zone()
        require(target.frame_id == 'map', 'tolerance check requires a map-frame zone')
        node.navigate(target.name)
        wait_settled()
        require(node.tracker.state == State.SUCCEEDED, str(node.tracker.status))
        completed = time.monotonic()
        require(wait_until(node, lambda: bool(odometry) and odometry[-1][0] > completed
                           and time.monotonic() - odometry[-1][0] < 0.3, 5.0),
                'no fresh post-result /odom')
        return final_position_detail(*odometry[-1][1:3], target, args.max_final_error)

    def busy():
        target = far_zone()
        node.navigate(target.name)
        require(wait_until(node, lambda: node.tracker.state == State.ACTIVE, 10.0), 'not ACTIVE')
        count = node.send_count
        node.navigate(next(z.name for z in node.zones[:6] if z != target), preempt=False)
        require(node.tracker.status.code == 'BUSY' and node.send_count == count,
                'second goal was sent instead of BUSY')
        node.cancel()
        wait_settled()

    checks = [('unknown zone', unknown), ('unreachable goal', unreachable),
              ('cancel and stop', canceled), ('valid goal', succeeded), ('busy policy', busy)]
    if args.tolerance_check:
        checks.append(('zone tolerance', tolerance_check))

    try:
        require(node.client.wait_for_server(timeout_sec=15.0), 'Nav2 unavailable')
        require(wait_until(node, lambda: bool(odometry), 5.0), 'no /odom; start guide.launch')
        require(wait_until(node, lambda: bool(costmaps), 5.0), 'no global costmap; start guide.launch')
        for name, check in checks:
            try:
                detail = check()
                rows.append((name, 'PASS', detail or ''))
            except Exception as exc:
                rows.append((name, 'FAIL', str(exc)))
            print(f'{rows[-1][1]}: {name} {rows[-1][2]}', flush=True)
            if node.tracker.busy:
                node.cancel()
                if not wait_until(node, settled, 10.0):
                    for remaining, _ in checks[len(rows):]:
                        rows.append((remaining, 'FAIL', 'previous goal did not settle'))
                    break
    except (Exception, KeyboardInterrupt) as exc:
        rows.append(('setup/interruption', 'FAIL', str(exc)))
    finally:
        if node.tracker.busy:
            node.cancel()
            if not wait_until(node, settled, 10.0):
                rows.append(('cleanup', 'FAIL', 'cancellation unconfirmed; inspect Nav2'))
        print('\n| Check | Result | Detail |\n| --- | --- | --- |')
        for name, result, detail in rows:
            print(f'| {name} | {result} | {detail} |')
        node.destroy_node()
        rclpy.shutdown()
    return 0 if len(rows) == len(checks) and all(row[1] == 'PASS' for row in rows) else 1
