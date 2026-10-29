"""Simulation check: a confidently wrong AMCL pose must not yield a trusted SUCCEEDED.

Publishes a wrong /initialpose (offset from simulation truth, small covariance),
sends a zone goal through ZoneNavigator and compares the outcome with truth.
Truth is used only by this check, never by the navigator.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter

from .goal_tracker import State
from .zones import lookup


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--zone', default='snacks')
    parser.add_argument('--offset', default='1.5,0.5,1.0', help='dx,dy,dyaw of the wrong pose')
    parser.add_argument('--timeout', type=float, default=150.0)
    parser.add_argument('--disable-guards', action='store_true',
                        help='baseline: disable the scan/map and map->odom jump guards')
    parser.add_argument('--max-final-error', type=float, default=0.35)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    from .node import ZoneNavigator
    node = ZoneNavigator('kidnap_check', enable_interfaces=False)
    if args.disable_guards:
        node.set_parameters([Parameter('arrival_min_scan_agreement', value=-1.0)])
        node.min_scan_agreement = -1.0
        node.jump_monitor.max_xy_step = node.jump_monitor.max_yaw_step = float('inf')
    truth = []
    node.create_subscription(Odometry, '/sim/ground_truth_odom',
                             lambda m: truth.append(m.pose.pose), 10)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def spin_until(cond, timeout):
        end = time.monotonic()+timeout
        while time.monotonic() < end and not cond():
            rclpy.spin_once(node, timeout_sec=0.1)
        return cond()

    if not spin_until(lambda: truth and node.client.server_is_ready() and pub.get_subscription_count(), 60):
        print('FAIL: no truth / action server / AMCL'); return 2
    p = truth[-1]
    yaw = 2*math.atan2(p.orientation.z, p.orientation.w)
    dx, dy, dyaw = (float(v) for v in args.offset.split(','))
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x, msg.pose.pose.position.y = p.position.x+dx, p.position.y+dy
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin((yaw+dyaw)/2), math.cos((yaw+dyaw)/2)
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
    msg.pose.covariance[35] = 0.01
    pub.publish(msg)
    spin_until(lambda: False, 4.0)
    node.navigate(args.zone)
    terminal = {State.SUCCEEDED, State.ABORTED, State.CANCELED, State.TIMEOUT,
                State.REJECTED, State.UNAVAILABLE}
    spin_until(lambda: node.tracker.state in terminal, args.timeout)
    spin_until(lambda: False, 1.0)
    zone = lookup(node.zones, args.zone)
    p = truth[-1]
    err = math.hypot(p.position.x-zone.x, p.position.y-zone.y)
    yerr = abs(math.atan2(math.sin(2*math.atan2(p.orientation.z, p.orientation.w)-zone.yaw),
                          math.cos(2*math.atan2(p.orientation.z, p.orientation.w)-zone.yaw)))
    state = node.tracker.state
    false_success = state == State.SUCCEEDED and err > args.max_final_error
    print(f'state={state} truth_error={err:.3f} m yaw_error={yerr:.3f} rad '
          f'guards={"off" if args.disable_guards else "on"} agreement={node.scan_agreement()}')
    print('RESULT:', 'FAIL (false SUCCEEDED)' if false_success else 'PASS (no false SUCCEEDED)')
    node.destroy_node(); rclpy.try_shutdown()
    return 1 if false_success else 0


if __name__ == '__main__':
    raise SystemExit(main())
