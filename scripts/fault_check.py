#!/usr/bin/env python3
"""Exercise an already running opt-in follow simulation; never publish velocity."""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/
                       'ros2_ws/src/shopping_robot_gazebo'))
from shopping_robot_sim.fault_check import Sample, evaluate_trial, moving


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-timeout', type=float, default=90.0,
                        help='Wall seconds allowed to find baseline/recovery motion')
    parser.add_argument('--suite', choices=['follow', 'obstacle'], default='follow')
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import String
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = Node('fault_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    samples = {'cmd': [], 'truth': []}
    state = {'status': None, 'clock': None}
    guard_samples = []
    def now():
        return node.get_clock().now().nanoseconds/1e9
    def record(key, twist):
        stamp = now()
        sample = Sample(stamp, twist.linear.x, twist.angular.z)
        # Multiple publishers may deliver commands at one clock tick. Preserve
        # motion over zero so a competing nonzero command cannot be hidden.
        if samples[key] and samples[key][-1].time == stamp:
            if moving(sample):
                samples[key][-1] = sample
        else:
            samples[key].append(sample)
    def status(msg):
        try:
            state['status'] = json.loads(msg.data)
        except ValueError:
            state['status'] = None
    def guard(msg):
        try:
            data = json.loads(msg.data)
            guard_samples.append(data)
        except (ValueError, KeyError, TypeError):
            pass
    node.create_subscription(String, '/safety/obstacle_state', guard, 100)
    def clock(msg):
        state['clock'] = msg.clock.sec+msg.clock.nanosec/1e9
    node.create_subscription(String, '/sim/fault_status', status, QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel', lambda m: record('cmd', m), 100)
    node.create_subscription(Odometry, '/sim/ground_truth_odom',
                             lambda m: record('truth', m.twist.twist), qos_profile_sensor_data)
    publisher = node.create_publisher(String, '/sim/fault', 10)
    def wait(predicate, timeout):
        until = time.monotonic()+timeout
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return True
        return False
    def send(command, expected):
        state['status'] = None
        publisher.publish(String(data=command))
        if not wait(lambda: state['status'] == expected, 5):
            raise RuntimeError('No fault status acknowledgement: '+command)
    def motion():
        return all(any(now()-1 <= s.time <= now() and moving(s) for s in samples[k])
                   and samples[k] and moving(samples[k][-1])
                   and now()-samples[k][-1].time < 0.25 for k in samples)
    results = []
    owns_faults = False
    try:
        if not wait(lambda: state['status'] is not None and state['clock'] is not None
                    and publisher.get_subscription_count() > 0, 10):
            raise RuntimeError('Requires /clock and opt-in /sim/fault_status')
        if not wait(lambda: len(node.get_publishers_info_by_topic('/cmd_vel')) == 1
                    and node.get_publishers_info_by_topic('/cmd_vel')[0].node_name
                    == 'cmd_vel_watchdog', 10):
            raise RuntimeError('/cmd_vel must have exactly one publisher: cmd_vel_watchdog')
        if state['status']:
            raise RuntimeError('Clear existing faults before running the check')
        owns_faults = True
        print('Fault | Result | cmd stop (s) | truth stop (s) | thresholds / reason', flush=True)
        faults = (('lidar_front:0.9', 'lidar_front:0.2', 'lidar_drop', 'lidar_freeze')
                  if args.suite == 'obstacle' else
                  ('uwb_stale', 'uwb_dropout:6.0', 'uwb_noise:0.5',
                   'lidar_drop', 'lidar_freeze', 'odom_freeze', 'lidar_front:0.2'))
        for fault in faults:
            if not wait(motion, args.motion_timeout):
                results.append(False)
                print(f'{fault} | FAIL | - | - | no moving baseline', flush=True)
                continue
            baseline = {k: [s for s in v if s.time >= now()-1] for k, v in samples.items()}
            start = now()
            send(fault, [fault.split(':')[0]])
            if not wait(lambda: now()-start >= 4.0, 60):
                raise RuntimeError('Simulation clock stopped or too slow')
            end = now()
            during = {k: [s for s in v if start <= s.time <= end] for k, v in samples.items()}
            send('clear', [])
            recovered = wait(lambda: now() > end+0.3 and motion()
                             and all(any(s.time > end+0.3 and moving(s) for s in v)
                                     for v in samples.values()), args.motion_timeout)
            recovery = {k: [s for s in v if s.time > end+0.3] if recovered else []
                        for k, v in samples.items()}
            ok, cmd, gt, reason = evaluate_trial(
                during['cmd'], during['truth'], baseline['cmd'], baseline['truth'],
                recovery['cmd'], recovery['truth'], start, end, noise=fault.startswith('uwb_noise'))
            if fault == 'lidar_front:0.9':
                observed = [g for g in guard_samples if start <= g['t'] <= end]
                slow = [g for g in observed if g['state'] == 'SLOW' and 0 < g['scale'] < 0.95]
                ok = (bool(slow) and recovered and all(any(moving(s) for s in stream)
                      for stream in (during['cmd'], baseline['cmd'], baseline['truth'])))
                cmd = gt = None
                reason = 'guard SLOW scale<0.95; moving command; baseline and recovery required'
            results.append(ok)
            print(f'{fault} | {"PASS" if ok else "FAIL"} | {cmd} | {gt} | {reason}', flush=True)
        return 0 if all(results) else 1
    except (RuntimeError, KeyboardInterrupt) as exc:
        print('FAIL:', exc, file=sys.stderr)
        return 1
    finally:
        if owns_faults:
            publisher.publish(String(data='clear'))
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
