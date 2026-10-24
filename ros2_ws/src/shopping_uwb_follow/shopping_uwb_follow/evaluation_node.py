"""Read-only live collectors for the simulation evaluation commands."""
import argparse
import json
import math
from pathlib import Path
import time

import yaml

from .evaluation_metrics import (check_row, coverage, follow_report, frame_valid,
                                 sync_report)
from .person_path import path_position


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('Must be positive and finite')
    return number


def parser(mode):
    result = argparse.ArgumentParser(description='Read-only simulation '+mode+' evaluation')
    result.add_argument('--duration', type=positive, default=60.0 if mode == 'sync' else 180.0,
                        help='Recording duration in simulation seconds, after 2 s warmup')
    result.add_argument('--wall-timeout', type=positive,
                        help='Wall-clock deadline including clock startup (default: 4*duration+30)')
    result.add_argument('--output', type=Path, help='Write JSON report, including failed checks')
    result.add_argument('--person-path', type=Path, help='Override installed shared path YAML')
    result.add_argument('--follow-params', type=Path, help='Override installed controller params YAML')
    if mode == 'sync':
        result.add_argument('--mean-limit', type=positive, default=0.05)
        result.add_argument('--lag-limit', type=positive, default=0.1)
    else:
        result.add_argument('--latency-limit', type=positive, default=0.3)
        result.add_argument('--tracking-limit', type=positive, default=1.5,
                            help='RMSE of true distance minus configured follow_distance (m)')
        result.add_argument('--reacquire-limit', type=positive, default=2.0)
        result.add_argument('--stop-deadline', type=positive, default=0.5)
    return result


def result_code(rows):
    return 1 if any(r['status'] == 'FAIL' for r in rows) else (
        2 if any(r['status'] != 'PASS' for r in rows) else 0)


def run(mode, args=None):
    # Parse help before importing ROS so CLI syntax can be inspected offline.
    options, ros_args = parser(mode).parse_known_args(args)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    path_file = options.person_path or Path(get_package_share_directory('shopping_robot_gazebo'))/'config/person_path.yaml'
    params_file = options.follow_params or Path(get_package_share_directory('shopping_uwb_follow'))/'config/uwb_follow.yaml'
    with path_file.open(encoding='utf-8') as stream:
        path = yaml.safe_load(stream)
    path_position(path, 0.0)
    with params_file.open(encoding='utf-8') as stream:
        params = yaml.safe_load(stream)['/**']['ros__parameters']
    if any(not math.isfinite(params[k]) or params[k] <= 0
           for k in ('stop_distance', 'follow_distance', 'lost_timeout')):
        raise ValueError('Controller distance and timeout parameters must be positive and finite')

    class Collector(Node):
        def __init__(self):
            super().__init__('person_sync_check' if mode == 'sync' else 'follow_eval',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
            if not self.get_parameter('use_sim_time').value:
                raise ValueError('Evaluation requires use_sim_time=true')
            self.data = {k: [] for k in ('person', 'model', 'robot', 'target', 'states')}
            self.frames = {}
            self.invalid = 0
            self.duplicates = {k: 0 for k in self.data}
            self.start = self.end = None
            self.create_subscription(PointStamped, '/uwb/person_ground_truth', self.on_person, 100)
            if mode == 'sync':
                self.create_subscription(TFMessage, '/simulation/person_pose', self.on_model, 100)
            else:
                self.create_subscription(Odometry, '/sim/ground_truth_odom', self.on_robot, qos_profile_sensor_data)
                self.create_subscription(PointStamped, '/uwb/target', self.on_target, 100)
                self.create_subscription(String, '/follow/state', self.on_state, 100)

        def now(self):
            return self.get_clock().now().nanoseconds/1e9

        def append(self, kind, t, value, parent=None, child=None):
            if self.start is None or not self.start <= self.now() <= self.end:
                return
            if parent is not None:
                key = f'{kind}:{parent}->{child}'
                self.frames[key] = self.frames.get(key, 0)+1
                if not frame_valid(kind, parent, child):
                    self.invalid += 1
                    return
            if (not math.isfinite(t) or t <= 0 or t > self.now()+0.25
                    or (kind != 'states' and not all(math.isfinite(x) for x in value))):
                self.invalid += 1
                return
            if not self.start <= t <= self.end:
                return
            series = self.data[kind]
            if series and t <= series[-1][0]:
                self.duplicates[kind] += 1
                return
            if len(series) >= 200000:
                self.invalid += 1
                return
            series.append((t, value))

        def on_person(self, msg):
            self.append('person', stamp(msg), (msg.point.x, msg.point.y), msg.header.frame_id)

        def on_target(self, msg):
            self.append('target', stamp(msg), (msg.point.x, msg.point.y), msg.header.frame_id)

        def on_model(self, msg):
            # Dedicated publisher contains only the top-level person transform.
            for transform in msg.transforms:
                p = transform.transform.translation
                self.append('model', stamp(transform), (p.x, p.y),
                            transform.header.frame_id, transform.child_frame_id)

        def on_robot(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            v = msg.twist.twist
            self.append('robot', stamp(msg), (p.x, p.y, yaw, math.hypot(v.linear.x, v.linear.y),
                                            abs(v.angular.z)), msg.header.frame_id, msg.child_frame_id)

        def on_state(self, msg):
            self.append('states', self.now(), msg.data.split(':', 1)[0].strip())

    rclpy.init(args=ros_args)
    node = Collector()
    wall_start, last_clock, first_clock = time.monotonic(), None, None
    completed, reason = False, 'wall timeout / missing or paused simulation clock'
    try:
        while rclpy.ok() and time.monotonic()-wall_start < (options.wall_timeout or 4*options.duration+30):
            rclpy.spin_once(node, timeout_sec=0.1)
            now = node.now()
            if last_clock is not None and now < last_clock:
                reason = 'simulation clock moved backwards; rerun after reset'
                break
            last_clock = now
            if now <= 0:
                continue
            if first_clock is None:
                first_clock = now
                node.start = now+2.0
                node.end = node.start+options.duration
                node.get_logger().info(f'Recording sim interval [{node.start:.3f}, {node.end:.3f}]')
            if now >= node.end:
                completed, reason = True, 'completed'
                break
    except KeyboardInterrupt:
        reason = 'interrupted; incomplete observation'
    finally:
        node.destroy_node()
        rclpy.shutdown()

    rows = [check_row('recording', completed, reason),
            check_row('frames_and_values', node.invalid == 0,
                      {'invalid': node.invalid, 'observed_frames': node.frames,
                       'duplicate_or_reordered': node.duplicates})]
    for kind in (('person', 'model') if mode == 'sync' else ('person', 'robot', 'states')):
        rows.append(check_row(kind+'_coverage', node.start is not None and
                              coverage(node.data[kind], node.start, node.end),
                              {'count': len(node.data[kind]), 'max_gap_s': 0.25}))
    if mode == 'sync':
        rows += sync_report(node.data['person'], node.data['model'], path,
                            options.mean_limit, options.lag_limit)
    else:
        rows.append(check_row('target_samples', len(node.data['target']) >= 10
                              and node.data['target'][0][0]-node.start <= 0.25,
                              {'count': len(node.data['target'])}))
        rows += follow_report(node.data['person'], node.data['robot'], node.data['target'],
                              node.data['states'], path, params, node.end or 0,
                              options.latency_limit, options.tracking_limit,
                              options.reacquire_limit, options.stop_deadline)
    report = {'schema_version': 1, 'mode': mode, 'start_sim_s': node.start,
              'end_sim_s': node.end, 'path_file': str(path_file), 'path': path,
              'params_file': str(params_file), 'controller_params': params,
              'thresholds': {k: str(v) if isinstance(v, Path) else v for k, v in vars(options).items()},
              'rows': rows, 'status': 'PASS' if result_code(rows) == 0 else
              'FAIL' if result_code(rows) == 1 else 'NOT_COVERED'}
    print('CHECK                         STATUS        DETAILS')
    for row in rows:
        print(f"{row['check']:<29} {row['status']:<13} {json.dumps(row['details'], ensure_ascii=False, allow_nan=False)}")
    print('OVERALL: '+report['status'])
    if options.output:
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
    return result_code(rows)


def stamp(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9


def person_sync_main(args=None):
    raise SystemExit(run('sync', args))


def follow_eval_main(args=None):
    raise SystemExit(run('follow', args))
