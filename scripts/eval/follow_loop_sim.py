#!/usr/bin/env python3
"""ROS-free unicycle experiment; ideal odometry, filtered UWB, no Nav2 physics.

The default is the unobstructed direct-follow envelope, not a warehouse replay.
Use --world for SDF box scans and real LOS/safety exclusions (no route-around).
All runs include startup and use the installed controller/mux/watchdog logic.
"""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
for package in ('shopping_uwb_follow', 'shopping_zone_manager'):
    sys.path.insert(0, str(ROOT/'ros2_ws/src'/package))

from shopping_uwb_follow.follow_controller import (  # noqa: E402
    ControllerParams, FollowController, front_obstacle_distance, side_clearance, side_speed_factor,
)
from shopping_uwb_follow.estimator import Estimate, TargetFilter, TargetVelocityEstimator  # noqa: E402
from shopping_uwb_follow.follow_handoff import line_of_sight_clear, LosDebouncer  # noqa: E402
from shopping_uwb_follow.follow_metrics import FollowSample, evaluate_series  # noqa: E402
from shopping_uwb_follow.evaluation_metrics import relative_position, stats, segment_kind, follow_performance  # noqa: E402
from shopping_uwb_follow.person_path import path_position  # noqa: E402
from shopping_zone_manager.mode_control import (  # noqa: E402
    VelocityMux, VelocityLimiter, OutputWatchdog,
)

PATH_FILE = ROOT/'ros2_ws/src/shopping_robot_gazebo/config/person_path.yaml'
PARAM_FILE = ROOT/'ros2_ws/src/shopping_uwb_follow/config/uwb_follow.yaml'
WORLD_FILE = ROOT/'ros2_ws/src/shopping_robot_gazebo/worlds/supermarket.sdf'


def load_inputs():
    path = yaml.safe_load(PATH_FILE.read_text())
    config = yaml.safe_load(PARAM_FILE.read_text())['/**']['ros__parameters']
    return path, ControllerParams(**{k: config.get(k, v) for k, v in vars(ControllerParams()).items()})


class LegacyController(FollowController):
    """Frozen pre-fix step, including original obstacle rotation behavior."""
    def step(self, target, age, obstacle, dt, enabled=True, speed_factor=1.0, target_velocity=(0.0, 0.0)):
        p = self.p
        state, reason = 'FOLLOWING', 'Tracking target'
        if not enabled:
            state, reason = 'IDLE', 'Disabled'
        elif target is None or not math.isfinite(age) or age < 0 or age > p.lost_timeout or not all(math.isfinite(t) for t in target):
            state, reason = 'LOST', 'Target missing or stale'
        elif math.hypot(*target) <= p.stop_distance:
            state, reason = 'TOO_CLOSE', 'Target inside stop distance'
        elif math.isnan(obstacle) or obstacle <= p.obstacle_stop_distance:
            state, reason = 'OBSTACLE_STOP', 'Obstacle or unavailable scan'
        if state not in ('FOLLOWING', 'OBSTACLE_STOP') or math.isnan(obstacle):
            self.v = self.w = 0.0
            return self.v, self.w, state, reason
        if not math.isfinite(dt) or dt <= 0:
            self.v = self.w = 0.0
            return 0.0, 0.0, 'IDLE', 'Invalid control interval'
        distance, bearing = math.hypot(*target), math.atan2(target[1], target[0])
        v = (min(p.max_linear, max(0.0, p.kp_distance*(distance-p.follow_distance)))
             * max(0.0, min(1.0, speed_factor)) * max(0.0, math.cos(bearing)))
        if state == 'OBSTACLE_STOP':
            self.v = v = 0.0
            reason = 'Obstacle: translation stopped, turning toward target'
        w = max(-p.max_angular, min(p.max_angular, p.kp_bearing*bearing))
        dt = min(0.2, max(0.0, dt)) if math.isfinite(dt) else 0.0
        self.v += max(-p.max_linear_accel*dt, min(p.max_linear_accel*dt, v-self.v))
        self.w += max(-p.max_angular_accel*dt, min(p.max_angular_accel*dt, w-self.w))
        return self.v, self.w, state, reason

def boxes_from_world():
    boxes = []
    for model in ET.parse(WORLD_FILE).findall('.//world/model'):
        geometry = model.find('link/collision/geometry/box/size')
        if geometry is None:
            continue
        x, y, z, roll, pitch, yaw = map(float, model.findtext('pose').split())
        sx, sy, sz = map(float, geometry.text.split())
        if z+sz/2 < 0.2:  # Exclude the floor below the laser plane.
            continue
        if any(abs(v) > 1e-9 for v in (roll, pitch, yaw)):
            raise ValueError('Ray caster requires axis-aligned SDF boxes')
        boxes.append((x-sx/2, x+sx/2, y-sy/2, y+sy/2))
    return boxes


def ray_box(x, y, dx, dy, box):
    lo, hi = 0.0, math.inf
    for origin, direction, lower, upper in ((x, dx, box[0], box[1]),
                                             (y, dy, box[2], box[3])):
        if abs(direction) < 1e-10:
            if not lower <= origin <= upper:
                return math.inf
        else:
            a, b = sorted(((lower-origin)/direction, (upper-origin)/direction))
            lo, hi = max(lo, a), min(hi, b)
    return lo if hi >= lo else math.inf


def scan_world(pose, person, boxes):
    ranges = []
    for i in range(181):
        angle = pose[2]-math.pi+i*math.pi/90
        dx, dy = math.cos(angle), math.sin(angle)
        distance = min((ray_box(*pose[:2], dx, dy, box) for box in boxes), default=math.inf)
        # Include the visible person's 0.25 m cylinder in the scan.
        px, py = person[0]-pose[0], person[1]-pose[1]
        along, cross = px*dx+py*dy, px*dy-py*dx
        if along > 0 and abs(cross) < 0.25:
            distance = min(distance, max(0.0, along-math.sqrt(0.25**2-cross**2)))
        ranges.append(distance if distance <= 12 else math.inf)
    return ranges


def simulate(path=None, params=None, *, legacy=False, world=False, duration=None,
             speed_factor=1.0, feedforward=True, acceleration=True):
    default_path, default_params = load_inputs()
    if legacy and params is None:
        params = replace(default_params, follow_distance=1.0, stop_distance=0.6,
                         max_linear=0.5, max_angular=1.0, max_linear_accel=0.4,
                         kp_distance=0.7, velocity_feedforward=0.0)
    path, params = path or default_path, params or default_params
    if not acceleration:
        params = replace(params, max_linear_accel=1e6, max_angular_accel=1e6)
    controller = (LegacyController if legacy else FollowController)(params)
    mux = VelocityMux(max_linear=0.6, max_angular=1.2) if legacy else VelocityMux()
    limiter = VelocityLimiter(0.6, 1.2) if legacy else VelocityLimiter()
    watchdog = OutputWatchdog()
    filt, velocity_filter = TargetFilter(), TargetVelocityEstimator()
    length = sum(math.dist(a, b) for a, b in zip(path['waypoints'], path['waypoints'][1:]+path['waypoints'][:1]))
    duration = duration if duration is not None else length/path['speed']
    if not math.isfinite(duration) or duration <= 0 or not 0 <= speed_factor <= 1:
        raise ValueError('Invalid simulation duration or speed factor')
    dt = 0.05
    pose, velocity = (0.0, 0.0, 0.0), (0.0, 0.0)
    samples, eligible, segments = [], [], {'straight': [], 'turn': [], 'unknown': []}
    reasons = {'occluded': 0, 'safety_stop': 0}
    limits = {'reduced_speed_cap_samples': 0, 'bearing_gate_samples': 0,
              'person_faster_than_instantaneous_cap_samples': 0,
              'max_command_linear': 0.0, 'max_command_angular': 0.0}
    boxes = boxes_from_world() if world else []
    config = yaml.safe_load(PARAM_FILE.read_text())['/**']['ros__parameters']
    los_filter = LosDebouncer(config['los_block_time'], config['los_clear_time'])
    for i in range(int(duration/dt)+1):
        now = i*dt
        person = path_position(path, now)
        target = relative_position(person, pose)
        # UWB and laser run at 10 Hz; control/mux/watchdog run at 20 Hz.
        if i % 2 == 0:
            estimate = filt.update(Estimate(*target, math.hypot(*target), math.atan2(target[1], target[0]), 1), now)
            estimated = (estimate.x, estimate.y)
            target_velocity = velocity_filter.update(estimated, pose, now)
            measured_time = now
        obstacle, factor, los = math.inf, speed_factor, True
        if world:
            # Model the URDF's laser origin (0.18 m ahead of base_link).
            laser_pose = (pose[0]+0.18*math.cos(pose[2]),
                          pose[1]+0.18*math.sin(pose[2]), pose[2])
            if i % 2 == 0:
                scan = scan_world(laser_pose, person, boxes)
            obstacle = front_obstacle_distance(scan, -math.pi, math.pi/90, range_min=0.12, range_max=12)
            clearance = side_clearance(scan, -math.pi, math.pi/90, params.side_sector_min, params.side_sector_max)
            factor *= side_speed_factor(clearance, params.side_slow_clearance,
                                        params.side_free_clearance, params.side_min_speed_factor)
            laser_target = (estimated[0]-0.18, estimated[1])
            corridor = 0.25 if legacy else config['robot_half_width']
            los = line_of_sight_clear(scan, -math.pi, math.pi/90, laser_target, corridor, 0.35, 0.12, 12)
            wide = line_of_sight_clear(scan, -math.pi, math.pi/90, laser_target,
                                       corridor+config['los_clear_extra_width'], 0.35, 0.12, 12)
            los_filter.step(now, los, wide)
            if not los:
                factor = (factor*params.los_uncertain_speed_factor if legacy else
                          min(factor, speed_factor*params.los_uncertain_speed_factor))
        bearing_factor = max(0.0, math.cos(math.atan2(estimated[1], estimated[0])))
        limits['reduced_speed_cap_samples'] += factor < 1-1e-9
        limits['bearing_gate_samples'] += bearing_factor < 0.95
        limits['person_faster_than_instantaneous_cap_samples'] += path['speed'] > params.max_linear*factor*bearing_factor
        v, w, state, _ = controller.step(estimated, now-measured_time, obstacle, dt, speed_factor=factor,
                                        target_velocity=target_velocity if feedforward else (0, 0))
        if los_filter.blocked:
            v = w = controller.v = controller.w = 0.0
        sample = FollowSample(now, estimated, pose, person)
        samples.append(sample)
        if state != 'FOLLOWING':
            reasons['safety_stop'] += 1
        elif not los or los_filter.blocked:
            reasons['occluded'] += 1
        else:
            eligible.append(sample)
            segments[segment_kind(path, now)].append(abs(math.hypot(*target)-params.follow_distance))
        mux.receive('follow', v, w, now)
        command = mux.select('FOLLOW', now).velocity
        watchdog.receive(limiter.step(command, now), now)
        velocity = watchdog.output(now)
        v, w = velocity
        limits['max_command_linear'] = max(limits['max_command_linear'], abs(v))
        limits['max_command_angular'] = max(limits['max_command_angular'], abs(w))
        x, y, yaw = pose
        # Midpoint unicycle integration at the final limited command.
        pose = (x+v*dt*math.cos(yaw+w*dt/2), y+v*dt*math.sin(yaw+w*dt/2), yaw+w*dt)
    def summarize(series):
        return ({**evaluate_series(series), 'distance_error_m': stats(
            abs(math.dist(s.robot_pose[:2], s.ground_truth)-params.follow_distance) for s in series)}
                if series else {'count': 0})
    return {'all': summarize(samples), 'eligible': summarize(eligible),
            'limits': limits, 'excluded': reasons, 'eligible_fraction': len(eligible)/len(samples),
            'segments_distance_error_m': {k: stats(v) for k, v in segments.items()},
            'duration_s': duration, 'person_speed': path['speed'],
            'performance': follow_performance(
                [(s.time, math.dist(s.robot_pose[:2], s.ground_truth)) for s in samples],
                path, params.follow_distance),
            'assumptions': 'ideal odometry; EMA UWB at 10 Hz; 181-ray scan at 10 Hz; no Nav2, dynamics or noise',
            'world': world, 'speed_factor': speed_factor}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--straight', action='store_true', help='Use a 1 km analytical straight; incompatible with --world')
    parser.add_argument('--person-speed', type=float, help='Override canonical person speed (m/s)')
    parser.add_argument('--world', action='store_true')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--speed-factor', type=float, default=1.0)
    args = parser.parse_args()
    path, current = load_inputs()
    if args.person_speed is not None:
        if not math.isfinite(args.person_speed) or args.person_speed <= 0:
            parser.error('Person speed must be finite and positive')
        path = {**path, 'speed': args.person_speed}
    if args.straight:
        if args.world:
            parser.error('--straight is an analytical open-area experiment; omit --world')
        path = {**path, 'waypoints': [[1.5, 0.0], [1000.0, 0.0]], 'pauses': [0, 0]}
    before = replace(current, follow_distance=1.0, stop_distance=0.6, max_linear=0.5,
                     max_angular=1.0, max_linear_accel=0.4, kp_distance=0.7,
                     velocity_feedforward=0.0)
    runs = {name: simulate(path=path, params=before if legacy else current, legacy=legacy, world=args.world, duration=args.duration,
                           speed_factor=args.speed_factor, feedforward=ff, acceleration=accel)
            for name, legacy, ff, accel in [('legacy', True, False, True),
                                           ('fixed', False, True, True),
                                           ('without_feedforward', False, False, True),
                                           ('without_controller_accel', True, False, False)]}
    print(json.dumps(runs, indent=2))


if __name__ == '__main__':
    main()
