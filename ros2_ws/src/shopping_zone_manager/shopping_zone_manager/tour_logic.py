"""ROS-free bounded waypoint sequencing and tour configuration."""
from dataclasses import dataclass, field
import math
import re

import yaml

from .zones import Zone


def map_prefix(directory, name):
    if not re.fullmatch(r'[A-Za-z0-9_-]+', name):
        raise ValueError('Use a simple map name (letters, numbers, underscore, hyphen)')
    return str(directory / name)


def load_tour(path):
    with open(path, encoding='utf-8') as stream:
        entries = yaml.safe_load(stream)['waypoints']
    if not isinstance(entries, list) or not entries:
        raise ValueError('Expected nonempty waypoints')
    goals = []
    for entry in entries:
        if any(isinstance(entry[key], bool) for key in ('x', 'y', 'yaw')):
            raise ValueError('Boolean coordinates are invalid')
        values = [float(entry[key]) for key in ('x', 'y', 'yaw')]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Waypoint coordinates must be finite')
        name = entry['name']
        if not isinstance(name, str) or not name or name in [g.name for g in goals]:
            raise ValueError('Waypoint names must be unique and nonempty')
        goals.append(Zone(name, (), *values, 'map'))
    return goals


@dataclass
class Sequence:
    goals: list
    started: float
    timeout: float = 1800.0
    max_failures: int = 2
    index: int = 0
    failures: int = 0
    reached: list = field(default_factory=list)
    skipped: list = field(default_factory=list)
    timed_out: bool = False

    def __post_init__(self):
        if (not self.goals or type(self.max_failures) is not int or self.max_failures < 1
                or not math.isfinite(self.started) or not math.isfinite(self.timeout) or self.timeout <= 0):
            raise ValueError('Nonempty goals, positive timeout and failure limit required')

    def current(self, now):
        if now - self.started >= self.timeout:
            self.timed_out = True
        return None if self.timed_out or self.index == len(self.goals) else self.goals[self.index]

    def record(self, success):
        if self.timed_out or self.index == len(self.goals):
            raise RuntimeError('Sequence has finished')
        if success:
            self.reached.append(self.goals[self.index].name)
        else:
            self.failures += 1
            if self.failures < self.max_failures:
                return
            self.skipped.append(self.goals[self.index].name)
        self.index += 1
        self.failures = 0

    def summary(self):
        return {'reached': list(self.reached), 'skipped': list(self.skipped),
                'pending': [g.name for g in self.goals[self.index:]],
                'timed_out': self.timed_out,
                'success': self.index == len(self.goals) and not self.skipped and not self.timed_out}
