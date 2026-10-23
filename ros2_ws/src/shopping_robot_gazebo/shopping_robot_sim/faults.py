"""ROS-free simulation fault commands, scheduling and message transforms."""
from copy import deepcopy
from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class Fault:
    name: str
    value: float = 0.0


def parse_command(command):
    parts = command.strip().split(':')
    name = parts[0]
    if name not in {'clear', 'uwb_stale', 'uwb_dropout', 'uwb_noise',
                    'lidar_drop', 'lidar_freeze', 'odom_freeze'}:
        raise ValueError('Unknown fault')
    if name in {'uwb_dropout', 'uwb_noise'}:
        if len(parts) != 2:
            raise ValueError('Fault requires a positive numeric value')
        value = float(parts[1])
        if not math.isfinite(value) or value <= 0:
            raise ValueError('Fault value must be positive and finite')
        return Fault(name, value)
    if len(parts) != 1:
        raise ValueError('Unexpected fault argument')
    return Fault(name)


class FaultEngine:
    def __init__(self, schedule=(), seed=42):
        self.active = {}
        self.cache = {}
        self.random = random.Random(seed)
        self.events = []
        self.last_time = None
        for row in schedule:
            offset, duration = float(row['time']), float(row['duration'])
            fault = parse_command(row['fault'])
            if not all(math.isfinite(v) and v >= 0 for v in (offset, duration)):
                raise ValueError('Schedule times must be finite and nonnegative')
            self.events.append((offset, fault, duration))
        self.events.sort(key=lambda e: e[0])
        self.pending = list(self.events)

    def command(self, command, now, duration=0.0):
        fault = parse_command(command) if isinstance(command, str) else command
        if fault.name == 'clear':
            self.active.clear()
            self.cache.clear()
            return
        if fault.name == 'uwb_dropout' and not duration:
            duration = fault.value
        self.active[fault.name] = (fault, now+duration if duration else math.inf)
        self.cache.pop(fault.name, None)

    def advance(self, now):
        if self.last_time is not None and now < self.last_time:
            self.command('clear', now)
            self.pending = list(self.events)
        self.last_time = now
        while self.pending and self.pending[0][0] <= now:
            offset, fault, duration = self.pending.pop(0)
            self.command(fault, offset, duration)
        for name, (_, end) in list(self.active.items()):
            if now >= end:
                del self.active[name]
                self.cache.pop(name, None)

    def mutate(self, stream, message):
        """Copy messages; frozen streams retain their original header stamp."""
        if stream == 'ranges' and 'uwb_dropout' in self.active:
            return None
        if stream == 'scan' and 'lidar_drop' in self.active:
            return None
        freeze = {'target': 'uwb_stale', 'scan': 'lidar_freeze',
                  'odom': 'odom_freeze'}.get(stream)
        if freeze in self.active:
            if freeze not in self.cache:
                self.cache[freeze] = deepcopy(message)
            return deepcopy(self.cache[freeze])
        result = deepcopy(message)
        if stream == 'ranges' and 'uwb_noise' in self.active:
            sigma = self.active['uwb_noise'][0].value
            result.data = [max(0.0, v+self.random.gauss(0, sigma))
                           if math.isfinite(v) else v for v in message.data]
        return result
