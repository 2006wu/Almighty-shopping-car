"""ROS-free safety model; all deadlines use monotonic seconds."""
import math


def forward_counter(previous, current):
    return previous is None or 0 < ((current-previous) & 0xffffffff) < 0x80000000


class Heartbeat:
    def __init__(self, timeout=0.5, recovery_count=3):
        self.timeout, self.required = timeout, recovery_count
        self.last = None
        self.counter = None
        self.count = 0

    def fresh(self, now):
        if self.last is None or now-self.last >= self.timeout or now < self.last:
            self.count = 0
        return self.count >= self.required

    def receive(self, counter, now):
        # After a timeout a rebooted peer may start from any counter.
        self.fresh(now)
        if self.count == 0:
            self.counter = None
        if forward_counter(self.counter, counter):
            self.count += 1
            self.counter, self.last = counter, now
            return True
        return False


class Firmware:
    def __init__(self, radius, track, timing, model):
        self.radius, self.track = radius, track
        self.model = model
        self.timeout = timing['command_timeout']
        self.link = Heartbeat(timing['heartbeat_timeout'], timing['recovery_count'])
        self.command = (0.0, 0.0)
        self.command_time = None
        self.invalid = False
        self.button = self.estop = False
        self.drop_until = self.drop_commands_until = -math.inf
        self.positions = [0.0]*4
        self.velocities = [0.0]*4
        self.status = 'LINK_LOST'
        self.last_step = None

    def connected(self, now):
        return now >= self.drop_until

    def heartbeat(self, value, now):
        if self.connected(now):
            was_ready = self.link.fresh(now)
            self.link.receive(value, now)
            if not was_ready:
                self.command_time = None

    def set_button(self, pressed):
        self.button = bool(pressed)
        self.estop |= self.button
        if self.estop:
            self.velocities = [0.0]*4
            self.command_time = None

    def reset(self, value, now):
        if value and not self.button and self.link.fresh(now):
            self.estop = False
            self.command_time = None

    def receive_command(self, values, now):
        if not self.connected(now) or now < self.drop_commands_until:
            return
        v, _, _, _, _, w = values
        self.invalid = (not all(math.isfinite(x) for x in values)
                        or any(values[i] != 0 for i in (1, 2, 3, 4)))
        if self.invalid:
            self.command_time = None
            self.velocities = [0.0]*4
            return
        # Negative linear commands are disabled by default; finite extremes saturate.
        self.command = (min(self.model['max_linear'], max(0.0, v)),
                        min(self.model['max_angular'], max(-self.model['max_angular'], w)))
        self.command_time = now if self.link.fresh(now) and not self.estop else None

    def drop(self, duration, now, commands_only=False):
        if math.isfinite(duration) and 0 <= duration <= 60:
            if commands_only:
                self.drop_commands_until = now+duration
                return
            else:
                self.drop_until = now+duration
                self.link.last = None
                self.link.count = 0
            self.command_time = None
            self.velocities = [0.0]*4

    def step(self, now):
        dt = 0.0 if self.last_step is None else max(0.0, min(now-self.last_step, 0.1))
        self.last_step = now
        linked = self.link.fresh(now) and self.connected(now)
        if not linked:
            self.command_time = None
        fresh_cmd = self.command_time is not None and 0 <= now-self.command_time < self.timeout
        self.status = ('ESTOP' if self.estop else 'LINK_LOST' if not linked else
                       'INVALID_COMMAND' if self.invalid else 'CMD_TIMEOUT' if not fresh_cmd else 'READY')
        if self.status != 'READY':
            self.velocities = [0.0]*4  # Safety stops bypass normal acceleration limits.
            return
        v, w = self.command
        targets = [(v-w*self.track/2)/self.radius]*2 + [(v+w*self.track/2)/self.radius]*2
        alpha = 1-math.exp(-dt/self.model['response_time'])
        limit = self.model['wheel_acceleration']*dt
        for i, target in enumerate(targets):
            delta = max(-limit, min(limit, (target-self.velocities[i])*alpha))
            self.velocities[i] += delta
            self.positions[i] += self.velocities[i]*dt

    @property
    def yaw_rate(self):
        return (sum(self.velocities[2:])-sum(self.velocities[:2]))*self.radius/(2*self.track)

    @property
    def ticks(self):
        return [round(p*self.model['ticks_per_revolution']/(2*math.pi)) for p in self.positions]


class LinkMonitor:
    def __init__(self, now, timing):
        self.started = now
        self.timeout = timing['heartbeat_timeout']
        self.link = Heartbeat(self.timeout, timing['recovery_count'])
        self.estop = False
        self.status = None
        self.status_time = None

    def heartbeat(self, value, now):
        self.link.receive(value, now)

    def receive_status(self, value, now):
        self.status, self.status_time = value, now

    def should_stop(self, now):
        if self.estop or self.status in ('ESTOP', 'INVALID_COMMAND'):
            return True
        return now-self.started >= self.timeout and (
            self.status == 'LINK_LOST' or not self.link.fresh(now) or self.status_time is None
            or not 0 <= now-self.status_time < self.timeout)
