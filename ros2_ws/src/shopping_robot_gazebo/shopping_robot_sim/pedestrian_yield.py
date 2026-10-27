"""Simulation actor yielding; simulator poses must never feed robot control."""
import math

from .trajectory import tracking_velocity


class YieldingTrajectory:
    """Pause path time while blocked; resume without chasing a moving schedule."""

    def __init__(self):
        self.last = None
        self.elapsed = None
        self.blocked = False

    def velocity(self, person, now, position, yaw, robot, enabled=True):
        if not all(math.isfinite(v) for v in (now, *position, yaw)):
            self.blocked = True
            return 0.0, 0.0
        dt = 0.0 if self.last is None else max(0.0, min(now-self.last, 0.5))
        if self.last is None or now < self.last:
            self.elapsed = now
        self.last = now
        candidate_time = self.elapsed + dt
        vx, vy = tracking_velocity(person, candidate_time, position, yaw)
        if not enabled:
            self.elapsed = now
            self.blocked = False
            return tracking_velocity(person, now, position, yaw)
        # Missing/stale truth stops the actor, not the robot. A circumscribed
        # footprint plus 0.20 m margin covers pose cadence and actor stopping.
        valid = (robot is not None and len(robot) == 3
                 and all(math.isfinite(v) for v in (*position, yaw, now, *robot))
                 and 0 <= now-robot[0] <= 0.5)
        if not valid:
            self.blocked = True
            return 0.0, 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = 0.5*(c*vx-s*vy), 0.5*(s*vx+c*vy)
        rx, ry = robot[1]-position[0], robot[2]-position[1]
        length2 = dx*dx+dy*dy
        fraction = max(0.0, min(1.0, (rx*dx+ry*dy)/length2)) if length2 else 0.0
        distance = math.hypot(rx-fraction*dx, ry-fraction*dy)
        threshold = math.hypot(0.30, 0.25)+person['radius']+0.20
        if self.blocked:
            threshold += 0.05
        self.blocked = distance <= threshold
        if self.blocked:
            return 0.0, 0.0
        self.elapsed = candidate_time
        return vx, vy
