"""Deterministic cyclic trajectories and bounded velocity tracking."""
import math

from shopping_uwb_follow.person_path import path_position as trajectory_position


def tracking_velocity(person, elapsed, position, yaw, dt=0.05):
    if elapsed < person['start_delay'] or person['speed'] == 0:
        return 0.0, 0.0
    target = trajectory_position(person, elapsed+dt, person['start_delay'])
    dx, dy = ((target[i]-position[i])/dt for i in (0, 1))
    scale = min(1.0, person['speed']/max(math.hypot(dx, dy), 1e-12))
    c, s = math.cos(yaw), math.sin(yaw)
    return scale*(c*dx+s*dy), scale*(-s*dx+c*dy)
