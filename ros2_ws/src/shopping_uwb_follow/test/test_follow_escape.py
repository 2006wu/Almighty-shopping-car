"""Shelf deadlock regression with synthetic planner commands, without ROS."""
import math

import pytest

from shopping_uwb_follow.follow_controller import (
    ControllerParams, FollowController, NavigationGuard, rotation_scan_clear,
)
from shopping_uwb_follow.follow_handoff import HandoffSupervisor, standoff_target


def scan_clear(ranges, **kwargs):
    args = dict(angle_min=-math.pi, angle_increment=2*math.pi/719,
                range_min=0.12, range_max=12, base_in_scan=(-0.18, 0, 0), radius=0.45)
    args.update(kwargs)
    return rotation_scan_clear(ranges, **args)


def test_rotation_disk_accounts_for_offset_and_rear_swept_corners():
    ranges = [math.inf]*720
    ranges[360] = 0.607  # Shelf ahead: translation blocked, disk clear.
    assert scan_clear(ranges)
    ranges[0] = 0.60  # Behind the forward-mounted laser, inside the 0.63 m boundary.
    assert not scan_clear(ranges)
    ranges[0] = 0.64
    assert scan_clear(ranges)
    ranges[180] = 0.3  # Side return inside swept disk also blocks rotation.
    assert not scan_clear(ranges)


@pytest.mark.parametrize('overrides', [
    dict(angle_increment=math.pi/719), dict(angle_increment=0),
    dict(angle_min=math.nan), dict(base_in_scan=(0.5, 0, 0)),
    dict(radius=math.nan), dict(range_max=0.2),
])
def test_rotation_unknown_or_incomplete_coverage_fails_closed(overrides):
    assert not scan_clear([math.inf]*720, **overrides)


@pytest.mark.parametrize('bad', [0, 0.1, math.nan, -math.inf, -1, 13])
def test_invalid_or_subminimum_return_anywhere_prevents_rotation(bad):
    ranges = [math.inf]*720
    ranges[20] = bad
    assert not scan_clear(ranges)


def test_rotation_disk_is_invariant_to_scan_yaw_and_scan_order():
    # A disk is yaw invariant, but its center offset is expressed in scan axes.
    assert scan_clear([0.7]*720, base_in_scan=(0, -0.18, 1.57))
    assert scan_clear([0.7]*720, angle_min=math.pi, angle_increment=-2*math.pi/719)


def test_fixed_front_shelf_hands_off_and_rotates_then_moves_when_heading_clears():
    control = FollowController()
    guard = NavigationGuard(control.p)
    supervisor = HandoffSupervisor()
    ranges = [math.inf]*720
    ranges[360] = 0.607
    rotated = False
    actions = []
    # Reproduce the pinned reading and a walking-away target over 120 seconds.
    # Nav2 is represented by a synthetic costmap-aware turn command; this is
    # a controller/ownership test, not an implementation of a global planner.
    for i in range(2401):
        now = i*0.05
        target = (1.1+0.35*now, 0.5)
        v, w, state, _ = control.step(target, 0, 0.607, 0.05)
        assert (v, w) == (0, 0)
        decision = supervisor.step(now, state, target, False, 0.607,
                                   nav_status='active', map_target=standoff_target(target, 1))
        actions.append(decision.action)
        if decision.mode == 'NAV2':
            v, w = guard.step((0.4, 0.8), 0.607, scan_clear(ranges), 0.05)
            assert v == 0
            rotated |= w > 0
            assert 0 <= w <= control.p.max_angular
    assert rotated and supervisor.mode == 'NAV2'
    assert actions.count('send_goal') == 1 and 'cancel_goal' not in actions
    # After the costmap-directed rotation puts the shelf outside the front
    # sector, forward travel is permitted again, with the same speed bounds.
    assert guard.step((0.4, 0.2), 1.0, True, 0.05)[0] > 0


@pytest.mark.parametrize('obstacle,clear,command,safe', [
    (math.nan, True, (0.4, 0.8), True),
    (0.607, False, (0.4, 0.8), True),
    (0.607, True, (0.4, 0.8), False),
    (1, True, None, True),
    (1, True, (math.nan, 0.8), True),
])
def test_navigation_guard_safety_stops_bypass_acceleration(obstacle, clear, command, safe):
    guard = NavigationGuard(ControllerParams())
    guard.v, guard.w = 0.5, 1.0
    assert guard.step(command, obstacle, clear, 0.05, safe=safe) == (0, 0)


def test_navigation_guard_limits_speed_acceleration_and_no_reverse():
    guard = NavigationGuard(ControllerParams())
    previous = (0, 0)
    for _ in range(100):
        v, w = guard.step((100, -100), 2, True, 0.05)
        assert 0 <= v <= 0.5 and -1 <= w <= 1
        assert abs(v-previous[0]) <= 0.4*0.05+1e-12
        assert abs(w-previous[1]) <= 1.5*0.05+1e-12
        previous = v, w
    assert guard.step((-1, 0.8), 0.607, True, 0.05)[0] == 0
    assert guard.step((0.5, 1), 2, True, 0) == (0, 0)


def test_success_returns_direct_without_refreshing_a_completed_goal():
    supervisor = HandoffSupervisor()
    args = dict(state='OBSTACLE_STOP', target=(3, 0), los_clear=False,
                front_clearance=0.607, map_target=(3, 0))
    supervisor.step(0, **args)
    assert supervisor.step(1, **args).action == 'send_goal'
    result = supervisor.step(2, nav_status='succeeded', **args)
    assert result.mode == 'DIRECT' and result.action == 'cancel_goal'


def test_synthetic_costmap_turn_escapes_a_fixed_shelf_in_closed_loop():
    guard = NavigationGuard(ControllerParams())
    x = y = yaw = 0.0
    initial_front = None
    moved = turned = False
    from shopping_uwb_follow.follow_controller import front_obstacle_distance

    for _ in range(300):
        # Fixed vertical shelf at x=0.787, spanning y=-1..1. Laser is 0.18 m forward.
        laser_x, laser_y = x+0.18*math.cos(yaw), y+0.18*math.sin(yaw)
        ranges = []
        for i in range(720):
            angle = -math.pi+i*2*math.pi/719
            dx, dy = math.cos(yaw+angle), math.sin(yaw+angle)
            distance = (0.787-laser_x)/dx if dx > 1e-9 else math.inf
            hit = math.isfinite(distance) and distance > 0 and -1 <= laser_y+distance*dy <= 1
            ranges.append(distance if hit and distance <= 12 else math.inf)
        front = front_obstacle_distance(ranges, -math.pi, 2*math.pi/719)
        if initial_front is None:
            initial_front = front
        # A synthetic planner first turns along the aisle, then follows it.
        # Production Nav2 remains responsible for selecting this clear heading.
        error = math.pi/2-yaw
        command = (0, min(0.8, 1.5*error)) if abs(error) > 0.05 else (0.35, 0)
        v, w = guard.step(command, front, scan_clear(ranges), 0.05)
        turned |= w > 0
        moved |= v > 0
        x += v*math.cos(yaw)*0.05
        y += v*math.sin(yaw)*0.05
        yaw += w*0.05
        assert x+0.45 < 0.787  # Swept footprint stays clear of the shelf.
    assert initial_front == pytest.approx(0.607, abs=1e-5)
    assert turned and moved and y > 1
