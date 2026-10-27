"""Direct-follow obstacle rotation safety regressions, without ROS."""
import math

import pytest

from shopping_uwb_follow.follow_controller import ControllerParams, FollowController, NavigationGuard
from shopping_uwb_follow.follow_handoff import HandoffSupervisor


@pytest.mark.parametrize('front', [0.593, math.inf])
@pytest.mark.parametrize('sign', [-1, 1])
def test_pass_through_stays_stopped_until_sustained_release(front, sign):
    control = FollowController()
    # The bearing flips behind the robot; a clear rotation disk is insufficient.
    positions = [(0.51, 0.01), (0.23, 0.01), (0.01, 0.01),
                 (-0.08, 0.01), (-0.39, 0.01), (-0.59, 0.01), (-0.70, 0.01)]
    for x, y in positions:
        assert control.step((x, sign*y), 0, front, 0.05,
                            rotation_clear=True)[:3] == (0, 0, 'TOO_CLOSE')
        assert control.near_latched
    for _ in range(5):
        assert control.step((-0.9, sign*0.01), 0, front, 0.05,
                            rotation_clear=True)[:3] == (0, 0, 'TOO_CLOSE')
    # A dip below the release margin restarts the sustained-clearance timer.
    assert control.step((-0.70, sign*0.01), 0, front, 0.05,
                        rotation_clear=True)[:2] == (0, 0)
    for _ in range(5):
        assert control.step((-0.9, sign*0.01), 0, front, 0.05,
                            rotation_clear=True)[:2] == (0, 0)
    v, w, state, _ = control.step((-0.9, sign*0.01), 0, front, 0.05,
                                   rotation_clear=True)
    assert not control.near_latched and v == 0 and sign*w > 0
    assert state == ('FOLLOWING' if front == math.inf else 'OBSTACLE_STOP')


@pytest.mark.parametrize('front', [0.593, math.inf])
def test_navigation_guard_near_latch_stops_existing_rotation_immediately(front):
    guard = NavigationGuard(ControllerParams())
    assert guard.step((0.4, 0.8), front, True, 0.05)[1] > 0
    for _ in range(10):
        assert guard.step((0.4, 0.8), front, True, 0.05,
                          near_latched=True) == (0, 0)
        assert guard.v == guard.w == 0


def test_near_latch_prevents_handoff_even_if_state_is_masked_by_guard():
    supervisor = HandoffSupervisor()
    for now in (0, 1, 2):
        decision = supervisor.step(now, 'OBSTACLE_STOP', (3, 0), False, 0.4,
                                   map_target=(3, 0), near_latched=True)
        assert decision.mode == 'DIRECT' and decision.action == 'none'
    supervisor.step(3, 'OBSTACLE_STOP', (3, 0), False, 0.4, map_target=(3, 0))
    assert supervisor.step(4, 'OBSTACLE_STOP', (3, 0), False, 0.4,
                           map_target=(3, 0)).action == 'send_goal'
    decision = supervisor.step(4.05, 'OBSTACLE_STOP', (-0.5, 0.01), False, 0.4,
                               'active', map_target=(-0.5, 0.01), near_latched=True)
    assert decision.mode == 'DIRECT' and decision.action == 'cancel_goal'


@pytest.mark.parametrize('bearing', [-1.35, -0.25, 0, 0.25, 1.35])
def test_clear_fresh_target_rotates_with_limits_and_never_translates(bearing):
    params = ControllerParams(kp_bearing=0.8, max_angular=0.7, max_angular_accel=1.2)
    control = FollowController(params)
    control.v = 0.5
    target = (3*math.cos(bearing), 3*math.sin(bearing))
    previous = control.w
    for _ in range(50):
        v, w, state, _ = control.step(target, params.lost_timeout, 0.593, 0.05,
                                       rotation_clear=True)
        assert state == 'OBSTACLE_STOP' and v == 0.0
        assert abs(w) <= params.max_angular
        assert abs(w-previous) <= params.max_angular_accel*0.05+1e-12
        previous = w
    assert w == pytest.approx(max(-0.7, min(0.7, 0.8*bearing)))
    # Clearing the front resumes the existing following logic and linear ramp.
    v, _, state, _ = control.step(target, 0, 1.0, 0.05, rotation_clear=True)
    assert state == 'FOLLOWING'
    if abs(bearing) >= params.rotate_to_target_min_angle:
        assert v == 0
        assert control.step((3, 0), 0, 1.0, 0.05, rotation_clear=True)[0] > 0
    else:
        assert 0 < v <= params.max_linear_accel*0.05


@pytest.mark.parametrize('kwargs,state', [
    ({'rotation_clear': False}, 'OBSTACLE_STOP'),
    ({'age': 0.501}, 'LOST'),
    ({'age': math.nan}, 'LOST'),
    ({'age': -0.1}, 'LOST'),
    ({'target': None}, 'LOST'),
    ({'target': (math.nan, 1)}, 'LOST'),
    ({'target': (0.2, 0.2)}, 'TOO_CLOSE'),
    ({'enabled': False}, 'IDLE'),
    ({'obstacle': math.nan}, 'OBSTACLE_STOP'),
    ({'obstacle': -1}, 'OBSTACLE_STOP'),
    ({'dt': 0}, 'IDLE'),
    ({'dt': math.nan}, 'IDLE'),
])
def test_faults_and_unclear_rotation_stop_immediately(kwargs, state):
    control = FollowController()
    control.step((2, 1), 0, 0.593, 0.1, rotation_clear=True)
    assert control.w > 0
    args = dict(target=(2, 1), age=0, obstacle=0.593, dt=0.1, rotation_clear=True)
    args.update(kwargs)
    assert control.step(**args)[:3] == (0.0, 0.0, state)
    assert control.v == control.w == 0.0


def test_disabled_parameter_restores_full_stop():
    control = FollowController(ControllerParams(obstacle_stop_rotate=False))
    control.v, control.w = 0.5, 1.0
    assert control.step((2, 1), 0, 0.593, 0.1, rotation_clear=True)[:3] == (
        0.0, 0.0, 'OBSTACLE_STOP')


def test_rotation_large_dt_and_direction_change_remain_acceleration_limited():
    control = FollowController()
    assert control.step((1, 2), 0, 0.593, 100, rotation_clear=True)[:2] == pytest.approx((0, 0.3))
    previous = control.w
    v, w, _, _ = control.step((1, -2), 0, 0.593, 0.05, rotation_clear=True)
    assert v == 0 and w == pytest.approx(previous-0.075)


@pytest.mark.parametrize('bearing', [-1.2, 1.2])
def test_near_release_latch_precedes_obstacle_rotation(bearing):
    control = FollowController()
    assert control.step((0.5, 0), 0, 0.593, 0.05, rotation_clear=True)[:3] == (
        0, 0, 'TOO_CLOSE')
    target = (2*math.cos(bearing), 2*math.sin(bearing))
    for _ in range(5):
        assert control.step(target, 0, 0.593, 0.05, rotation_clear=True)[:3] == (
            0, 0, 'TOO_CLOSE')
    v, w, state, _ = control.step(target, 0, 0.593, 0.05, rotation_clear=True)
    assert v == 0 and w*bearing > 0 and state == 'OBSTACLE_STOP'
    # Clearing the front still requires alignment before translation.
    assert control.step(target, 0, math.inf, 0.05)[0] == 0
    assert control.step((2, 0.3), 0, math.inf, 0.05)[0] > 0


def test_default_runtime_uses_configured_proportional_tracking_without_velocity():
    for distance in (0.8, 1.02, 1.06, 1.2, 2.0):
        control = FollowController()
        for _ in range(60):
            v, _, state, _ = control.step((distance, 0), 0, math.inf, 0.05,
                                           speed_factor=0.6)
        assert state == 'FOLLOWING'
        assert v == pytest.approx(min(1.2, max(0, 1.0*(distance-1.2)))*0.6)


def test_direct_rotation_recovers_from_fixed_shelf_in_closed_loop():
    from shopping_uwb_follow.follow_controller import front_obstacle_distance, rotation_scan_clear

    control = FollowController()
    x = y = yaw = 0.0
    control.v = 0.5
    rotated = moved = False
    for _ in range(200):
        laser_x, laser_y = x+0.18*math.cos(yaw), y+0.18*math.sin(yaw)
        ranges = []
        for i in range(720):
            angle = -math.pi+i*2*math.pi/719
            dx, dy = math.cos(yaw+angle), math.sin(yaw+angle)
            distance = (0.787-laser_x)/dx if dx > 1e-9 else math.inf
            hit = math.isfinite(distance) and distance > 0 and -1 <= laser_y+distance*dy <= 1
            ranges.append(distance if hit and distance <= 12 else math.inf)
        front = front_obstacle_distance(ranges, -math.pi, 2*math.pi/719)
        clear = rotation_scan_clear(ranges, -math.pi, 2*math.pi/719,
                                    0.12, 12, (-0.18, 0, 0), 0.45)
        # Synthetic target observations in base_link; no ROS or simulator truth input.
        target = (-x*math.cos(yaw)+(3-y)*math.sin(yaw),
                  x*math.sin(yaw)+(3-y)*math.cos(yaw))
        v, w, state, _ = control.step(target, 0, front, 0.05, rotation_clear=clear)
        if front <= control.p.obstacle_stop_distance:
            assert v == 0 and state == 'OBSTACLE_STOP'
            rotated |= w > 0
        moved |= v > 0
        x += v*math.cos(yaw)*0.05
        y += v*math.sin(yaw)*0.05
        yaw += w*0.05
        assert x+0.45 < 0.787
    assert rotated and moved and y > 1
