"""Direct-follow obstacle rotation safety regressions, without ROS."""
import math

import pytest

from shopping_uwb_follow.follow_controller import ControllerParams, FollowController


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
    assert state == 'FOLLOWING' and 0 < v <= params.max_linear_accel*0.05


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
