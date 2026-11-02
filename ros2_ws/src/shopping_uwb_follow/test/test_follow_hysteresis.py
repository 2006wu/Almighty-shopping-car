"""LOS persistence, clearance scaling, ownership holds, and safety regressions."""
import math

import pytest

from shopping_uwb_follow.follow_controller import (
    ControllerParams, FollowController, side_clearance, side_speed_factor,
)
from shopping_uwb_follow.follow_handoff import (
    HandoffParams, HandoffSupervisor, LosDebouncer, line_of_sight_clear,
)


def test_debounce_block_clear_and_clock_reset():
    los = LosDebouncer()
    assert not los.step(0, False, False) and los.uncertain
    los.step(0.29, False, False)
    assert not los.blocked
    los.step(0.3, False, False)
    assert los.blocked and not los.uncertain
    assert not los.step(1, True, True)
    assert not los.step(1.59, True, True)
    assert los.step(1.6, True, True) and not los.blocked
    los.step(2, False, False)
    los.step(2.3, False, False)
    los.step(0, False, False)
    assert not los.blocked and los.uncertain
    los.step(math.nan, False, False)
    assert not los.blocked and los.last_time is None


def test_short_block_pulses_do_not_accumulate():
    los = LosDebouncer()
    for t in range(10):
        los.step(t, False, False)
        los.step(t+0.2, True, True)
        assert not los.blocked
    assert not los.clear  # Even a close target needs confirmed-clear LOS.


def test_wider_corridor_must_clear_continuously():
    # Return at lateral distance 0.30 m clears 0.25 m but not 0.35 m.
    args = dict(ranges=[1, math.inf], angle_min=math.asin(0.3),
                angle_increment=-math.asin(0.3), target_xy=(3, 0),
                margin=0.35, range_min=0.1, range_max=12)
    raw = line_of_sight_clear(**args, robot_half_width=0.25)
    wide = line_of_sight_clear(**args, robot_half_width=0.35)
    assert raw and not wide
    los = LosDebouncer()
    los.step(0, False, False)
    los.step(0.3, False, False)
    assert not los.step(1, raw, wide)
    assert not los.step(10, raw, wide) and los.blocked
    los.step(11, True, True)
    los.step(11.5, raw, wide)
    los.step(12, True, True)
    assert not los.step(12.59, True, True)
    assert los.step(12.61, True, True)


@pytest.mark.parametrize('clearance,expected', [(0, 0.4), (0.45, 0.4),
    (0.675, 0.7), (0.9, 1), (2, 1), (math.inf, 1), (math.nan, 0.4)])
def test_side_factor(clearance, expected):
    assert side_speed_factor(clearance) == pytest.approx(expected)


def test_side_sectors_measure_lateral_distance_and_both_sides():
    # Front and rear returns do not affect the lateral sectors.
    ranges = [0.01, 0.5, math.inf, 0.01, 1, math.inf, 0.01]
    assert side_clearance(ranges, -math.pi, math.pi/3, math.pi/6,
                          math.radians(100)) == pytest.approx(math.sin(math.pi/3))
    assert math.isnan(side_clearance([2], 0, 1, math.pi/6, math.radians(100)))


@pytest.mark.parametrize('field,value', [('side_sector_min', -1),
    ('side_sector_max', 4), ('side_free_clearance', 0.2),
    ('side_min_speed_factor', 1.1), ('los_uncertain_speed_factor', 1.1),
    ('side_slow_clearance', math.nan)])
def test_slowdown_validation(field, value):
    params = ControllerParams()
    setattr(params, field, value)
    with pytest.raises(ValueError):
        FollowController(params)


def test_uncertain_and_side_factors_cap_speed_with_acceleration_and_safety():
    normal, slow = FollowController(), FollowController()
    factor = slow.p.los_uncertain_speed_factor*side_speed_factor(0.45)
    for _ in range(50):
        v, _, state, _ = slow.step((4, 0), 0, 3, 0.05, speed_factor=factor)
        normal.step((4, 0), 0, 3, 0.05)
    assert state == 'FOLLOWING'
    assert v == pytest.approx(1.0*0.6*0.4) and v < normal.v
    # Emergency stops ignore both the speed cap and acceleration ramp.
    for kwargs in [dict(target=None), dict(age=1), dict(target=(0.1, 0)),
                   dict(enabled=False), dict(obstacle=math.nan)]:
        args = dict(target=(4, 0), age=0, obstacle=3, dt=0.05, speed_factor=factor)
        args.update(kwargs)
        slow.v = slow.w = 0.5
        assert slow.step(**args)[:2] == (0, 0)


def step(s, t, **kwargs):
    args = dict(state='FOLLOWING', target=(3, 0), los_clear=False,
                los_clear_wide=False, front_clearance=3, map_target=(3, 0))
    args.update(kwargs)
    return s.step(t, **args)


def nav():
    s = HandoffSupervisor(HandoffParams(handoff_delay=0.5))
    step(s, 0)
    step(s, 0.3)
    assert step(s, 0.8).action == 'send_goal'
    return s


@pytest.mark.parametrize('status', ['active', 'succeeded'])
def test_nav_hold_and_debounced_close_return(status):
    s = nav()
    clear = dict(los_clear=True, los_clear_wide=True, target=(1.3, 0), nav_status=status)
    assert step(s, 0.9, **clear).mode == 'NAV2'
    assert step(s, 1.51, **clear).mode == 'NAV2'  # Clear, minimum hold still active.
    assert step(s, 2.29, **clear).mode == 'NAV2'
    assert step(s, 2.31, **clear).action == 'cancel_goal'
    s = nav()
    step(s, 3, **clear)
    assert step(s, 3.59, **clear).mode == 'NAV2'
    assert step(s, 3.61, **clear).mode == 'NAV2'
    assert step(s, 4.12, **clear).action == 'cancel_goal'


def test_direct_hold_after_normal_clear_exit():
    s = nav()
    clear = dict(los_clear=True, los_clear_wide=True, target=(1.3, 0))
    step(s, 1, **clear)
    step(s, 1.61, **clear)
    assert step(s, 2.31, **clear).action == 'cancel_goal'
    step(s, 2.32)
    step(s, 2.63)  # Newly debounced blockage.
    assert step(s, 3.14).mode == 'DIRECT'  # handoff_delay passed, hold has not.
    assert step(s, 3.32).action == 'send_goal'


def test_clear_line_of_sight_does_not_release_navigation_before_catching_up():
    s = nav()
    clear = dict(los_clear=True, los_clear_wide=True, target=(4, 0))
    for t in (1, 2, 3, 5, 10):
        assert step(s, t, **clear).mode == 'NAV2'
    clear['target'] = (1.3, 0)
    assert step(s, 10.1, **clear).mode == 'NAV2'
    assert step(s, 10.61, **clear).action == 'cancel_goal'


@pytest.mark.parametrize('override', [dict(state='LOST'), dict(state='TOO_CLOSE'),
    dict(state='IDLE'), dict(enabled=False), dict(scan_valid=False),
    dict(front_clearance=math.nan), dict(nav_status='failed'),
    dict(nav_status='canceled')])
def test_safety_and_terminal_events_bypass_minimum_nav_hold(override):
    s = nav()
    assert step(s, 0.81, **override).action == 'cancel_goal'


def test_uncertain_slowdown_also_applies_below_speed_limit():
    s = FollowController()
    for _ in range(50):
        result = s.step((1.4, 0), 0, 3, 0.05, speed_factor=0.6)
    assert result[0] == pytest.approx(1.0*0.2*0.6)


def test_near_stop_latches_both_axes_through_range_noise_and_fresh_recovery():
    controller = FollowController()
    for _ in range(20):
        controller.step((3, 1), 0, math.inf, .05)
    assert controller.step((.5, .1), 0, math.inf, .05)[:3] == (0., 0., 'TOO_CLOSE')
    for distance in (.61, .74, .72, .65)*5:
        assert controller.step((0, distance), 0, math.inf, .05)[:3] == (0., 0., 'TOO_CLOSE')
    for _ in range(4):
        assert controller.step((0, 1.2), 0, math.inf, .05)[:3] == (0., 0., 'TOO_CLOSE')
    assert controller.step(None, 1, math.inf, .05)[:3] == (0., 0., 'LOST')
    for _ in range(5):
        assert controller.step((0, 1.2), 0, math.inf, .05)[:3] == (0., 0., 'TOO_CLOSE')
    assert controller.step((0, 1.2), 0, math.inf, .05)[2] == 'FOLLOWING'


def test_invalid_dt_cannot_release_near_stop_or_count_missing_time():
    controller = FollowController()
    controller.step((.4, 0), 0, math.inf, .05)
    for dt in (float('nan'), float('inf'), -1, 0):
        assert controller.step((2, 1), 0, math.inf, dt)[:3] == (0., 0., 'TOO_CLOSE')
    assert controller.step((2, 1), 0, math.inf, 100)[:3] == (0., 0., 'TOO_CLOSE')


def test_target_corner_alignment_has_hysteresis_and_preserves_acceleration_limits():
    controller = FollowController()
    for _ in range(20):
        controller.step((3, 0), 0, math.inf, .05)
    old = controller.v
    for _ in range(30):
        v, w, state, reason = controller.step((1, 2), 0, math.inf, .05)
        assert state == 'FOLLOWING' and 0 <= old-v <= .8*.05+1e-10
        assert w > 0
        old = v
    assert v == 0
    for _ in range(10):
        assert controller.step((2, 1), 0, math.inf, .05)[0] == 0
    v, w, state, _ = controller.step((3, .1), 0, math.inf, .05)
    assert state == 'FOLLOWING' and 0 < v <= .8*.05+1e-10
    assert controller.step((.1, .2), 0, math.inf, .05)[:3] == (0., 0., 'TOO_CLOSE')
