"""Approaching people and constrained standoff preserve open-area tracking."""
import math
from pathlib import Path

import pytest
import yaml

from shopping_uwb_follow.follow_controller import (
    ControllerParams, FollowController, approach_cap, radial_velocity,
)


def settled(distance, velocity=(0.0, 0.0), **kwargs):
    control = FollowController()
    for _ in range(100):
        command = control.step((distance, 0), 0, math.inf, .05,
                               target_velocity=velocity, **kwargs)
    return command[0]


@pytest.mark.parametrize('distance,expected', [(2.0, .185), (1.8, 0.0), (1.5, 0.0)])
@pytest.mark.parametrize('feedforward', [True, False])
def test_constrained_approach_reduces_demand_even_without_receding_feedforward(
        distance, expected, feedforward):
    # At 2 m the specified default formula gives .5 - .9*.35 = .185 m/s.
    assert settled(distance, (-.35, 0), clearance=.4,
                   feedforward_enabled=feedforward) == pytest.approx(expected)


@pytest.mark.parametrize('distance', [1.1, 1.5, 1.7, 2.0, 3.0])
def test_constrained_standoff_disables_receding_feedforward(distance):
    expected = min(.5, max(0.0, distance-1.5))
    for velocity in [(0, 0), (.35, 0), (1.2, 0)]:
        assert settled(distance, velocity, clearance=.4) == pytest.approx(expected)


@pytest.mark.parametrize('distance', [1.3, 1.5, 2.0, 3.0])
@pytest.mark.parametrize('forward', [.2, .5, 1.2])
def test_open_receding_command_matches_previous_feedforward_and_headway(distance, forward):
    p = ControllerParams()
    previous_demand = max(0, p.kp_distance*(distance-p.follow_distance
                          - p.follow_time_headway*forward)+p.velocity_feedforward*forward)
    assert settled(distance, (forward, 0), clearance=math.inf) == pytest.approx(
        min(p.max_linear, previous_demand))


@pytest.mark.parametrize('clearance,valid,los,expected', [
    (.5, True, True, 1.4), (.49, True, True, 1.5),
    (math.nan, True, True, 1.5), (math.inf, False, True, 1.5),
    (math.inf, True, False, 1.5)])
def test_effective_follow_distance_shares_standoff_and_headway(clearance, valid, los, expected):
    control = FollowController()
    assert control.effective_follow_distance(clearance, valid, los, .5) == pytest.approx(expected)


def test_constrained_standoff_stops_carried_motion_without_stopping_turn():
    control = FollowController()
    control.v = .5
    v, w, state, _ = control.step((1.4, .1), 0, math.inf, .05, clearance=.4)
    assert v == 0 and w > 0 and state == 'FOLLOWING'
    assert not control.near_latched


@pytest.mark.parametrize('velocity,age', [
    (None, 0), ((math.nan, 0), 0), ((-1, 0), .51), ((-1, 0), -1)])
def test_invalid_velocity_cannot_apply_approach_term(velocity, age):
    control = FollowController()
    control.v = .5
    v = control.step((2, 0), 0, math.inf, .05, target_velocity=velocity,
                     velocity_age=age, clearance=.4)[0]
    assert v == pytest.approx(.5)


@pytest.mark.parametrize('radial,expected', [(-.1, .3), (-.1001, .20991), (0, .3), (.35, .3)])
def test_direct_approach_threshold_is_strict(radial, expected):
    assert settled(1.8, (radial, 0), clearance=.4) == pytest.approx(expected)


def test_approach_uses_radial_projection_instead_of_forward_component():
    control = FollowController()
    # Target at (1.6, 1.2): forward velocity is positive, but radial is -.35.
    velocity = (.1, -.43/.6)
    assert radial_velocity((1.6, 1.2), velocity) == pytest.approx(-.35)
    for _ in range(100):
        v = control.step((1.6, 1.2), 0, math.inf, .05,
                         target_velocity=velocity, clearance=.4)[0]
    assert v == pytest.approx(.185*.8)


@pytest.mark.parametrize('target,velocity,expected', [
    ((3, 4), (-1, -2), -2.2), ((3, 4), (4, -3), 0),
    ((0, 0), (-1, 0), 0), (None, (1, 0), 0),
    ((1, 0), None, 0), ((1, 0), (math.nan, 0), 0)])
def test_radial_velocity(target, velocity, expected):
    assert radial_velocity(target, velocity) == pytest.approx(expected)


@pytest.mark.parametrize('distance,effective,radial,kp,ff,expected', [
    (2, 1.5, -.35, 1, .9, .185), (1.5, 1.5, -.35, 1, .9, 0),
    (1.7, 1.5, -.35, 1, .9, 0), (2, 1.5, .35, 1, .9, .5),
    (1.4, 1.5, 0, 1, .9, 0), (2, 1.5, -.2, 2, .5, .9),
    (2, 1.5, -.05, 1, .9, .455)])
def test_approach_cap(distance, effective, radial, kp, ff, expected):
    assert approach_cap(distance, effective, radial, kp, ff) == pytest.approx(expected)


def test_approach_defaults_match_yaml():
    config = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/uwb_follow.yaml').read_text())
    params = config['/**']['ros__parameters']
    defaults = ControllerParams()
    assert params['constrained_extra_distance'] == defaults.constrained_extra_distance == .3
    assert params['approach_speed_threshold'] == defaults.approach_speed_threshold == .1
