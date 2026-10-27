"""Time headway and proactive braking limits preserve hard safety stops."""
import math

import pytest

from shopping_uwb_follow.follow_controller import ControllerParams, FollowController


def settled(params, distance, velocity, obstacle=math.inf):
    control = FollowController(params)
    for _ in range(100):
        result = control.step((distance, 0), 0, obstacle, .05, target_velocity=velocity)
    return result[0]


@pytest.mark.parametrize('forward', [0, .5, 1.2, -.5])
def test_headway_uses_only_positive_fresh_forward_velocity(forward):
    p = ControllerParams()
    distance = 2
    expected = max(0, p.velocity_feedforward*forward+distance-p.follow_distance
                   -p.follow_time_headway*max(0, forward))
    assert settled(p, distance, (forward, 0)) == pytest.approx(min(p.max_linear, expected))
    assert settled(p, distance, (0, forward)) == pytest.approx(.8)


@pytest.mark.parametrize('front', [.71, .8, 1., 1.3, 3., math.inf])
def test_command_braking_distance_never_exceeds_measured_clearance(front):
    p = ControllerParams()
    control = FollowController(p)
    control.v = p.max_linear
    v = control.step((3, 0), 0, front, .05, target_velocity=(1.2, 0))[0]
    assert p.braking_reaction*v+v*v/(2*p.braking_deceleration) <= min(3, front)-p.stop_distance+1e-12
    assert 0 <= v <= p.max_linear


def test_headway_does_not_replace_base_distance_hard_stop():
    p = ControllerParams()
    assert settled(p, 1.3, (1, 0)) > 0
    control = FollowController(p)
    control.v = 1.2
    assert control.step((1.19, 0), 0, math.inf, .05, target_velocity=(1.2, 0))[0] == 0
    for target, obstacle, state in [((.6, 0), math.inf, 'TOO_CLOSE'),
                                    ((3, 0), .5, 'OBSTACLE_STOP'),
                                    ((3, 0), math.nan, 'OBSTACLE_STOP')]:
        control = FollowController(p)
        control.v = 1.2
        assert control.step(target, 0, obstacle, .05)[:3] == (0, 0, state)


@pytest.mark.parametrize('field,value', [('follow_time_headway', -1),
    ('follow_time_headway', math.nan), ('braking_reaction', 0), ('braking_deceleration', 0)])
def test_braking_parameters_reject_invalid_values(field, value):
    with pytest.raises(ValueError):
        FollowController(ControllerParams(**{field: value}))


def test_follow_braking_defaults_match_guard_node_and_yaml():
    import ast
    from pathlib import Path
    import yaml
    from shopping_zone_manager.obstacle_guard import ObstacleGuard
    src = Path(__file__).resolve().parents[2]
    config = yaml.safe_load((src/'shopping_uwb_follow/config/uwb_follow.yaml').read_text())['/**']['ros__parameters']
    tree = ast.parse((src/'shopping_zone_manager/shopping_zone_manager/obstacle_guard_node.py').read_text())
    defaults = next(node.value for node in ast.walk(tree) if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == 'defaults' for target in node.targets))
    guard_node = {kw.arg: kw.value.value for kw in defaults.keywords if isinstance(kw.value, ast.Constant)}
    p, guard = ControllerParams(), ObstacleGuard()
    assert config['follow_time_headway'] == p.follow_time_headway == .4
    assert config['braking_reaction'] == p.braking_reaction == guard.reaction == guard_node['reaction']
    assert config['braking_deceleration'] == p.braking_deceleration == guard.deceleration == guard_node['deceleration']


def test_slow_guard_retains_headway_without_feedforward():
    control = FollowController()
    for _ in range(100):
        v, _, _, _ = control.step((2, 0), 0, math.inf, .05,
                                  target_velocity=(1, 0), feedforward_enabled=False)
    assert v == pytest.approx(2-1.2-.4)
