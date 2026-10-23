import math
import pytest
from shopping_uwb_follow.uwb_model import Anchor, DEFAULT_ANCHORS, RangeSimulator
from shopping_uwb_follow.estimator import Estimate, TargetFilter, trilaterate
from shopping_uwb_follow.follow_controller import ControllerParams, FollowController, front_obstacle_distance


@pytest.mark.parametrize('kwargs', [
    {'anchors': []}, {'anchors': [Anchor(float('nan'), 0)]*2},
    {'noise_std': -1}, {'noise_std': float('nan')}, {'bias': float('inf')},
    {'nlos_bias': -1}, {'nlos_bias': float('nan')},
    {'dropout': -0.1}, {'dropout': float('nan')},
    {'nlos_probability': -1}, {'nlos_probability': 1.1}])
def test_range_configuration_validation(kwargs):
    with pytest.raises(ValueError):
        RangeSimulator(**kwargs)


@pytest.mark.parametrize('target', [(float('nan'), 0), (0, float('inf'))])
def test_range_target_validation(target):
    with pytest.raises(ValueError):
        RangeSimulator().sample(*target)


def test_noise_statistics_and_partial_dropout():
    clean = RangeSimulator(noise_std=0).sample(2, 0.4)
    sim = RangeSimulator(noise_std=0.02, seed=32)
    errors = [sim.sample(2, 0.4)[0]-clean[0] for _ in range(2000)]
    mean = sum(errors)/len(errors)
    std = math.sqrt(sum((e-mean)**2 for e in errors)/len(errors))
    assert abs(mean) < 0.002
    assert 0.018 < std < 0.022
    sim = RangeSimulator(dropout=0.3, seed=32)
    missing = sum(math.isnan(r) for _ in range(500) for r in sim.sample(2, 0.4))
    assert 350 < missing < 550
    assert RangeSimulator(noise_std=0, bias=-100).sample(0, 0) == [0, 0, 0]


@pytest.mark.parametrize('refine', [True, False])
@pytest.mark.parametrize('target', [(0, 0), (-2, -1), (1, 3)])
def test_estimate_outputs_and_linear_solution(refine, target):
    observed = RangeSimulator(noise_std=0).sample(*target)
    result = trilaterate(DEFAULT_ANCHORS, observed, refine=refine)
    assert (result.x, result.y) == pytest.approx(target, abs=1e-7)
    assert result.distance == pytest.approx(math.hypot(*target), abs=1e-7)
    if target != (0, 0):
        assert result.bearing == pytest.approx(math.atan2(target[1], target[0]))
    assert result.quality == pytest.approx(1)


def test_residual_quality_and_two_anchor_tangent():
    observed = RangeSimulator(noise_std=0).sample(2, 0.3)
    observed[0] += 0.05
    noisy = trilaterate(DEFAULT_ANCHORS, observed)
    assert 0 < noisy.quality < 1
    result = trilaterate([Anchor(0, 0), Anchor(2, 0)], [1, 1])
    assert (result.x, result.y) == (1, 0)
    assert result.quality == 0.5


@pytest.mark.parametrize('observed', [[-1, 1, float('inf')], [0, float('nan'), 2]])
def test_invalid_ranges_are_not_positions(observed):
    with pytest.raises(ValueError):
        trilaterate(DEFAULT_ANCHORS, observed)


@pytest.mark.parametrize('kwargs', [{'timeout': 0}, {'timeout': float('nan')}, {'alpha': 0}, {'alpha': 1.1}])
def test_filter_configuration(kwargs):
    with pytest.raises(ValueError):
        TargetFilter(**kwargs)


def test_filter_timeout_boundary_reset_and_invalid_input():
    filt = TargetFilter(timeout=0.5, alpha=0.5)
    first = Estimate(2, 0, 2, 0, 1)
    next_value = Estimate(0, 2, 2, math.pi/2, 0.5)
    filt.update(first, 1)
    smooth = filt.update(next_value, 1.2)
    assert (smooth.x, smooth.y, smooth.distance, smooth.bearing, smooth.quality) == pytest.approx((1, 1, math.sqrt(2), math.pi/4, 0.5))
    assert filt.get(1.7) is smooth
    assert filt.get(1.70001) is None
    assert filt.update(first, 2) == first
    with pytest.raises(ValueError):
        filt.update(first, float('nan'))
    with pytest.raises(ValueError):
        filt.update(Estimate(float('nan'), 0, 0, 0, 1), 2)


@pytest.mark.parametrize('ranges,angle,increment,expected', [
    ([float('inf')], 0, 1, float('inf')),
    ([0], 0, 1, 0), ([0.3, 0.1], 0, math.pi, 0.3),
    ([0.2, 5, 0.8], math.pi, -math.pi/2, 0.8),
    ([0.6], 2*math.pi, 0, 0.6), ([0.4], math.pi/6, 0, 0.4),
    ([float('nan'), -1, float('-inf'), 2], -0.1, 0.05, 2)])
def test_front_sector_valid_cases(ranges, angle, increment, expected):
    assert front_obstacle_distance(ranges, angle, increment) == expected


@pytest.mark.parametrize('ranges,angle,increment', [
    ([], 0, 1), ([float('nan'), -1, float('-inf')], 0, 0.1),
    ([0.1], math.pi, 1), ([1], float('nan'), 1), ([1], 0, float('inf'))])
def test_unavailable_sector_fails_closed(ranges, angle, increment):
    distance = front_obstacle_distance(ranges, angle, increment)
    assert math.isnan(distance)
    assert FollowController().step((2, 0), 0, distance, 0.1)[:3] == (0, 0, 'OBSTACLE_STOP')


def test_scan_sensor_limits():
    assert front_obstacle_distance([0.05, 2], 0, 0.1, range_min=0.12, range_max=12) == 0
    assert math.isnan(front_obstacle_distance([13], 0, 0.1, range_max=12))
    for sector in (0, -1, 7, float('nan')):
        assert math.isnan(front_obstacle_distance([1], 0, 1, sector))


@pytest.mark.parametrize('field', list(vars(ControllerParams())))
@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_controller_parameter_validation(field, value):
    if value == 0 and field in ('side_slow_clearance', 'side_sector_min',
                                'side_min_speed_factor', 'los_uncertain_speed_factor'):
        FollowController(ControllerParams(**{field: value}))
    else:
        with pytest.raises(ValueError):
            FollowController(ControllerParams(**{field: value}))


@pytest.mark.parametrize('target,age,obstacle,state', [
    ((0.6, 0), 0, 2, 'TOO_CLOSE'), ((2, 0), 0, 0.65, 'OBSTACLE_STOP'),
    ((2, 0), float('nan'), 2, 'LOST'), ((2, float('inf')), 0, 2, 'LOST')])
def test_stop_boundaries_and_resume(target, age, obstacle, state):
    controller = FollowController()
    controller.step((3, 1), 0, 5, 0.1)
    assert controller.step(target, age, obstacle, 0.1)[:3] == (0, 0, state)
    v, w, actual, _ = controller.step((3, 0), 0.5, 5, 0.1)
    assert actual == 'FOLLOWING' and v == pytest.approx(0.04) and w == 0


def test_deceleration_turn_sign_and_clock_guard():
    control = FollowController()
    for _ in range(50):
        control.step((5, 1), 0, 5, 0.1)
    old_v, old_w = control.v, control.w
    v, w, _, _ = control.step((0.8, -0.2), 0, 5, 0.1)
    assert old_v-v <= 0.04+1e-12 and old_w-w <= 0.15+1e-12
    for _ in range(50):
        v, w, _, _ = control.step((-2, -2), 0, 5, 0.1)
    assert v == 0 and w == -1
    for dt in (-1, 0, float('nan'), float('inf')):
        assert control.step((5, 0), 0, 5, dt)[:3] == (0, 0, 'IDLE')
