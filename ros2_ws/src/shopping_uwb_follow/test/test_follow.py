import math
import pytest
from shopping_uwb_follow.uwb_model import Anchor, DEFAULT_ANCHORS, RangeSimulator
from shopping_uwb_follow.estimator import Estimate, TargetFilter, trilaterate
from shopping_uwb_follow.follow_controller import ControllerParams, FollowController, front_obstacle_distance
from shopping_uwb_follow.person_path import path_position


def ranges(x, y, anchors=DEFAULT_ANCHORS):
    return [math.hypot(x-a.x, y-a.y) for a in anchors]


@pytest.mark.parametrize('target', [(2, 0.3), (1, -0.7), (-1.2, 0.2), (0.5, 2.0)])
def test_exact_trilateration(target):
    result = trilaterate(DEFAULT_ANCHORS, ranges(*target))
    assert (result.x, result.y) == pytest.approx(target, abs=1e-6)
    assert result.quality == pytest.approx(1)


def test_noisy_trilateration():
    simulator = RangeSimulator(noise_std=0.005, seed=42)
    errors = []
    for _ in range(150):
        result = trilaterate(DEFAULT_ANCHORS, simulator.sample(2, 0.3))
        errors.append(math.hypot(result.x-2, result.y-0.3))
    assert sum(errors)/len(errors) < 0.1


def test_two_anchor_ambiguity():
    anchors = DEFAULT_ANCHORS[:2]
    observed = ranges(2, 0.3, anchors)
    front = trilaterate(anchors, observed)
    back = trilaterate(anchors, observed, previous=(-2, 0.2))
    assert (front.x, front.y) == pytest.approx((2, 0.3))
    assert (back.x, back.y) == pytest.approx((-2, 0.3))
    assert front.quality == pytest.approx(0.5)


def test_drop_one_anchor():
    observed = ranges(2, 0.3)
    observed[2] = float('nan')
    assert trilaterate(DEFAULT_ANCHORS, observed).x == pytest.approx(2)


@pytest.mark.parametrize('anchors,observed', [
    (DEFAULT_ANCHORS, [1, float('nan'), float('nan')]),
    ([Anchor(0, 0)]*3, [1, 1, 1]),
    ([Anchor(0, 0), Anchor(1, 0), Anchor(2, 0)], [1, 2, 3]),
    (DEFAULT_ANCHORS[:2], [0.1, 10]),
    (DEFAULT_ANCHORS, [1, 2])])
def test_invalid_measurements(anchors, observed):
    with pytest.raises(ValueError):
        trilaterate(anchors, observed)


def test_filter_timeout_and_clock_reset():
    filt = TargetFilter(timeout=0.5)
    estimate = Estimate(2, 0, 2, 0, 1)
    assert filt.get(0) is None
    filt.update(estimate, 1)
    assert filt.get(1.4) == estimate
    assert filt.get(1.51) is None
    assert filt.get(0) is None
    filt.update(Estimate(4, 0, 4, 0, 1), 1.1)
    assert filt.get(1.1).x == pytest.approx(3.2)
    filt.update(estimate, 0)
    assert filt.get(0).x == 2


@pytest.mark.parametrize('target,age,obstacle,enabled,state', [
    ((2, 0), 0, 5, False, 'IDLE'),
    (None, 0, 5, True, 'LOST'),
    ((2, 0), 0.51, 5, True, 'LOST'),
    ((2, 0), -1, 5, True, 'LOST'),
    ((float('nan'), 0), 0, 5, True, 'LOST'),
    ((0.3, 0.2), 0, 5, True, 'TOO_CLOSE'),
    ((2, 0), 0, 0.4, True, 'OBSTACLE_STOP'),
    ((2, 0), 0, float('nan'), True, 'OBSTACLE_STOP')])
def test_immediate_safety_stops(target, age, obstacle, enabled, state):
    controller = FollowController()
    for _ in range(30):
        controller.step((3, 1), 0, 5, 0.1)
    v, w, actual, _ = controller.step(target, age, obstacle, 0.1, enabled)
    assert v == w == 0 and actual == state


def test_velocity_acceleration_and_no_reverse():
    controller = FollowController()
    previous = (0, 0)
    for _ in range(100):
        v, w, state, _ = controller.step((10, 5), 0, float('inf'), 0.05)
        assert 0 <= v <= 0.5 and abs(w) <= 1
        assert abs(v-previous[0]) <= 0.4*0.05+1e-10
        assert abs(w-previous[1]) <= 1.5*0.05+1e-10
        previous = v, w
    for _ in range(100):
        v, w, _, _ = controller.step((-2, -2), 0, 5, 0.05)
        assert v >= 0 and abs(w) <= 1
    assert v == 0


def test_near_follow_distance_and_dt_guard():
    controller = FollowController()
    assert controller.step((0.8, 0), 0, 5, 0.1)[0] == 0
    assert controller.step((4, 0), 0, 5, -1)[0] == 0
    assert controller.step((4, 0), 0, 5, 100)[0] <= 0.08+1e-10


def test_invalid_controller_configuration():
    with pytest.raises(ValueError):
        FollowController(ControllerParams(stop_distance=2))


def test_front_sector():
    scan = [8.0]*361
    scan[180] = 0.4
    scan[0] = 0.1
    scan[175] = float('nan')
    assert front_obstacle_distance(scan, -math.pi, math.pi/180) == 0.4
    assert front_obstacle_distance([float('inf')]*10, -0.1, 0.02) == float('inf')
    assert math.isnan(front_obstacle_distance([], 0, 0))
    assert front_obstacle_distance([0.3], 2*math.pi, 0) == 0.3


def test_range_determinism_noise_and_faults():
    first, second = RangeSimulator(seed=7), RangeSimulator(seed=7)
    assert [first.sample(2, 0) for _ in range(10)] == [second.sample(2, 0) for _ in range(10)]
    exact = RangeSimulator(noise_std=0, bias=0.2, nlos_probability=1, nlos_bias=0.5)
    assert exact.sample(2, 0) == pytest.approx([r+0.7 for r in ranges(2, 0)])
    assert all(math.isnan(r) for r in RangeSimulator(dropout=1).sample(2, 0))
    with pytest.raises(ValueError):
        RangeSimulator(dropout=2)


def test_shared_path_loop():
    path = {'speed': 1, 'waypoints': [[0, 0], [2, 0], [2, 2], [0, 2]]}
    assert path_position(path, 1) == (1, 0)
    assert path_position(path, 3) == (2, 1)
    assert path_position(path, 9) == (1, 0)


def test_obstacle_stops_rotation_immediately_and_resume_is_limited():
    controller = FollowController()
    for _ in range(10):
        controller.step((3, 1), 0, 5, 0.1)
    assert controller.w > 0
    for _ in range(30):
        v, w, state, _ = controller.step((1, -2), 0, 0.4, 0.1)
        assert v == w == 0 and state == 'OBSTACLE_STOP'
    v, w, state, _ = controller.step((3, -1), 0, 5, 0.1)
    assert state == 'FOLLOWING' and v == pytest.approx(0.04)
    assert -0.15-1e-12 <= w < 0


@pytest.mark.parametrize('target,age,obstacle,dt,enabled,state', [
    ((1, 1), 0, float('nan'), 0.1, True, 'OBSTACLE_STOP'),
    ((0.2, 0.2), 0, 0.4, 0.1, True, 'TOO_CLOSE'),
    ((1, 1), 0.51, 0.4, 0.1, True, 'LOST'),
    (None, 0, 0.4, 0.1, True, 'LOST'),
    ((1, 1), 0, 0.4, 0.1, False, 'IDLE'),
    ((1, 1), 0, 0.4, -1, True, 'IDLE')])
def test_rotation_still_fully_stops_on_safety_fault(target, age, obstacle, dt, enabled, state):
    controller = FollowController()
    controller.step((1, 1), 0, 0.4, 0.1)
    assert controller.step(target, age, obstacle, dt, enabled)[:3] == (0, 0, state)


def test_obstacle_stops_rotation_even_with_a_large_control_interval():
    controller = FollowController()
    assert controller.step((1, 2), 0, 0.4, 100)[:3] == (0, 0, 'OBSTACLE_STOP')
