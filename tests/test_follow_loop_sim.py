"""Closed-loop tracking and calibrated model-command timing regressions."""
from dataclasses import replace
import importlib.util
import math
from pathlib import Path

import pytest

from shopping_uwb_follow.estimator import TargetVelocityEstimator
from shopping_uwb_follow.follow_controller import ControllerParams, FollowController
from shopping_uwb_follow.person_path import model_command_position, path_position
from shopping_uwb_follow.evaluation_metrics import sync_report

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('follow_loop_sim', ROOT/'scripts/eval/follow_loop_sim.py')
loop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(loop)


def historical_path():
    path, _ = loop.load_inputs()
    return {**path, 'speed': 0.35}


@pytest.fixture(scope='module')
def world_runs():
    return (loop.simulate(path=historical_path(), legacy=True, world=True),
            loop.simulate(path=historical_path(), world=True, params=replace(
                ControllerParams(), follow_distance=1.0, stop_distance=0.6,
                max_linear=0.5, max_angular=1.0, max_linear_accel=0.4,
                kp_distance=0.7, velocity_feedforward=1.0, follow_time_headway=0.0,
                distance_hysteresis_enter=0.04, distance_hysteresis_exit=0.08)))


def test_open_path_feedforward_removes_proportional_lag():
    old, new = (loop.simulate(path=historical_path(), legacy=True),
                loop.simulate(path=historical_path(), params=ControllerParams(follow_time_headway=0.0)))
    assert 1.35 < old['eligible']['follow_distance_mean'] < 1.55
    assert new['eligible_fraction'] == 1
    error = new['eligible']['distance_error_m']
    assert error['mean'] < 0.5 and error['rmse'] < 1.0
    assert error['mean'] < old['eligible']['distance_error_m']['mean']/3
    no_accel = loop.simulate(path=historical_path(), legacy=True, acceleration=False)
    assert abs(no_accel['all']['follow_distance_mean']-old['all']['follow_distance_mean']) < 0.02


def test_historical_world_experiment_preserves_error_and_exclusion_coverage(world_runs):
    old, new = world_runs
    assert 2.5 < old['all']['follow_distance_mean'] < 4.0
    assert old['eligible_fraction'] < 0.65  # Do not hide the blocked path.
    assert old['excluded']['occluded'] > 0
    assert new['eligible_fraction'] > old['eligible_fraction']
    error = new['eligible']['distance_error_m']
    assert error['mean'] < 0.5 and error['rmse'] < 1.0
    for kind in ('straight', 'turn'):
        assert new['segments_distance_error_m'][kind]['count'] >= 10
        assert new['segments_distance_error_m'][kind]['rmse'] < 1.0


def test_sustained_reduced_cap_cannot_track_faster_person():
    result = loop.simulate(path={**historical_path(), 'speed': 1.0}, speed_factor=0.4)
    assert result['eligible']['distance_error_m']['rmse'] > 1.0


def settle(controller, distance, velocity=(0, 0), factor=1):
    for _ in range(60):
        result = controller.step((distance, 0), 0, math.inf, 0.05,
                                 target_velocity=velocity, speed_factor=factor,
                                 clearance=math.inf)
    return result


def test_feedforward_target_motion_and_physical_speed_cap():
    control = FollowController(ControllerParams(velocity_feedforward=1.0, follow_time_headway=0.0,
        distance_hysteresis_enter=0.04, distance_hysteresis_exit=0.08))
    assert settle(control, 1.2, (0.35, 0))[0] == pytest.approx(0.35)
    assert settle(control, 1.2)[0] == 0
    assert settle(control, 4, (100, 0), 0.4)[0] == pytest.approx(0.4)
    assert settle(control, 1.2, (-0.3, 0))[0] == 0  # Never reverse.
    assert settle(control, 1.2, (math.nan, 0))[0] == 0


def test_distance_hysteresis_retains_feedforward_but_suppresses_chatter():
    control = FollowController(ControllerParams(velocity_feedforward=1.0, follow_time_headway=0.0,
        distance_hysteresis_enter=0.04, distance_hysteresis_exit=0.08))
    assert settle(control, 1.23, (0.3, 0))[0] == pytest.approx(0.3)
    assert control.holding_distance
    assert settle(control, 1.26, (0.3, 0))[0] == pytest.approx(0.3)
    assert control.holding_distance
    assert settle(control, 1.29, (0.3, 0))[0] == pytest.approx(0.3+1.0*0.09)
    assert not control.holding_distance
    assert settle(control, 1.26, (0.3, 0))[0] == pytest.approx(0.3+1.0*0.06)
    assert settle(control, 1.22, (0.3, 0))[0] == pytest.approx(0.3)
    with pytest.raises(ValueError):
        FollowController(ControllerParams(distance_hysteresis_enter=0.2))


@pytest.mark.parametrize('override,state', [
    ({'target': (0.6, 0)}, 'TOO_CLOSE'), ({'obstacle': 0.4}, 'OBSTACLE_STOP'),
    ({'obstacle': math.nan}, 'OBSTACLE_STOP'), ({'target': None}, 'LOST'),
    ({'age': 0.51}, 'LOST'), ({'enabled': False}, 'IDLE')])
def test_feedforward_never_bypasses_safety(override, state):
    control = FollowController(ControllerParams(velocity_feedforward=1.0, follow_time_headway=0.0,
        distance_hysteresis_enter=0.04, distance_hysteresis_exit=0.08))
    settle(control, 2, (0.4, 0))
    args = dict(target=(2, 1), age=0, obstacle=math.inf, dt=0.05,
                target_velocity=(0.6, 0.4))
    args.update(override)
    assert control.step(**args)[:3] == (0, 0, state)


def test_velocity_estimate_compensates_robot_translation_and_rotation():
    filt = TargetVelocityEstimator()
    for i in range(100):
        t = i*0.05
        pose = (0.2*t, -0.1*t, 0.3*t)
        person = (2+0.35*t, 1)
        relative = loop.relative_position(person, pose)
        vx, vy = filt.update(relative, pose, t)
    assert vx == pytest.approx(0.35*math.cos(pose[2]), abs=1e-6)
    assert vy == pytest.approx(-0.35*math.sin(pose[2]), abs=1e-6)
    assert filt.update(relative, pose, t+1) == (0, 0)
    assert filt.update(relative, pose, t-1) == (0, 0)
    assert filt.update((math.nan, 0), pose, t) == (0, 0)


@pytest.mark.parametrize('service_delay', [0.20, 0.25, 0.30])
def test_model_prediction_under_measured_service_and_hold_delay(service_delay):
    path = historical_path()
    times = [1+i/30 for i in range(1800)]
    truth = [(t, path_position(path, t)) for t in times]
    # A set_pose command is held until the next completed request. The service
    # component is an explicit assumption, not a measured Fortress test.
    def observed(period, compensation):
        return [(t, model_command_position(path,
                 math.floor((t-service_delay)/period+1e-9)*period, compensation)) for t in times]
    before = sync_report(truth, observed(0.1, 0), path)
    after = sync_report(truth, observed(0.05, 0.3), path)
    assert next(r for r in before if r['check'] == 'model_vs_path')['status'] == 'FAIL'
    assert all(row['status'] == 'PASS' for row in after)


def test_model_prediction_wraps_shared_path_and_rejects_invalid_timing():
    path, _ = loop.load_inputs()
    for t in (0, 2.5, 16.8, 94.8, 200):
        assert model_command_position(path, t, 0.3) == path_position(path, t+0.3)
    for compensation in (-1, math.nan, math.inf):
        with pytest.raises(ValueError):
            model_command_position(path, 1, compensation)


def test_realistic_browsing_pace_exposes_sustained_narrow_cap_lag():
    path, params = loop.load_inputs()
    assert path['speed'] == 0.35
    path = {**path, 'speed': 1.0}
    assert path['speed'] > params.max_linear*params.side_min_speed_factor
    result = loop.simulate(path=path, speed_factor=params.side_min_speed_factor)
    assert result['all']['distance_error_m']['rmse'] > 1.0
    assert result['limits']['max_command_linear'] <= params.max_linear*params.side_min_speed_factor


def test_controller_reports_active_limits_without_changing_safety_reason():
    control = FollowController(ControllerParams(velocity_feedforward=1.0, follow_time_headway=0.0,
        distance_hysteresis_enter=0.04, distance_hysteresis_exit=0.08))
    result = control.step((4, 1), 0, math.inf, 0.05, speed_factor=0.4)
    assert 'speed cap' in result[3] and 'linear acceleration' in result[3]
    result = control.step((1, 1), 0, math.inf, 0.05)
    assert 'heading' in result[3] and 'angular acceleration' in result[3]
    result = settle(control, 1.2, (0.18, 0))
    assert 'distance hold' in result[3]
    assert control.step((2, 0), 0, 0.5, 0.05) == (0, 0, 'OBSTACLE_STOP', 'Obstacle or unavailable scan')


@pytest.mark.parametrize('speed', [0.7, 0.9, 1.0, 1.2])
def test_fast_straight_runtime_defaults_and_informational_gap(speed):
    path = dict(speed=speed, waypoints=[[1.5, 0], [1000, 0]])
    before = loop.simulate(path=path, legacy=True, duration=60)
    after = loop.simulate(path=path, duration=60)
    error = after['performance']['steady_state_distance_error_m']
    assert error['mean'] < before['performance']['steady_state_distance_error_m']['mean']
    assert after['limits']['max_command_linear'] <= 1.0
    assert after['performance']['person_speed_m_s'] == speed
    assert after['performance']['follow_distance_m'] == 1.2
    if speed < ControllerParams().max_linear:
        no_ff = loop.simulate(path=path, duration=60, feedforward=False)
        # Default headway intentionally adds 0.4 m at 1 m/s; compare the
        # residual proportional lag after accounting for that requested gap.
        assert abs(error['mean']-after['performance']['follow_time_headway_s']*speed) < no_ff['performance']['steady_state_distance_error_m']['mean']/3


@pytest.mark.parametrize('velocity_age', [0.51, -0.1, math.nan, math.inf])
def test_stale_velocity_cannot_add_feedforward(velocity_age):
    controller = FollowController()
    for _ in range(100):
        v, _, _, _ = controller.step((1.5, 0), 0, math.inf, .05,
                                    target_velocity=(1.2, 0), velocity_age=velocity_age)
    assert v == pytest.approx(.3)


def test_velocity_projection_and_inside_follow_distance_stop():
    controller = FollowController()
    for _ in range(100):
        v, _, _, _ = controller.step((1.2, .2), 0, math.inf, .05,
                                    target_velocity=(0, 1.2))
    distance = math.hypot(1.2, .2)
    assert v == pytest.approx((distance-1.2)*1.2/distance)
    controller.v = 1.2
    assert controller.step((1.19, 0), 0, math.inf, .05, target_velocity=(1.5, 0))[0] == 0


def test_estimator_read_rejects_stale_invalid_and_rotates_current_heading():
    estimator = TargetVelocityEstimator(alpha=1)
    assert estimator.update((2, 0), (0, 0, 0), 0) == (0, 0)
    assert estimator.estimate(0, 0) == (0, 0)
    estimator.update((2.12, 0), (0, 0, 0), .1)
    assert estimator.estimate(.1, 0) == pytest.approx((1.2, 0))
    assert estimator.estimate(.1, math.pi/2) == pytest.approx((0, -1.2))
    assert estimator.estimate(.61, 0) == (0, 0)
    assert estimator.estimate(-.1, 0) == (0, 0)
    estimator.update((math.nan, 0), (0, 0, 0), .2)
    assert estimator.estimate(.2, 0) == (0, 0)
