"""Threshold boundaries, angular wraparound and missing-data failures."""
import math

import pytest

from shopping_zone_manager.localization_metrics import (
    aggregate, evaluate_zone, pose_error, statistics, validate_truth_frames,
)
from shopping_zone_manager.zones import Zone

GOAL = Zone('test', (), 0, 0, 0, 'map')


@pytest.mark.parametrize('parent,child', [
    ('sim_world', 'base_footprint_gt'), ('odom', 'base_footprint'),
    ('world', 'base_footprint'), ('map', 'base_footprint'),
])
def test_accept_simulator_and_legacy_truth_frames(parent, child):
    validate_truth_frames(parent, child)


@pytest.mark.parametrize('parent,child', [
    ('sim_world', 'base_footprint'), ('odom', 'base_footprint_gt'),
    ('world', 'base_footprint_gt'), ('map', 'base_footprint_gt'),
    ('sim_world', 'base_link'), ('odom', 'base_link'),
    ('unknown', 'base_footprint'), ('unknown', 'base_footprint_gt'), ('', ''),
])
def test_reject_unknown_or_mixed_truth_frames_with_clear_message(parent, child):
    with pytest.raises(ValueError, match='Unsupported truth frames') as error:
        validate_truth_frames(parent, child)
    assert repr(parent) in str(error.value)
    assert repr(child) in str(error.value)
    assert '(sim_world, base_footprint_gt)' in str(error.value)
    assert '(odom|world|map, base_footprint)' in str(error.value)


def test_pose_error_wraps_angles():
    distance, angle = pose_error((0, 0, math.radians(179)), (0.03, 0.04, math.radians(-179)))
    assert distance == pytest.approx(0.05)
    assert math.degrees(angle) == pytest.approx(2)


def test_statistics():
    assert statistics([0, 3, 4]) == pytest.approx({'mean': 7/3, 'rmse': math.sqrt(25/3), 'max': 4})


def test_stationary_samples_and_goal_errors():
    row = evaluate_zone(GOAL, [((0.12, 0, 0), (0.02, 0, 0))]*5, True)
    assert row['pass']
    assert row['final_map_goal_m'] == 0.12
    assert row['final_truth_goal_m'] == 0.02
    assert row['localization_m']['max'] == pytest.approx(0.10)
    assert aggregate([row], 1)['pass']
    assert not aggregate([row], 6)['pass']
    assert not aggregate([], 0)['pass']


@pytest.mark.parametrize('estimate,truth', [((0.15, 0, 0), (0, 0, 0)),
                                          ((0, 0, math.radians(5)), (0, 0, 0)),
                                          ((0.31, 0, 0), (0.31, 0, 0))])
def test_thresholds_fail_at_localization_limits_or_beyond_zone_limit(estimate, truth):
    assert not evaluate_zone(GOAL, [(estimate, truth)]*5, True)['pass']


def test_zone_boundary_and_all_samples_must_pass():
    good = ((0.3, 0, 0), (0.3, 0, 0))
    bad = ((1, 0, 0), (0, 0, 0))
    assert evaluate_zone(GOAL, [good]*5, True)['pass']
    assert not evaluate_zone(GOAL, [bad]+[good]*5, True)['pass']
    assert not evaluate_zone(GOAL, [good]*4, True)['pass']
    assert not evaluate_zone(GOAL, [good]*5, False)['pass']
    assert not evaluate_zone(GOAL, [], True)['pass']


@pytest.mark.parametrize('values', [[], [float('nan')], [-1], [float('inf')]])
def test_invalid_statistics(values):
    with pytest.raises(ValueError):
        statistics(values)


def test_nonfinite_pose_cannot_pass():
    with pytest.raises(ValueError):
        pose_error((float('nan'), 0, 0), (0, 0, 0))


def test_aggregate_weights_statistics_by_sample_count():
    first = evaluate_zone(GOAL, [((0.1, 0, 0), (0, 0, 0))]*5, True)
    second = evaluate_zone(GOAL, [((0, 0, 0), (0, 0, 0))]*10, True)
    report = aggregate([first, second], 2)
    assert report['pass']
    assert report['localization_m']['mean'] == pytest.approx(0.1/3)
    assert report['localization_m']['rmse'] == pytest.approx(math.sqrt(0.01/3))
    assert report['localization_m']['max'] == 0.1
