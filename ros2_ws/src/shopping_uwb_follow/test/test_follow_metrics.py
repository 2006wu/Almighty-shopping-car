import math
import pytest
from shopping_uwb_follow.follow_metrics import (
    FollowSample, evaluate_series, sample_metrics, synchronized_sample)


def test_rotated_translated_estimate_and_true_distance():
    sample = FollowSample(1, (2, 0), (3, 4, math.pi/2), (3, 7))
    assert sample_metrics(sample) == pytest.approx((1, 3))
    assert sample_metrics(FollowSample(0, (0, 2), (3, 4, -math.pi/2), (5, 4))) == pytest.approx((0, 2))


def test_series_statistics():
    samples = [FollowSample(1, (2, 0), (0, 0, 0), (2, 0)),
               FollowSample(3, (2, 0), (1, 0, 0), (5, 0))]
    assert evaluate_series(samples) == pytest.approx({
        'count': 2, 'duration': 2, 'estimate_error_mean': 1,
        'estimate_error_rmse': math.sqrt(2), 'estimate_error_max': 2,
        'follow_distance_mean': 3, 'follow_distance_min': 2, 'follow_distance_max': 4})


@pytest.mark.parametrize('samples', [[],
    [FollowSample(0, (1, 0), (0, 0, 0), (1, 0))]*2,
    [FollowSample(2, (1, 0), (0, 0, 0), (1, 0)),
     FollowSample(1, (1, 0), (0, 0, 0), (1, 0))],
    [FollowSample(0, (float('nan'), 0), (0, 0, 0), (1, 0))],
    [FollowSample(float('inf'), (1, 0), (0, 0, 0), (1, 0))],
    [FollowSample(0, (1, 0), (0, 0), (1, 0))]])
def test_reject_invalid_series(samples):
    with pytest.raises(ValueError):
        evaluate_series(samples)


def test_single_sample_statistics():
    result = evaluate_series([FollowSample(0, (1, 0), (0, 0, 0), (1, 0))])
    assert result['count'] == 1 and result['duration'] == 0
    assert result['estimate_error_rmse'] == 0


def test_synchronization_missing_stale_and_clock_reset():
    estimate = (1, (1, 0))
    pose = (1, (0, 0, 0))
    truth = (1.125, (1, 0))
    assert synchronized_sample(estimate, pose, truth, 0.125) is not None
    assert synchronized_sample(estimate, pose, truth, 0.1) is None
    assert synchronized_sample(estimate, None, truth) is None
    assert synchronized_sample((0, (1, 0)), pose, truth) is None
    assert synchronized_sample((float('nan'), (1, 0)), pose, truth) is None
    with pytest.raises(ValueError):
        synchronized_sample(estimate, pose, truth, -1)
