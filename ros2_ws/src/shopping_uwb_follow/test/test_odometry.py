"""Exercise noise statistics and aligned drift without ROS."""
import ast
import math
from pathlib import Path
import statistics
import pytest
from shopping_uwb_follow.odom_noise import WheelNoise
from shopping_uwb_follow.odom_metrics import DriftMetrics, Pose2, interpolate_pose


def test_noise_seed_and_scale():
    a, b = WheelNoise(seed=8), WheelNoise(seed=8)
    assert [a.sample(0.3, 0.1) for _ in range(20)] == [b.sample(0.3, 0.1) for _ in range(20)]
    samples = [a.sample(1.0, 1.0) for _ in range(20000)]
    for column, mean, stddev in ((0, 1.01, .01), (1, 1.02, .02)):
        values = [s[column] for s in samples]
        assert statistics.mean(values) == pytest.approx(mean, abs=.0005)
        assert statistics.stdev(values) == pytest.approx(stddev, rel=.03)
    cov = a.covariance()
    assert cov[0] >= a.linear_stddev**2
    assert cov[35] >= a.yaw_stddev**2
    assert cov[35] > .005**2  # Trust gyro more than skid-steer yaw.
    assert len(cov) == 36


@pytest.mark.parametrize('kwargs', [{'linear_scale': float('nan')}, {'yaw_scale': 0},
                                    {'linear_stddev': -1}, {'yaw_stddev': float('inf')}])
def test_bad_noise_configuration(kwargs):
    with pytest.raises(ValueError):
        WheelNoise(**kwargs)


def test_bad_noise_input():
    with pytest.raises(ValueError):
        WheelNoise().sample(float('nan'), 0)


def test_interpolation_wrap_and_missing_data():
    a, b = Pose2(0, 0, 0, math.radians(179)), Pose2(.1, 2, 4, math.radians(-179))
    middle = interpolate_pose(a, b, .05)
    assert (middle.x, middle.y) == (1, 2)
    assert abs(middle.yaw) == pytest.approx(math.pi)
    assert interpolate_pose(a, b, -.01) is None
    assert interpolate_pose(a, b, .11) is None
    assert interpolate_pose(a, a, 0) is None
    assert interpolate_pose(a, b, .05, max_gap=.05) is None
    with pytest.raises(ValueError):
        interpolate_pose(a, b, float('nan'))


def test_drift_distance_and_wrapped_heading():
    metrics = DriftMetrics()
    first = metrics.update(Pose2(0, 0, 0, 0), Pose2(0, 0, 0, 0))
    assert first.translation_per_metre is None and first.heading_per_metre is None
    metrics.update(Pose2(1, 3, 0, 0), Pose2(1, 3, 0, 0))
    result = metrics.update(Pose2(2, 6, 8, -math.pi+.1), Pose2(2, 3, 4, math.pi-.1))
    assert result.translation == 5
    assert result.heading == pytest.approx(.2)
    assert result.distance == 7  # Accumulated path, not displacement from spawn.
    assert result.translation_per_metre == pytest.approx(5/7)
    assert result.heading_per_metre == pytest.approx(.2/7)
    with pytest.raises(ValueError):
        metrics.update(Pose2(3, 0, 0, 0), Pose2(4, 0, 0, 0))
    with pytest.raises(ValueError):
        metrics.update(Pose2(2, 0, 0, 0), Pose2(2, 0, 0, 0))
    with pytest.raises(ValueError):
        Pose2(0, float('inf'), 0, 0)


def test_pure_odometry_modules_have_no_ros_imports():
    package = Path(__file__).parents[1]/'shopping_uwb_follow'
    for name in ('odom_noise.py', 'odom_metrics.py'):
        for node in ast.walk(ast.parse((package/name).read_text())):
            imports = ([a.name for a in node.names] if isinstance(node, ast.Import)
                       else [node.module or ''] if isinstance(node, ast.ImportFrom) else [])
            assert all(i.split('.')[0] != 'rclpy' for i in imports)
