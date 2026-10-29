"""Arrival confidence can reject uncertainty, but cannot certify localization."""
import math

import pytest

from shopping_zone_manager.localization_health import amcl_healthy


def covariance(xx=.001, yy=.008, yaw=.003, xy=0.):
    result = [0.]*36
    result[0], result[7], result[35] = xx, yy, yaw
    result[1] = result[6] = xy
    return result


def test_t71_healthy_and_false_success_covariances():
    assert amcl_healthy(covariance(.0004, .0116, .0035), .8)
    assert not amcl_healthy(covariance(.000803, .027611, .005172), 1.963)
    assert not amcl_healthy(covariance(1.6562, .5673, 1.3302), .09)
    # The first push remains falsely confident: explicitly preserve this limitation.
    assert amcl_healthy(covariance(.000377, .008185, .002569), 1.8)


def test_principal_axis_and_yaw_are_bounded():
    assert not amcl_healthy(covariance(.02, .02, xy=.01), .1)
    assert not amcl_healthy(covariance(yaw=.13), .1)
    assert amcl_healthy(covariance(xx=.15**2, yy=.15**2, yaw=.35**2), 5.)


@pytest.mark.parametrize('age', [-.01, 5.01, math.nan, math.inf])
def test_invalid_or_stale_stamp(age):
    assert not amcl_healthy(covariance(), age)


@pytest.mark.parametrize('cov', [None, [], [0.]*36, covariance(xx=-1.),
                                 covariance(xx=math.nan), covariance(xy=1.)])
def test_missing_nonfinite_or_impossible_covariance(cov):
    assert not amcl_healthy(cov, 0.)


def test_nonsymmetric_covariance_is_invalid():
    cov = covariance()
    cov[1] = .001
    assert not amcl_healthy(cov, .1)


def test_map_odom_jump_monitor():
    from shopping_zone_manager.localization_health import MapOdomJumpMonitor
    m = MapOdomJumpMonitor(0.3, 0.3)
    for i in range(50):
        assert not m.update(0.004*i, 0.0, 0.002*i)  # slow drift correction is fine
    assert m.update(5.0, 0.0, 0.1)                   # kidnap-like jump
    m.reset()
    assert not m.jumped and not m.update(5.01, 0.0, 0.1)
    assert m.update(5.01, 0.0, 2.2)                  # yaw flip
    m.reset()
    assert m.update(float('nan'), 0.0, 0.0)


def _box_world():
    # 10 x 10 m room, 0.1 m cells, walls on the border, origin (-5, -5)
    w = h = 100
    grid = [0]*(w*h)
    for i in range(w):
        for j in (0, h-1):
            grid[j*w+i] = grid[i*w+j] = 100
    return grid, w, h


def _ray(x, y, a, lo=-4.95, hi=4.95):
    best = float('inf')
    c, s = math.cos(a), math.sin(a)
    for wall, d in ((hi, c), (lo, c)):
        if abs(d) > 1e-9 and (wall-x)/d > 0:
            best = min(best, (wall-x)/d)
    for wall, d in ((hi, s), (lo, s)):
        if abs(d) > 1e-9 and (wall-y)/d > 0:
            best = min(best, (wall-y)/d)
    return best


def test_scan_map_agreement_detects_wrong_pose():
    from shopping_zone_manager.localization_health import scan_map_agreement
    grid, w, h = _box_world()
    true_pose = (1.0, -2.0, 0.0)
    n = 360
    inc = 2*math.pi/n
    ranges = [_ray(true_pose[0], true_pose[1], -math.pi+i*inc) for i in range(n)]
    args = (grid, w, h, 0.1, (-5.0, -5.0))
    good = scan_map_agreement(*args, true_pose, ranges, -math.pi, inc, 12.0)
    assert good > 0.9
    wrong = scan_map_agreement(*args, (-2.5, 1.5, 2.11), ranges, -math.pi, inc, 12.0)
    assert wrong < 0.4
    assert scan_map_agreement(*args, true_pose, [float('inf')]*n, -math.pi, inc, 12.0) is None
