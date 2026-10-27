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
