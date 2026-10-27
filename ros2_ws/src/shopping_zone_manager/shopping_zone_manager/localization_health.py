"""Conservative arrival confidence check, using localization only, never truth."""
import math


def amcl_healthy(covariance, age, max_xy_std=0.15, max_yaw_std=0.35, max_age=5.0):
    """Bound the largest planar covariance eigenvalue and yaw variance.

    Small covariance does not prove correct localization after an unobserved slip.
    """
    if covariance is None or len(covariance) != 36:
        return False
    if (not all(math.isfinite(v) for v in (*covariance, age, max_xy_std, max_yaw_std, max_age))
            or min(max_xy_std, max_yaw_std, max_age) <= 0 or not 0 <= age <= max_age):
        return False
    xx, xy, yx, yy, yaw = (covariance[i] for i in (0, 1, 6, 7, 35))
    if min(xx, yy, yaw) <= 0 or abs(xy-yx) > 1e-6 or xx*yy < xy*xy:
        return False
    largest = (xx+yy+math.hypot(xx-yy, 2*xy))/2
    return largest <= max_xy_std**2 and yaw <= max_yaw_std**2
