import math


def path_position(path, elapsed):
    points, speed = path['waypoints'], float(path['speed'])
    segments = [(a, b, math.hypot(b[0]-a[0], b[1]-a[1]))
                for a, b in zip(points, points[1:]+points[:1])]
    total = sum(s[2] for s in segments)
    if speed <= 0 or total <= 0:
        raise ValueError('Path requires positive speed and length')
    travel = max(0.0, elapsed)*speed % total
    for a, b, length in segments:
        if travel <= length and length > 0:
            ratio = travel/length
            return a[0]+ratio*(b[0]-a[0]), a[1]+ratio*(b[1]-a[1])
        travel -= length
    return tuple(points[0])


def model_command_position(path, elapsed, model_lag_compensation_s=0.3):
    """Predict only the visual model command; UWB truth stays at current time.

    Fortress set_pose has no scheduled application timestamp. Compensation is
    calibrated in simulation seconds and includes command/hold latency.
    """
    if (not math.isfinite(elapsed) or not math.isfinite(model_lag_compensation_s)
            or model_lag_compensation_s < 0):
        raise ValueError('Model timing must be finite with nonnegative compensation')
    return path_position(path, elapsed+model_lag_compensation_s)
