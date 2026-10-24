import math


def path_position(path, elapsed, delay=0.0):
    # ROS float fields reject integers, and scenario YAML may contain them.
    return tuple(float(v) for v in _path_position(path, elapsed, delay))


def _path_position(path, elapsed, delay=0.0):
    points, speed = path['waypoints'], path['speed']
    if elapsed <= delay or speed == 0:
        return tuple(points[0])
    pauses = path.get('pauses', [0]*len(points))
    segments = [(a, b, math.dist(a, b)/speed, pause)
                for a, b, pause in zip(points, points[1:]+points[:1], pauses)]
    period = sum(duration+pause for _, _, duration, pause in segments)
    if period <= 0:
        return tuple(points[0])
    t = (elapsed-delay) % period
    for a, b, duration, pause in segments:
        if t < pause:
            return tuple(a)
        t -= pause
        if duration > 0 and t < duration:
            return tuple(x+(y-x)*t/duration for x, y in zip(a, b))
        t -= duration
    return tuple(points[0])
