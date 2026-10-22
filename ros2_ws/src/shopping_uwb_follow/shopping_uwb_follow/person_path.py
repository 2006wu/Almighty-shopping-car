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
