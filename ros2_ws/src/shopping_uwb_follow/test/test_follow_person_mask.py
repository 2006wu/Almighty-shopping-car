import math

from shopping_uwb_follow.follow_controller import FollowController, front_obstacle_distance


def _scan(person=(1.0, 0.1), shelf=None, n=720):
    inc = 2*math.pi/n
    ranges = []
    for i in range(n):
        a = -math.pi+i*inc
        r = math.inf
        for obj, rad in ((person, 0.2), (shelf, 0.05)):
            if obj is None:
                continue
            d, b = math.hypot(*obj), math.atan2(obj[1], obj[0])
            if abs(math.atan2(math.sin(a-b), math.cos(a-b))) < math.asin(min(1, rad/d)):
                r = min(r, d-rad)
        ranges.append(r)
    return ranges, -math.pi, inc


def test_exclude_ignores_only_the_person():
    ranges, amin, inc = _scan()
    assert front_obstacle_distance(ranges, amin, inc) < 0.85
    assert front_obstacle_distance(ranges, amin, inc, exclude=(1.0, 0.1, 0.4)) == math.inf
    ranges, amin, inc = _scan(shelf=(0.55, -0.2))
    masked = front_obstacle_distance(ranges, amin, inc, exclude=(1.0, 0.1, 0.4))
    assert masked < 0.65  # a shelf beside the person still stops the robot


def test_masked_person_keeps_turning_toward_target_not_moving():
    c = FollowController()
    target = (0.9, 0.5)  # ~1.03 m, misaligned
    v, w, state, _ = c.step(target, 0.0, math.inf, 0.1)
    assert state == 'FOLLOWING' and v == 0.0 and w > 0
    # Near-person cap still applies from the target distance itself.
    assert c.linear_cap(min(math.inf, 1.0)) <= c.p.near_person_max_linear
