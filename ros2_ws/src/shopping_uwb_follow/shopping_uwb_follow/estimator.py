import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Estimate:
    x: float
    y: float
    distance: float
    bearing: float
    quality: float


def trilaterate(anchors, ranges, previous=None, refine=True):
    if (len(anchors) not in (2, 3)
            or not all(math.isfinite(a.x) and math.isfinite(a.y) for a in anchors)
            or (previous is not None and (len(previous) != 2 or not all(math.isfinite(v) for v in previous)))):
        raise ValueError('Require two or three finite anchors and a finite previous position')
    pairs = [(a, r) for a, r in zip(anchors, ranges) if math.isfinite(r) and r > 0]
    if len(pairs) < 2 or len(ranges) != len(anchors):
        raise ValueError('At least two valid ranges required')
    a, ra = pairs[0]
    b, rb = pairs[1]
    dx, dy = b.x-a.x, b.y-a.y
    length = math.hypot(dx, dy)
    if length < 1e-8:
        raise ValueError('Coincident anchors')
    if len(pairs) == 2:
        along = (ra*ra-rb*rb+length*length)/(2*length)
        height2 = ra*ra-along*along
        if height2 < -0.05:
            raise ValueError('Inconsistent ranges')
        height = math.sqrt(max(0.0, height2))
        cx, cy = a.x+along*dx/length, a.y+along*dy/length
        candidates = [(cx-height*dy/length, cy+height*dx/length),
                      (cx+height*dy/length, cy-height*dx/length)]
        x, y = min(candidates, key=lambda q: math.hypot(q[0]-previous[0], q[1]-previous[1])) if previous else max(candidates, key=lambda q: q[0])
    else:
        c, rc = pairs[2]
        A, B, C, D = 2*dx, 2*dy, 2*(c.x-a.x), 2*(c.y-a.y)
        u = ra*ra-rb*rb+b.x*b.x+b.y*b.y-a.x*a.x-a.y*a.y
        v = ra*ra-rc*rc+c.x*c.x+c.y*c.y-a.x*a.x-a.y*a.y
        det = A*D-B*C
        if abs(det) < 1e-8:
            raise ValueError('Collinear anchors')
        x, y = (u*D-B*v)/det, (A*v-u*C)/det
        if refine:
            for _ in range(8):
                hxx = hxy = hyy = gx = gy = 0.0
                for anchor, r in pairs:
                    d = max(1e-8, math.hypot(x-anchor.x, y-anchor.y))
                    jx, jy = (x-anchor.x)/d, (y-anchor.y)/d
                    hxx += jx*jx; hxy += jx*jy; hyy += jy*jy
                    gx += jx*(d-r); gy += jy*(d-r)
                det = hxx*hyy-hxy*hxy
                if abs(det) < 1e-10:
                    break
                x -= (hyy*gx-hxy*gy)/det
                y -= (hxx*gy-hxy*gx)/det
    residual = math.sqrt(sum((math.hypot(x-a.x, y-a.y)-r)**2 for a, r in pairs)/len(pairs))
    quality = (1.0 if len(pairs) == 3 else 0.5)/(1+10*residual)
    return Estimate(x, y, math.hypot(x, y), math.atan2(y, x), quality)


class TargetFilter:
    def __init__(self, timeout=0.5, alpha=0.6):
        if not math.isfinite(timeout) or timeout <= 0 or not math.isfinite(alpha) or not 0 < alpha <= 1:
            raise ValueError('Require positive timeout and alpha in (0, 1]')
        self.timeout, self.alpha = timeout, alpha
        self.value = self.stamp = None

    def update(self, estimate, now):
        if not math.isfinite(now) or not all(math.isfinite(v) for v in vars(estimate).values()):
            raise ValueError('Estimate and timestamp must be finite')
        if self.value is not None and self.stamp is not None and 0 <= now-self.stamp <= self.timeout:
            x = self.alpha*estimate.x+(1-self.alpha)*self.value.x
            y = self.alpha*estimate.y+(1-self.alpha)*self.value.y
            estimate = Estimate(x, y, math.hypot(x, y), math.atan2(y, x), estimate.quality)
        self.value, self.stamp = estimate, now
        return estimate

    def get(self, now):
        return self.value if self.stamp is not None and 0 <= now-self.stamp <= self.timeout else None


class TargetVelocityEstimator:
    """Differentiate UWB positions in odom, then express velocity in base_link.

    Using successive base-frame positions directly mistakes ego motion for target
    velocity. Gaps/clock resets discard history; bounded EMA limits noise spikes.
    """
    def __init__(self, timeout=0.5, alpha=0.2, max_speed=1.5):
        if (not all(math.isfinite(v) and v > 0 for v in (timeout, alpha, max_speed))
                or alpha > 1):
            raise ValueError('Invalid target velocity filter parameters')
        self.timeout, self.alpha, self.max_speed = timeout, alpha, max_speed
        self.reset()

    def reset(self):
        self.previous = self.stamp = None
        self.valid = False
        self.velocity = (0.0, 0.0)

    def update(self, target, pose, now):
        if not all(math.isfinite(v) for v in (*target, *pose, now)):
            self.reset()
            return (0.0, 0.0)
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        world = (x+c*target[0]-s*target[1], y+s*target[0]+c*target[1])
        dt = now-self.stamp if self.stamp is not None else 0.0
        if self.previous is not None and 0 < dt <= self.timeout:
            self.valid = True
            raw = tuple((a-b)/dt for a, b in zip(world, self.previous))
            scale = min(1.0, self.max_speed/max(math.hypot(*raw), 1e-9))
            self.velocity = tuple(self.alpha*v*scale+(1-self.alpha)*old
                                  for v, old in zip(raw, self.velocity))
        else:
            self.valid = False
            self.velocity = (0.0, 0.0)
        self.previous, self.stamp = world, now
        vx, vy = self.velocity
        return c*vx+s*vy, -s*vx+c*vy

    def estimate(self, now, yaw):
        """Return fresh absolute target velocity along the current base axes."""
        if (not self.valid or self.stamp is None or not math.isfinite(now)
                or not math.isfinite(yaw) or not 0 <= now-self.stamp <= self.timeout):
            return (0.0, 0.0)
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = self.velocity
        return c*vx+s*vy, -s*vx+c*vy
