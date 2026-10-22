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
        self.timeout, self.alpha = timeout, alpha
        self.value = self.stamp = None

    def update(self, estimate, now):
        if self.value is not None and self.stamp is not None and 0 <= now-self.stamp <= self.timeout:
            x = self.alpha*estimate.x+(1-self.alpha)*self.value.x
            y = self.alpha*estimate.y+(1-self.alpha)*self.value.y
            estimate = Estimate(x, y, math.hypot(x, y), math.atan2(y, x), estimate.quality)
        self.value, self.stamp = estimate, now
        return estimate

    def get(self, now):
        return self.value if self.stamp is not None and 0 <= now-self.stamp <= self.timeout else None
