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


class MapOdomJumpMonitor:
    """Flag sudden map->odom corrections (AMCL relocalization/kidnap) during a goal.

    map->odom only moves when AMCL corrects odometry; normal drift correction is
    small per update.  A large step means the map pose the goal checker used
    jumped, so a Nav2 SUCCEEDED in that goal cannot be trusted.
    """

    def __init__(self, max_xy_step=0.3, max_yaw_step=0.3):
        self.max_xy_step = max_xy_step
        self.max_yaw_step = max_yaw_step
        self.last = None
        self.jumped = False
        self.worst = (0.0, 0.0)

    def reset(self):
        """Start a new goal window; keep the last sample as the reference."""
        self.jumped = False
        self.worst = (0.0, 0.0)

    def update(self, x, y, yaw):
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            self.jumped = True
            return True
        if self.last is not None:
            lx, ly, lyaw = self.last
            dxy = math.hypot(x-lx, y-ly)
            dyaw = abs(math.atan2(math.sin(yaw-lyaw), math.cos(yaw-lyaw)))
            self.worst = (max(self.worst[0], dxy), max(self.worst[1], dyaw))
            if dxy > self.max_xy_step or dyaw > self.max_yaw_step:
                self.jumped = True
        self.last = (x, y, yaw)
        return self.jumped


def scan_map_agreement(grid, width, height, resolution, origin, pose, ranges,
                       angle_min, angle_increment, range_max, radius=0.15, max_beams=180):
    """Fraction of valid scan endpoints within ``radius`` of an occupied map cell.

    ``pose`` is the laser pose (x, y, yaw) in the map frame and ``origin`` the map
    origin (x, y); rotated map origins are not supported.  Returns ``None`` when
    there are too few valid beams to judge.  Independent of AMCL covariance, so
    it catches a confident but wrong pose (e.g. wrong aisle or rotated).
    """
    x0, y0, yaw = pose
    ox, oy = origin
    reach = max(1, int(math.ceil(radius/resolution)))
    step = max(1, len(ranges)//max_beams)
    valid = hits = 0
    for i in range(0, len(ranges), step):
        r = ranges[i]
        if not math.isfinite(r) or r <= 0.05 or r >= range_max:
            continue
        a = yaw+angle_min+i*angle_increment
        cx = int(math.floor((x0+r*math.cos(a)-ox)/resolution))
        cy = int(math.floor((y0+r*math.sin(a)-oy)/resolution))
        valid += 1
        found = False
        for dy in range(-reach, reach+1):
            yy = cy+dy
            if found or not 0 <= yy < height:
                continue
            for dx in range(-reach, reach+1):
                xx = cx+dx
                if 0 <= xx < width and grid[yy*width+xx] >= 65:
                    found = True
                    break
        hits += found
    return None if valid < 20 else hits/valid
