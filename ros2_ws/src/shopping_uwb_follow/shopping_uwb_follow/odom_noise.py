"""Seeded simulation wheel velocity errors, independent of ROS."""
import math
import random


class WheelNoise:
    def __init__(self, seed=42, linear_scale=1.01, yaw_scale=1.02,
                 linear_stddev=0.01, yaw_stddev=0.02):
        values = (linear_scale, yaw_scale, linear_stddev, yaw_stddev)
        if not all(math.isfinite(v) for v in values) or min(values) <= 0:
            raise ValueError('Scales and standard deviations must be finite and positive')
        self.rng = random.Random(seed)
        self.linear_scale, self.yaw_scale = linear_scale, yaw_scale
        self.linear_stddev, self.yaw_stddev = linear_stddev, yaw_stddev

    def sample(self, vx, vyaw):
        if not all(math.isfinite(v) for v in (vx, vyaw)):
            raise ValueError('Velocities must be finite')
        return (vx*self.linear_scale + self.rng.gauss(0, self.linear_stddev),
                vyaw*self.yaw_scale + self.rng.gauss(0, self.yaw_stddev))

    def covariance(self):
        # Conservative floors account for skid slip and unmodelled scale error.
        result = [0.0]*36
        for i in (0, 7, 14, 21, 28, 35):
            result[i] = 1e6
        result[0] = max(0.02**2, self.linear_stddev**2)
        result[35] = max(0.10**2, self.yaw_stddev**2)
        return result
