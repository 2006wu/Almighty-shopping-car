import math
import random
from dataclasses import dataclass


@dataclass(frozen=True)
class Anchor:
    x: float
    y: float


DEFAULT_ANCHORS = (Anchor(0.0, 0.22), Anchor(0.0, -0.22), Anchor(-0.25, 0.0))


class RangeSimulator:
    def __init__(self, anchors=DEFAULT_ANCHORS, noise_std=0.02, bias=0.0,
                 nlos_probability=0.0, nlos_bias=0.5, dropout=0.0, seed=None):
        if (len(anchors) not in (2, 3) or noise_std < 0 or nlos_bias < 0
                or not all(math.isfinite(v) for v in (noise_std, bias, nlos_bias))
                or not all(math.isfinite(a.x) and math.isfinite(a.y) for a in anchors)):
            raise ValueError('Require two or three anchors and nonnegative noise')
        if not 0 <= dropout <= 1 or not 0 <= nlos_probability <= 1:
            raise ValueError('Probabilities must be in [0, 1]')
        self.anchors, self.noise_std, self.bias = tuple(anchors), noise_std, bias
        self.nlos_probability, self.nlos_bias, self.dropout = nlos_probability, nlos_bias, dropout
        self.rng = random.Random(seed)

    def sample(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('Target coordinates must be finite')
        result = []
        for a in self.anchors:
            if self.rng.random() < self.dropout:
                result.append(float('nan'))
                continue
            nlos = self.nlos_bias if self.rng.random() < self.nlos_probability else 0.0
            result.append(max(0.0, math.hypot(x-a.x, y-a.y) + self.bias + nlos +
                              self.rng.gauss(0.0, self.noise_std)))
        return result
