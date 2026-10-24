"""ROS-free simulation measurements and side-by-side comparison."""
import math


def goal_metrics(samples, target):
    """Samples are (simulation_seconds, truth_x, truth_y), including goal start/end."""
    if len(samples) < 2 or any(not all(math.isfinite(v) for v in s) for s in samples):
        raise ValueError('Need at least two finite truth samples')
    if any(b[0] <= a[0] or b[0] - a[0] > 0.5 for a, b in zip(samples, samples[1:])):
        raise ValueError('Truth clock reversed, duplicated or coverage gap exceeded 0.5 s')
    if not all(math.isfinite(v) for v in target):
        raise ValueError('Invalid target')
    return dict(seconds=samples[-1][0]-samples[0][0],
                path_m=sum(math.hypot(b[1]-a[1], b[2]-a[2]) for a, b in zip(samples, samples[1:])),
                final_error_m=math.hypot(samples[-1][1]-target[0], samples[-1][2]-target[1]))


def comparison_table(apt, fork):
    if apt['stack'] != 'apt' or fork['stack'] != 'fork':
        raise ValueError('Expected apt report followed by fork report')
    if not apt['goals'] or [g['zone'] for g in apt['goals']] != [g['zone'] for g in fork['goals']]:
        raise ValueError('Reports must contain the same nonempty ordered zone sequence')
    rows = ['| Zone | apt result / s / path m / error m | fork result / s / path m / error m |',
            '| --- | --- | --- |']
    def cell(g):
        values = ['n/a' if g.get(k) is None else f'{g[k]:.3f}'
                  for k in ('seconds', 'path_m', 'final_error_m')]
        return ' / '.join([g['result'], *values])
    for a, f in zip(apt['goals'], fork['goals']):
        rows.append(f"| {a['zone']} | {cell(a)} | {cell(f)} |")
    return '\n'.join(rows)
