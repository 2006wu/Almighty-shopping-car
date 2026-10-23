"""ROS-free planar localization statistics; angles are radians internally."""
import math


def validate_truth_frames(parent, child):
    """Accept only simulator truth or the legacy truth frame pairs."""
    if ((parent == 'sim_world' and child == 'base_footprint_gt')
            or (parent in ('odom', 'world', 'map') and child == 'base_footprint')):
        return
    raise ValueError(
        f'Unsupported truth frames ({parent!r}, {child!r}); expected '
        '(sim_world, base_footprint_gt) or (odom|world|map, base_footprint)')


def pose_error(estimate, truth):
    if len(estimate) != 3 or len(truth) != 3 or not all(math.isfinite(v) for v in (*estimate, *truth)):
        raise ValueError('Expected finite planar poses')
    return (math.hypot(estimate[0]-truth[0], estimate[1]-truth[1]),
            abs(math.atan2(math.sin(estimate[2]-truth[2]), math.cos(estimate[2]-truth[2]))))


def statistics(values):
    if not values or not all(math.isfinite(v) and v >= 0 for v in values):
        raise ValueError('Expected nonempty finite nonnegative errors')
    return {'mean': sum(values)/len(values), 'rmse': math.sqrt(sum(v*v for v in values)/len(values)),
            'max': max(values)}


def evaluate_zone(goal, samples, navigation_success, min_samples=5):
    """Require every stationary sample to pass, including the final truth goal error."""
    errors = [pose_error(estimate, truth) for estimate, truth in samples]
    result = {'zone': goal.name, 'navigation_success': navigation_success, 'samples': len(samples),
              'pass': False}
    if not samples:
        return result
    estimate, truth = samples[-1]
    result.update(localization_m=statistics([e[0] for e in errors]),
                  localization_deg=statistics([math.degrees(e[1]) for e in errors]),
                  final_map_goal_m=math.hypot(estimate[0]-goal.x, estimate[1]-goal.y),
                  final_truth_goal_m=math.hypot(truth[0]-goal.x, truth[1]-goal.y))
    result['pass'] = bool(navigation_success and len(samples) >= min_samples
                          and result['localization_m']['max'] < 0.15
                          and result['localization_deg']['max'] < 5.0
                          and result['final_truth_goal_m'] <= 0.3)
    return result


def aggregate(results, expected_count):
    report = {'zones': results, 'pass': len(results) == expected_count and expected_count > 0
              and all(row['pass'] for row in results)}
    rows = [row for row in results if row['samples'] > 0]
    count = sum(row['samples'] for row in rows)
    if count:
        for metric in ('localization_m', 'localization_deg'):
            report[metric] = {
                'mean': sum(row[metric]['mean']*row['samples'] for row in rows)/count,
                'rmse': math.sqrt(sum(row[metric]['rmse']**2*row['samples'] for row in rows)/count),
                'max': max(row[metric]['max'] for row in rows)}
    return report
