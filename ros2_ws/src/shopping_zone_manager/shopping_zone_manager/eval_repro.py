"""ROS-free canonical reports and metric comparisons."""
import argparse
import json
import math
import re


def bag_name(value):
    if not re.fullmatch(r'[A-Za-z0-9_-]+', value):
        raise argparse.ArgumentTypeError('Use a simple bag name')
    return value


def rounded(value):
    """Round floats to nine decimal places and reject non-finite numbers."""
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError('Non-finite report number')
        return round(value, 9) or 0.0
    if isinstance(value, dict):
        return {key: rounded(child) for key, child in sorted(value.items())}
    if isinstance(value, list):
        return [rounded(child) for child in value]
    return value


def json_text(value):
    return json.dumps(rounded(value), ensure_ascii=False, sort_keys=True,
                      indent=2, allow_nan=False)+'\n'


def canonical_report(report):
    """Exclude provenance paths and explicitly separate generation time from hashing."""
    return json_text({key: value for key, value in report.items()
                      if key not in ('source', 'generated_at_utc')})


def flatten(value, prefix=''):
    if isinstance(value, (dict, list)) and not value:
        yield prefix, value
    elif isinstance(value, dict):
        for key, child in sorted(value.items()):
            yield from flatten(child, f'{prefix}.{key}' if prefix else key)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from flatten(child, f'{prefix}[{index}]')
    else:
        yield prefix, value


def changed_keys(before, after):
    before = dict(flatten(json.loads(canonical_report(before))))
    after = dict(flatten(json.loads(canonical_report(after))))
    missing = object()
    return [key for key in sorted(before.keys() | after.keys())
            if before.get(key, missing) != after.get(key, missing)
            or type(before.get(key, missing)) is not type(after.get(key, missing))]
