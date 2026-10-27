"""ROS-free CSV/JSONL adapter and eval_report command."""
import argparse
import csv
import json
from pathlib import Path

from .eval_repro import json_text, rounded
from .eval_metrics import (clearance_metrics, finite, follow_metrics, load_store_boxes,
                           navigation_metrics, state_metrics, stop_metrics)


def load_log(path):
    """Load sparse rows; timestamps must be globally nondecreasing."""
    path = Path(path)
    with path.open(encoding='utf-8', newline='') as stream:
        if path.suffix.lower() == '.csv':
            reader = csv.DictReader(stream)
            if not reader.fieldnames or not {'type', 't'} <= set(reader.fieldnames):
                raise ValueError('CSV requires type and t columns')
            rows = list(reader)
        elif path.suffix.lower() == '.jsonl':
            rows = [json.loads(line) for line in stream if line.strip()]
        else:
            raise ValueError('Expected .csv or .jsonl input')
    previous = -float('inf')
    for row in rows:
        if not isinstance(row, dict) or 't' not in row:
            raise ValueError('Each log row must be an object with t')
        row['t'] = finite(row['t'])
        if row['t'] < previous:
            raise ValueError('Log timestamps must be nondecreasing')
        previous = row['t']
    return rows


def evaluate_log(rows, boxes, *, target=1.0, band=0.2, too_close=0.6,
                 length=0.6, width=0.5, person_radius=0.25,
                 linear_threshold=0.02, angular_threshold=0.05):
    """Split typed log rows into independent validated metric streams."""
    streams = {name: [] for name in ('pose', 'distance', 'state', 'cmd', 'odom', 'trigger')}
    episodes = {}
    previous = -float('inf')
    for row in rows:
        t = finite(row['t'])
        if t < previous:
            raise ValueError('Log timestamps must be nondecreasing')
        previous = t
        kind = row.get('type')
        if kind == 'navigation':
            identity = row.get('episode')
            if not isinstance(identity, str) or not identity.strip():
                raise ValueError('Navigation rows require an episode string')
            raw_success = row.get('success')
            if isinstance(raw_success, bool):
                success = raw_success
            elif isinstance(raw_success, str) and raw_success in ('true', 'false'):
                success = raw_success == 'true'
            else:
                raise ValueError('Navigation success must be true or false')
            goal = tuple(finite(row[k]) for k in ('goal_x', 'goal_y', 'goal_yaw'))
            episode = episodes.setdefault(identity, dict(id=identity, success=success,
                                                        goal=goal, samples=[]))
            if (episode['success'], episode['goal']) != (success, goal):
                raise ValueError('Episode goal and outcome must remain constant')
            episode['samples'].append(row)
        elif kind in streams:
            cleaned = {k: v for k, v in row.items() if v is not None and v != ''}
            if kind == 'trigger' and not isinstance(cleaned.get('event'), str):
                raise ValueError('Trigger requires an event string')
            streams[kind].append(cleaned)
        else:
            raise ValueError(f'Unknown record type: {kind}')
    parameters = dict(target=target, band=band, too_close=too_close, length=length,
                      width=width, person_radius=person_radius,
                      linear_threshold=linear_threshold, angular_threshold=angular_threshold)
    # Metric functions validate even parameters unused by an empty stream.
    result = dict(
        navigation=navigation_metrics(episodes.values()),
        clearance=clearance_metrics(streams['pose'], boxes, length, width, person_radius),
        follow=follow_metrics(streams['distance'], target, band, too_close),
        states=state_metrics(streams['state']),
        stop=stop_metrics(streams['trigger'], streams['cmd'], streams['odom'],
                          linear_threshold, angular_threshold))
    result['parameters'] = {key: finite(value) for key, value in parameters.items()}
    return result


def markdown_report(report):
    """Render all results, including pending events, in a compact Markdown table."""
    lines = ['# 模擬離線評估報告', '',
             '定義、單位與建議門檻見 docs/design/METRICS.md；缺資料以「無資料」呈現。', '',
             '| 指標／欄位 | 值 |', '| --- | --- |']

    def emit(value, key):
        if isinstance(value, dict):
            for name, child in value.items():
                emit(child, f'{key}.{name}' if key else name)
        elif isinstance(value, list):
            if not value:
                lines.append(f'| {key} | 無資料 |')
            for index, child in enumerate(value):
                emit(child, f'{key}[{index}]')
        else:
            display = '無資料' if value is None else str(value)
            display = display.replace('|', r'\|').replace('\n', ' ').replace('\r', ' ')
            lines.append(f'| {key} | {display} |')

    emit(report, '')
    return '\n'.join(lines)+'\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description='Evaluate simulation CSV/JSONL without ROS')
    parser.add_argument('log', type=Path)
    parser.add_argument('--world', type=Path, required=True, help='Source supermarket.sdf path')
    parser.add_argument('--output', type=Path, required=True, help='Output prefix (.json and .md)')
    for name, default in [('target', 1.0), ('band', 0.2), ('too-close', 0.6),
                          ('length', 0.6), ('width', 0.5), ('person-radius', 0.25),
                          ('linear-threshold', 0.02), ('angular-threshold', 0.05)]:
        parser.add_argument('--'+name, type=float, default=default)
    args = parser.parse_args(argv)
    try:
        report = evaluate_log(load_log(args.log), load_store_boxes(args.world),
                              **{name: rounded(getattr(args, name)) for name in
                                 ('target', 'band', 'too_close', 'length', 'width',
                                  'person_radius', 'linear_threshold', 'angular_threshold')})
        report['source'] = dict(log=str(args.log), world=str(args.world))
        report = rounded(report)
        payload = json_text(report)
        output = str(args.output)
        Path(output+'.json').write_text(payload, encoding='utf-8')
        Path(output+'.md').write_text(markdown_report(report), encoding='utf-8')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
