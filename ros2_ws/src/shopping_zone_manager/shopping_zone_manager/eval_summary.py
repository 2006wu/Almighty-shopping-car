"""ROS-free multi-run statistics and provenance warnings."""
import argparse
import json
import math
from pathlib import Path
import statistics

from .eval_repro import bag_name, flatten, json_text, rounded
from .eval_report import markdown_report


def aggregate(runs):
    """Aggregate numeric leaves by field path; booleans are never numeric metrics."""
    metrics, verdicts, warnings = {}, {}, []
    provenance = {key: set() for key in ('commit', 'scenario', 'nav2_stack')}
    for name, report, manifest in runs:
        if not isinstance(report, dict) or (manifest is not None and not isinstance(manifest, dict)):
            raise ValueError(f'{name}: report and manifest must be objects')
        for key, value in flatten(report):
            if key.startswith(('parameters.', 'source.')):
                continue
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if not math.isfinite(value):
                    raise ValueError(f'Non-finite metric: {name}: {key}')
                metrics.setdefault(key, []).append(value)
            if key.split('.')[-1] == 'verdict' or key.startswith('verdicts.'):
                verdict = {True: 'pass', False: 'fail'}.get(value) if isinstance(value, bool) else value
                if isinstance(verdict, str) and verdict.lower() in ('pass', 'fail'):
                    counts = verdicts.setdefault(key, dict(pass_count=0, fail_count=0))
                    counts[verdict.lower()+'_count'] += 1
        if manifest is None:
            warnings.append(f'{name}: 缺少 manifest.json')
        else:
            values = dict(commit=manifest.get('git', {}).get('commit'),
                          **{key: manifest.get('metadata', {}).get(key)
                             for key in ('scenario', 'nav2_stack')})
            for key, value in values.items():
                if value is None or value == 'unknown':
                    warnings.append(f'{name}: 缺少 {key}')
                provenance[key].add(json.dumps(value, sort_keys=True))
    for key, values in provenance.items():
        if len(values) > 1:
            warnings.append(f'各次紀錄的 {key} 不一致')
    result = {}
    for key, values in sorted(metrics.items()):
        values = sorted(values)
        result[key] = dict(count=len(values), mean=statistics.mean(values), min=min(values),
                           max=max(values), stdev=statistics.stdev(values) if len(values) > 1 else 0.0)
    return rounded(dict(runs=sorted(name for name, _, _ in runs), metrics=result,
                        verdicts=verdicts, warnings=sorted(warnings)))


def main(argv=None):
    parser = argparse.ArgumentParser(description='Summarize recorded simulation evaluations')
    parser.add_argument('names', nargs='+', type=bag_name)
    parser.add_argument('--bags-dir', type=Path, default=Path('/repo/bags'))
    parser.add_argument('--output', type=Path, default=Path('/repo/bags/summary'))
    args = parser.parse_args(argv)
    if len(set(args.names)) != len(args.names):
        parser.error('Duplicate bag names')
    try:
        runs = []
        for name in args.names:
            directory = args.bags_dir/name
            manifest = directory/'manifest.json'
            runs.append((name, json.loads((directory/'report.json').read_text(encoding='utf-8')),
                         json.loads(manifest.read_text(encoding='utf-8')) if manifest.exists() else None))
        result = aggregate(runs)
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output/'summary.json').write_text(json_text(result), encoding='utf-8')
        (args.output/'summary.md').write_text(markdown_report(result), encoding='utf-8')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
