"""Reconvert and evaluate a recorded simulation without publishing ROS messages."""
import argparse
import json
from pathlib import Path
from tempfile import TemporaryDirectory

from .eval_repro import bag_name, canonical_report, changed_keys


def verify(bag, convert, evaluate):
    """Use stored conversion settings and metric parameters; leave artifacts untouched."""
    bag = Path(bag)
    stored = json.loads((bag/'report.json').read_text(encoding='utf-8'))
    info = json.loads((bag/'observations.jsonl.meta.json').read_text(encoding='utf-8'))
    with TemporaryDirectory(prefix='eval-verify-') as directory:
        log = Path(directory)/'observations.jsonl'
        arguments = [str(bag), '--output', str(log), '--max-skew', str(info['max_skew'])]
        if info.get('zones'):
            arguments += ['--zones', info['zones']]
        if info.get('map_world_transform') is not None:
            arguments += ['--map-world-transform', *map(str, info['map_world_transform'])]
        convert(arguments)
        output = Path(directory)/'report'
        arguments = [str(log), '--world', stored['source']['world'], '--output', str(output)]
        for key, value in stored['parameters'].items():
            arguments += ['--'+key.replace('_', '-'), str(value)]
        evaluate(arguments)
        fresh = json.loads(output.with_suffix('.json').read_text(encoding='utf-8'))
    return canonical_report(stored) == canonical_report(fresh), changed_keys(stored, fresh)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Verify stored simulation evaluation')
    parser.add_argument('name', type=bag_name)
    parser.add_argument('--bags-dir', type=Path, default=Path('/repo/bags'))
    args = parser.parse_args(argv)
    from .bag_to_eval import main as convert
    from .eval_report import main as evaluate
    try:
        same, keys = verify(args.bags_dir/args.name, convert, evaluate)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
        parser.error(str(exc))
    print('Report matches' if same else 'Changed keys: '+', '.join(keys[:20]))
    if len(keys) > 20:
        print(f'... and {len(keys)-20} more')
    return 0 if same else 1


if __name__ == '__main__':
    raise SystemExit(main())
