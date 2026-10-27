#!/usr/bin/env python3
"""Run the ROS-free UWB study and export reproducible measurements."""
import argparse
from pathlib import Path
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/shopping_uwb_follow'))
from shopping_uwb_follow.uwb_study import render_csv, render_markdown, run_study


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials', type=int, default=None)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--out-md', type=Path, default=ROOT/'docs/research/UWB_STUDY.md')
    parser.add_argument('--out-csv', type=Path, default=ROOT/'docs/research/uwb_study.csv')
    parser.add_argument('--quick', action='store_true')
    args = parser.parse_args(argv)
    trials = args.trials if args.trials is not None else (3 if args.quick else 40)
    if trials < 1:
        parser.error('--trials must be positive')
    path = yaml.safe_load((ROOT/'ros2_ws/src/shopping_robot_gazebo/config/person_path.yaml').read_text())
    start = time.perf_counter()
    rows = run_study(path, trials, args.seed, args.quick)
    elapsed = time.perf_counter()-start
    for target, content in ((args.out_md, render_markdown(rows, trials, args.seed, elapsed, args.quick)),
                            (args.out_csv, render_csv(rows))):
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content, encoding='utf-8')
    print(f'UWB study runtime: {elapsed:.2f} s; {len(rows)} rows; trials={trials}; seed={args.seed}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
