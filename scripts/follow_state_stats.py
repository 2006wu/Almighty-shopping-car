#!/usr/bin/env python3
"""Summarize follow state echo logs without ROS or rosbag."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/shopping_uwb_follow'))
from shopping_uwb_follow.follow_metrics import parse_state_log, state_statistics


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', type=Path)
    parser.add_argument('--sample-period', type=float, default=0.05)
    args = parser.parse_args(argv)
    try:
        samples = parse_state_log(args.logfile.read_text().splitlines(), args.sample_period)
        result = state_statistics(samples)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    result['timing'] = 'timestamps or assumed sample period; boundary dwells are censored'
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
