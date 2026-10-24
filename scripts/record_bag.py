#!/usr/bin/env python3
"""Run the shared topic manifest through the Humble recorder in compute."""
import argparse
import math
from pathlib import Path
import signal
import subprocess
import sys

import yaml


def main(argv=None):
    parser = argparse.ArgumentParser(description='Record simulation topics (wall-time duration)')
    parser.add_argument('name')
    parser.add_argument('--duration', type=float)
    parser.add_argument('topics', nargs='*')
    args = parser.parse_intermixed_args(argv)
    if not args.name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.name):
        parser.error('Use a simple bag name')
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('Duration must be positive and finite')
    if any(not topic.startswith('/') or topic.startswith('/-') for topic in args.topics):
        parser.error('Extra topics must be absolute ROS topic names')
    root = Path(__file__).resolve().parents[1]
    manifest = root/'ros2_ws/src/shopping_zone_manager/config/record_topics.yaml'
    topics = yaml.safe_load(manifest.read_text())['topics']
    output = root/'bags'/args.name
    if output.exists():
        parser.error('Bag already exists; choose a new name')
    output.parent.mkdir(exist_ok=True)
    process = subprocess.Popen(['ros2', 'bag', 'record', '--storage', 'sqlite3',
                                '--use-sim-time', '--output', str(output),
                                *dict.fromkeys(topics + args.topics)])
    try:
        return process.wait(timeout=args.duration)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        # SIGINT lets rosbag flush sqlite and write metadata.yaml.
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        result = process.wait()
        return 0 if result in (0, -signal.SIGINT, 130) else result


if __name__ == '__main__':
    sys.exit(main())
