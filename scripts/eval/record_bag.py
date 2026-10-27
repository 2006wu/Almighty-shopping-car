#!/usr/bin/env python3
"""Run the shared topic manifest through the Humble recorder in compute."""
import argparse
import math
import json
import re
from datetime import datetime, timezone
from pathlib import Path
import signal
import subprocess
import sys

import yaml


def metadata_pair(value):
    key, separator, content = value.partition('=')
    if not separator or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', key):
        raise argparse.ArgumentTypeError('Metadata must be KEY=VALUE with an identifier key')
    return key, content


def git_provenance():
    """Probe the mounted checkout without making recording depend on git."""
    def git(*arguments):
        try:
            result = subprocess.run(['git', '-C', '/repo', *arguments],
                                    capture_output=True, text=True, timeout=3, check=True)
            return result.stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return None

    commit = git('rev-parse', 'HEAD')
    status = git('status', '--porcelain', '--untracked-files=normal')
    submodule = git('-C', '/repo/third_party/eurobot_nav2', 'rev-parse', 'HEAD')
    # An uninitialized submodule directory can otherwise resolve the parent HEAD.
    initialized = git('submodule', 'status', 'third_party/eurobot_nav2')
    if not initialized or initialized.startswith('-'):
        submodule = None
    return dict(commit=commit or 'unknown', dirty=status != '' if status is not None else 'unknown',
                submodule_commit=submodule or 'unknown')


def build_manifest(topics, duration, metadata, start_time, git):
    """Build serializable recording provenance independently of ROS."""
    return dict(start_time_utc=start_time, git=dict(git), topics=list(dict.fromkeys(topics)),
                duration=duration, metadata=dict(metadata))


def main(argv=None):
    parser = argparse.ArgumentParser(description='Record simulation topics (wall-time duration)')
    parser.add_argument('name')
    parser.add_argument('--duration', type=float)
    parser.add_argument('topics', nargs='*')
    parser.add_argument('--meta', action='append', type=metadata_pair, default=[])
    args = parser.parse_intermixed_args(argv)
    if not args.name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.name):
        parser.error('Use a simple bag name')
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('Duration must be positive and finite')
    if any(not topic.startswith('/') or topic.startswith('/-') for topic in args.topics):
        parser.error('Extra topics must be absolute ROS topic names')
    root = Path(__file__).resolve().parents[2]
    manifest = root/'ros2_ws/src/shopping_zone_manager/config/record_topics.yaml'
    topics = yaml.safe_load(manifest.read_text())['topics']
    output = root/'bags'/args.name
    if output.exists():
        parser.error('Bag already exists; choose a new name')
    output.parent.mkdir(exist_ok=True)
    topics = list(dict.fromkeys(topics + args.topics))
    run_manifest = build_manifest(topics, args.duration, args.meta,
                                  datetime.now(timezone.utc).isoformat(), git_provenance())
    process = subprocess.Popen(['ros2', 'bag', 'record', '--storage', 'sqlite3',
                                '--use-sim-time', '--output', str(output),
                                *topics])
    try:
        return process.wait(timeout=args.duration)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        # SIGINT lets rosbag flush sqlite and write metadata.yaml.
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        result = process.wait()
        return 0 if result in (0, -signal.SIGINT, 130) else result
    finally:
        # rosbag creates its own output directory; write only after it has started.
        if output.is_dir():
            (output/'manifest.json').write_text(
                json.dumps(run_manifest, ensure_ascii=False, sort_keys=True, indent=2)+'\n',
                encoding='utf-8')


if __name__ == '__main__':
    sys.exit(main())
