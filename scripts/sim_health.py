#!/usr/bin/env python3
"""Check local Gazebo transport readiness without starting a ROS participant."""
import subprocess
import sys

try:
    result = subprocess.run(['ign', 'topic', '-l'], capture_output=True, text=True,
                            timeout=3, check=True)
    sys.exit(0 if '/world/supermarket/clock' in result.stdout.splitlines() else 1)
except (OSError, subprocess.SubprocessError):
    sys.exit(1)
