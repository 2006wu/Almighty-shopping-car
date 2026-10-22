#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${1:-python3}"
if [[ $# -gt 0 ]]; then shift; fi
export PYTHONPATH="$repo_dir/ros2_ws/src/shopping_zone_manager:$repo_dir/ros2_ws/src/shopping_uwb_follow${PYTHONPATH:+:$PYTHONPATH}"
cd "$repo_dir"
exec "$python_bin" -m pytest -q "$@"
