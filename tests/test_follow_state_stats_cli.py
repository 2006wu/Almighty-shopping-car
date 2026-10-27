"""Check the ROS-free state statistics command with real echo-style text."""
import json
from pathlib import Path
import subprocess
import sys


def test_state_statistics_cli(tmp_path):
    script = Path(__file__).resolve().parents[1]/'scripts/eval/follow_state_stats.py'
    log = tmp_path/'states.log'
    log.write_text('[10.0] data: FOLLOWING: tracking\n---\n[12.0] data: BLOCKED_WAIT: holding\n')
    result = subprocess.run([sys.executable, str(script), str(log)], capture_output=True,
                            text=True, check=True)
    stats = json.loads(result.stdout)
    assert stats['transitions'] == 1 and stats['transitions_per_minute'] == 30
    assert stats['dwell']['FOLLOWING']['total'] == 2
    log.write_text('[10.0] data: FOLLOWING\ndata: LOST\n')
    result = subprocess.run([sys.executable, str(script), str(log)], capture_output=True, text=True)
    assert result.returncode != 0 and 'mix' in result.stderr
