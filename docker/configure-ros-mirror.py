"""Rewrite ROS apt URIs in both deb822 and legacy sources before apt update."""
from pathlib import Path
import sys
from urllib.parse import urlsplit


def configure(mirror, source_dir=Path('/etc/apt/sources.list.d')):
    if not mirror:
        return
    parsed = urlsplit(mirror)
    if parsed.scheme not in ('http', 'https') or not parsed.netloc or any(c.isspace() for c in mirror):
        raise ValueError('ROS_APT_MIRROR must be an HTTP(S) ROS2 Ubuntu repository URL')
    for pattern in ('*.sources', '*.list'):
        for path in source_dir.glob(pattern):
            original = path.read_text()
            changed = original
            for scheme in ('http', 'https'):
                changed = changed.replace(f'{scheme}://packages.ros.org/ros2/ubuntu', mirror.rstrip('/'))
            if changed != original:
                path.write_text(changed)


if __name__ == '__main__':
    configure(sys.argv[1])
