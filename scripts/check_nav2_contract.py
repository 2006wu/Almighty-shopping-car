"""Reject a fork checkout that cannot honor shopping differential-drive settings."""
from pathlib import Path

PARAMETERS = ('holonomic', 'avoidance_mode', 'use_rival_topic', 'gentle_slowdown_dist',
              'gentle_min_speed_scale', 'gentle_stop_dist', 'gentle_wait_before_replan',
              'gentle_max_speed_near_people')


def missing_parameters(source):
    return [name for name in PARAMETERS if f'".{name}"' not in source]


def main():
    source = Path('/nav2_ws/src/custom_controller/src/teb_controller.cpp').read_text()
    missing = missing_parameters(source)
    if missing:
        raise SystemExit('Fork TEB contract 尚未整合 / missing parameters: '
                         + ', '.join(missing) + '. Update the host submodule to the reviewed TEB commit.')


if __name__ == '__main__':
    main()
