"""Reject a fork checkout that cannot honor shopping differential-drive settings."""
from pathlib import Path
import argparse

FORK_PARAMS = Path('/ws/src/shopping_robot_navigation/config/nav2_params_fork.yaml')


def check_planner_install(params_text, install):
    """Reject apt fallback: Smac must be compiled against the fork core/costmap."""
    if 'nav2_smac_planner/SmacPlanner2D' not in params_text:
        return
    libraries = list(install.glob('*/lib/libnav2_smac_planner_2d.so'))
    libraries += list(install.glob('lib/libnav2_smac_planner_2d.so'))
    if not any(path.is_file() for path in libraries):
        raise RuntimeError(
            'Fork SmacPlanner2D requires libnav2_smac_planner_2d.so in '
            f'{install}; refusing apt plugin fallback (Nav2 ABI mixing). '
            'Build nav2_smac_planner from the fork in /nav2_ws.')


PARAMETERS = ('holonomic', 'avoidance_mode', 'use_rival_topic', 'gentle_slowdown_dist',
              'gentle_min_speed_scale', 'gentle_stop_dist', 'gentle_wait_before_replan',
              'gentle_max_speed_near_people')


def missing_parameters(source):
    return [name for name in PARAMETERS if f'".{name}"' not in source]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-only', action='store_true',
                        help='Pre-build source check; install check runs after the build.')
    args = parser.parse_args()
    source = Path('/nav2_ws/src/custom_controller/src/teb_controller.cpp').read_text()
    missing = missing_parameters(source)
    if missing:
        raise SystemExit('Fork TEB contract 尚未整合 / missing parameters: '
                         + ', '.join(missing) + '. Update the host submodule to the reviewed TEB commit.')
    if not args.source_only:
        check_planner_install(FORK_PARAMS.read_text(), Path('/nav2_ws/install'))


if __name__ == '__main__':
    main()
