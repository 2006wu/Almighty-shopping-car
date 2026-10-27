"""Check dynamic linkage of the mixed apt/fork underlay inside compute."""
from pathlib import Path
import subprocess
from check_nav2_contract import check_planner_install

# Newer apt Humble Nav2 ships Route Server BT nodes built against Route messages that the
# 1.1.16 fork's nav2_msgs does not define. They are only loaded when listed in a BT
# navigator plugin list, which the fork parameters must never do.
UNUSED_APT = {'libnav2_compute_route_bt_node.so', 'libnav2_compute_and_track_route_bt_node.so'}
FORK_PARAMS = Path('/ws/src/shopping_robot_navigation/config/nav2_params_fork.yaml')


def apt_targets(apt, fork_libs, params_text):
    """Apt libraries that can be loaded next to the fork, with the reasons others are skipped."""
    targets, skipped = set(), []
    for path in apt.glob('libnav2*.so'):
        if path.name in fork_libs:
            skipped.append(f'{path.name}: shadowed by the fork install')
        elif path.name in UNUSED_APT:
            plugin = path.name[3:-3]
            if plugin in params_text:
                targets.add(path)  # Referenced: must link cleanly.
            else:
                skipped.append(f'{path.name}: Route Server BT node not referenced by fork params')
        else:
            targets.add(path)
    targets.update(p for p in (apt/'nav2_amcl').glob('*') if p.name not in fork_libs)
    return targets, skipped


def main():
    apt = Path('/opt/ros/humble/lib')
    fork = set(Path('/nav2_ws/install').glob('*/lib/*.so'))
    fork.update(Path('/nav2_ws/install/lib').glob('*.so'))
    params_text = FORK_PARAMS.read_text()
    check_planner_install(params_text, Path('/nav2_ws/install'))
    targets, skipped = apt_targets(apt, {p.name for p in fork}, params_text)
    targets |= fork
    failures = []
    for path in sorted(targets):
        if not path.is_file():
            continue
        result = subprocess.run(['ldd', '-r', str(path)], capture_output=True, text=True)
        output = result.stdout + result.stderr
        if result.returncode or 'not found' in output or 'undefined symbol:' in output:
            failures.append(f'{path}: {output}')
    for reason in skipped:
        print(f'SKIP {reason}')
    if failures or not targets:
        raise RuntimeError('Nav2 ABI/link check failed:\n' + '\n'.join(failures))
    print(f'PASS: {len(targets)} fork/apt dynamic linkage checks; runtime ABI still needs simulation')


if __name__ == '__main__':
    main()
