"""Check dynamic linkage of the mixed apt/fork underlay inside compute."""
from pathlib import Path
import subprocess


def main():
    apt = Path('/opt/ros/humble/lib')
    targets = set(Path('/nav2_ws/install').glob('*/lib/*.so'))
    targets.update(apt.glob('libnav2*.so'))
    targets.update((apt/'nav2_amcl').glob('*'))
    failures = []
    for path in sorted(targets):
        if not path.is_file():
            continue
        result = subprocess.run(['ldd', '-r', str(path)], capture_output=True, text=True)
        output = result.stdout + result.stderr
        if result.returncode or 'not found' in output or 'undefined symbol:' in output:
            failures.append(f'{path}: {output}')
    if failures or not targets:
        raise RuntimeError('Nav2 ABI/link check failed:\n' + '\n'.join(failures))
    print(f'PASS: {len(targets)} fork/apt dynamic linkage checks; runtime ABI still needs simulation')


if __name__ == '__main__':
    main()
