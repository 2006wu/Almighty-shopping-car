import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]


def test_navigation_entry_points_and_commands():
    package = Path(__file__).resolve().parents[1]
    tree = ast.parse((package / 'setup.py').read_text())
    setup = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name) and n.func.id == 'setup')
    entries = ast.literal_eval(next(k.value for k in setup.keywords if k.arg == 'entry_points'))
    scripts = entries['console_scripts']
    assert 'cancel_zone = shopping_zone_manager.cli:cancel_main' in scripts
    assert 'nav_action_check = shopping_zone_manager.nav_action_check:main' in scripts
    dev_path = ROOT / 'scripts/dev.sh'
    if not dev_path.exists():  # colcon test inside the container only mounts ros2_ws/src
        return
    dev = dev_path.read_text()
    assert '  cancel)' in dev and 'shopping_zone_manager cancel_zone' in dev
    assert '  nav-check)' in dev and 'shopping_zone_manager nav_action_check' in dev


def test_pure_modules_and_tests_do_not_import_ros():
    package = Path(__file__).resolve().parents[1]
    paths = [package / 'shopping_zone_manager' / name for name in ('goal_tracker.py', 'zones.py', 'nav_check_logic.py')]
    paths += list((package / 'test').glob('test_*.py'))
    for path in paths:
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Import):
                assert all(not alias.name.startswith('rclpy') for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                assert not (node.module or '').startswith('rclpy')
