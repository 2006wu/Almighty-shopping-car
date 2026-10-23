"""Check launch opt-in wiring and pytest discovery metadata without ROS."""
import ast
from pathlib import Path


SRC = Path(__file__).resolve().parents[1]/'ros2_ws/src'


def calls(path, name):
    return [node for node in ast.walk(ast.parse(path.read_text()))
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
            and node.func.id == name]


def keywords(call):
    return {kw.arg: kw.value for kw in call.keywords}


def test_person_is_opt_in_and_only_gates_mover():
    path = SRC/'shopping_robot_gazebo/launch/sim.launch.py'
    declaration = next(c for c in calls(path, 'DeclareLaunchArgument')
                       if ast.literal_eval(c.args[0]) == 'person')
    assert ast.literal_eval(keywords(declaration)['default_value']) == 'false'
    mover = next(c for c in calls(path, 'Node')
                 if isinstance(keywords(c).get('executable'), ast.Constant)
                 and keywords(c)['executable'].value == 'person_path_node')
    condition = keywords(mover)['condition']
    assert condition.func.id == 'IfCondition'
    assert condition.args[0].func.id == 'LaunchConfiguration'
    assert ast.literal_eval(condition.args[0].args[0]) == 'person'


def test_bringup_person_arguments_and_evaluation_node():
    for name in ('mapping', 'guide', 'follow'):
        path = SRC/f'shopping_robot_bringup/launch/{name}.launch.py'
        person_values = [value for node in ast.walk(ast.parse(path.read_text()))
                         if isinstance(node, ast.Dict)
                         for key, value in zip(node.keys, node.values)
                         if isinstance(key, ast.Constant) and key.value == 'person']
        assert len(person_values) == 1
        value = person_values[0]
        if name == 'follow':
            assert ast.literal_eval(value) == 'true'
            assert any(isinstance(n, ast.Constant) and n.value == 'uwb_eval_node'
                       for n in ast.walk(ast.parse(path.read_text())))
        else:
            assert value.func.id == 'LaunchConfiguration'
            assert ast.literal_eval(value.args[0]) == 'person'
            declaration = next(c for c in calls(path, 'DeclareLaunchArgument')
                               if ast.literal_eval(c.args[0]) == 'person')
            assert ast.literal_eval(keywords(declaration)['default_value']) == 'false'


def test_ament_python_packages_enable_pytest():
    for path in SRC.glob('*/setup.py'):
        setup, = calls(path, 'setup')
        assert ast.literal_eval(keywords(setup)['tests_require']) == ['pytest']


def test_pytest_files_do_not_import_rclpy():
    root = SRC.parents[1]
    for path in root.rglob('test_*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            modules = ([alias.name for alias in node.names] if isinstance(node, ast.Import)
                       else [node.module or ''] if isinstance(node, ast.ImportFrom) else [])
            assert all(name.split('.')[0] != 'rclpy' for name in modules), path


def test_follow_navigation_handoff_wiring_and_dependencies():
    import xml.etree.ElementTree as ET
    path = SRC/'shopping_robot_bringup/launch/follow.launch.py'
    text = path.read_text()
    assert "get_package_share_directory('shopping_robot_navigation')" in text
    assert '/launch/navigation.launch.py' in text
    assert 'zone_navigator' not in text
    for name, default in [('slam_mode', 'mapping'), ('map', ''), ('route_around', 'true')]:
        declaration = next(c for c in calls(path, 'DeclareLaunchArgument')
                           if ast.literal_eval(c.args[0]) == name)
        assert ast.literal_eval(keywords(declaration)['default_value']) == default
        assert f"LaunchConfiguration('{name}')" in text
    assert "'route_around': ParameterValue(LaunchConfiguration('route_around'), value_type=bool)" in text
    root = ET.parse(SRC/'shopping_uwb_follow/package.xml').getroot()
    dependencies = {el.text for el in root.findall('exec_depend')}
    assert {'nav2_msgs', 'tf2_ros', 'action_msgs'} <= dependencies
