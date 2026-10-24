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
        if 'third_party' in path.relative_to(root).parts:
            continue
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


def test_odom_source_is_declared_and_forwarded():
    paths = [SRC/'shopping_robot_gazebo/launch/sim.launch.py',
             SRC/'shopping_robot_description/launch/display.launch.py',
             *(SRC/f'shopping_robot_bringup/launch/{name}.launch.py'
               for name in ('mapping', 'guide', 'follow'))]
    for path in paths:
        declaration = next(c for c in calls(path, 'DeclareLaunchArgument')
                           if ast.literal_eval(c.args[0]) == 'odom_source')
        assert ast.literal_eval(keywords(declaration)['default_value']) == 'ekf'
        assert ast.literal_eval(keywords(declaration)['choices']) == ['ekf', 'ground_truth']
        assert "LaunchConfiguration('odom_source')" in path.read_text()
    for path in paths[2:]:
        assert "'odom_source': LaunchConfiguration('odom_source')" in path.read_text()
    assert "'odom_source': LaunchConfiguration('odom_source')" in paths[0].read_text()
    assert "' odom_source:='" in paths[1].read_text()


def test_ekf_velocity_selection_and_frames():
    import yaml
    p = yaml.safe_load((SRC/'shopping_robot_gazebo/config/ekf.yaml').read_text())['ekf_filter_node']['ros__parameters']
    assert p['two_d_mode'] is True and p['publish_tf'] is True and p['use_sim_time'] is True
    assert 30 <= p['frequency'] <= 50
    assert p['world_frame'] == p['odom_frame'] == 'odom'
    assert p['base_link_frame'] == 'base_footprint'
    assert p['odom0'] == '/wheel_odom_noisy'
    assert p['imu0'] == '/imu/with_covariance'
    assert len(p['odom0_config']) == len(p['imu0_config']) == 15
    assert [i for i, enabled in enumerate(p['odom0_config']) if enabled] == [6, 11]
    assert [i for i, enabled in enumerate(p['imu0_config']) if enabled] == [11]


def test_tf_ownership_for_both_expanded_modes():
    import xml.etree.ElementTree as ET
    import xacro
    import yaml
    path = SRC/'shopping_robot_gazebo/launch/sim.launch.py'
    def node(executable, filename=None):
        return next(c for c in calls(path, 'Node')
                    if isinstance(keywords(c).get('executable'), ast.Constant)
                    and keywords(c)['executable'].value == executable
                    and (filename is None or filename in ast.unparse(c)))
    for executable, mode, filename in [('ekf_node', 'ekf', None),
                                       ('wheel_odom_noise', 'ekf', None),
                                       ('parameter_bridge', 'ground_truth', 'bridge_ground_truth.yaml'),
                                       ('static_transform_publisher', 'ground_truth', None)]:
        condition = keywords(node(executable, filename))['condition']
        assert condition.func.id == 'LaunchConfigurationEquals'
        assert [ast.literal_eval(a) for a in condition.args] == ['odom_source', mode]
    assert "('odometry/filtered', output('/odom'))" in ast.unparse(node('ekf_node'))
    common = yaml.safe_load((SRC/'shopping_robot_gazebo/config/bridge.yaml').read_text())
    legacy = yaml.safe_load((SRC/'shopping_robot_gazebo/config/bridge_ground_truth.yaml').read_text())
    assert not any(b['ros_topic_name'] in ('/odom', '/tf') for b in common)
    assert {b['ros_topic_name'] for b in legacy} == {'/odom', '/tf'}
    assert all(b['gz_topic_name'] not in ('wheel_tf', '/wheel_tf', '/sim/ground_truth_tf') for b in common+legacy)
    for mode in ('ekf', 'ground_truth'):
        model = ET.fromstring(xacro.process_file(
            str(SRC/'shopping_robot_description/urdf/shopping_robot.urdf.xacro'),
            mappings={'odom_source': mode}).toxml())
        publishers = [p for p in model.findall('.//plugin') if p.get('name').endswith('::OdometryPublisher')]
        truth, = [p for p in publishers if p.findtext('odom_topic') == '/sim/ground_truth_odom']
        assert truth.findtext('odom_frame') == 'sim_world'
        assert truth.findtext('robot_base_frame') == 'base_footprint_gt'
        assert truth.findtext('tf_topic') == '/sim/ground_truth_tf'
        legacy_publishers = [p for p in publishers if p.findtext('tf_topic') == 'tf']
        # Only EKF in ekf mode; only the legacy simulator plugin otherwise.
        assert len(legacy_publishers) + (mode == 'ekf') == 1
        if legacy_publishers:
            assert legacy_publishers[0].findtext('odom_frame') == 'odom'
            assert legacy_publishers[0].findtext('robot_base_frame') == 'base_footprint'
        drive, = [p for p in model.findall('.//plugin') if p.get('name').endswith('::DiffDrive')]
        assert drive.findtext('tf_topic') == 'wheel_tf'
        for group in ('angular_velocity', 'linear_acceleration'):
            for axis in ('x', 'y', 'z'):
                noise = model.find(f'.//sensor/imu/{group}/{axis}/noise')
                assert noise.get('type') == 'gaussian'
                assert float(noise.findtext('stddev')) > 0
                assert float(noise.findtext('bias_mean')) > 0


def test_truth_consumers_and_navigation_odom():
    evaluation = (SRC/'shopping_uwb_follow/shopping_uwb_follow/uwb_eval_node.py').read_text()
    assert "Odometry, '/sim/ground_truth_odom'" in evaluation
    assert "msg.header.frame_id != 'sim_world'" in evaluation
    simulation = (SRC/'shopping_uwb_follow/shopping_uwb_follow/uwb_sim_node.py').read_text()
    assert "'/simulation/poses'" in simulation
    import yaml
    nav = yaml.safe_load((SRC/'shopping_robot_navigation/config/nav2_params.yaml').read_text())
    assert nav['velocity_smoother']['ros__parameters']['odom_topic'] == '/odom'
    slam = yaml.safe_load((SRC/'shopping_robot_slam/config/mapper_params_online_async.yaml').read_text())
    assert slam['slam_toolbox']['ros__parameters']['odom_frame'] == 'odom'
