"""ROS-free boundary execution and repository integration checks (not DDS acceptance)."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import xml.etree.ElementTree as ET

from shopping_mcu_bridge.contract import load_contract
from shopping_mcu_bridge.logic import Firmware, LinkMonitor

PKG = Path(__file__).resolve().parents[1]
ROOT = PKG.parents[2]
SPEC = load_contract(PKG/'config/interface.yaml')


def load_class(filename, name, env):
    tree = ast.parse((PKG/'shopping_mcu_bridge'/filename).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), filename, 'exec'), env)
    return env[name]


def test_all_contract_topics_are_used_at_ros_boundary_and_monitor_opt_in():
    used_pub, used_sub = set(), set()
    clock = NS(now=0.0)

    class FakeNode:
        def __init__(self, name):
            pass

        def declare_parameter(self, name, default):
            return NS(value=default)

    def pub(node, spec):
        used_pub.add(spec['name'])
        return Mock()

    def sub(node, spec, callback):
        used_sub.add(spec['name'])
        return Mock()

    env = dict(Node=FakeNode, contract=lambda: SPEC, publisher=pub, subscribe=sub,
               message_type=lambda _: lambda **kw: NS(**kw), steady_timer=Mock(),
               LinkMonitor=LinkMonitor, time=NS(monotonic=lambda: clock.now))
    cls = load_class('monitor_node.py', 'MonitorNode', env)
    node = cls()
    clock.now = 1
    node.tick()
    node.mode.publish.assert_not_called()
    assert node.hb.publish.call_args.args[0].data == 0
    node.stop_enabled = True
    node.tick()
    assert node.mode.publish.call_args.args[0].data == 'estop'
    node.mode.publish.reset_mock()
    node.estop(NS(data=True))
    assert node.mode.publish.call_args.args[0].data == 'estop'

    env.update(Path=Path, get_package_share_directory=lambda pkg: str(PKG.parent/pkg),
               Codec=Mock(), Firmware=Firmware, FirmwareWire=Mock(), geometry=lambda _: (0.1, 0.92))
    cls = load_class('nodes.py', 'FirmwareNode', env)
    cls()
    assert used_pub | used_sub == {s['name'] for s in SPEC['topics'].values()}
    assert '/cmd_vel' not in used_pub
    assert '/cmd_vel' in used_sub


def test_contract_is_standard_bounded_and_documented():
    assert len({s['name'] for s in SPEC['topics'].values()}) == len(SPEC['topics'])
    ids = [s['wire_id'] for s in SPEC['topics'].values() if 'wire_id' in s]
    assert len(ids) == len(set(ids))
    doc = (ROOT/'docs/MCU_INTERFACE.md').read_text()
    for key, spec in SPEC['topics'].items():
        assert spec['type'].split('/')[0] in ('std_msgs', 'sensor_msgs', 'geometry_msgs')
        assert spec['depth'] > 0
        assert spec['reliability'] in ('best_effort', 'reliable')
        assert f"`{spec['name']}`" in doc
        assert f"`{spec['type']}`" in doc
        assert f"| {spec['rate'] or '事件'} | {spec['reliability']}／{spec['depth']}" in doc
    assert SPEC['topics']['estop']['durability'] == 'transient_local'
    assert SPEC['topics']['cmd']['name'] == '/cmd_vel'


def test_pure_modules_have_no_ros_imports_or_undeclared_dependencies():
    pure = ['logic', 'codec', 'contract', 'transport', 'wire']
    for module in pure:
        tree = ast.parse((PKG/'shopping_mcu_bridge'/f'{module}.py').read_text())
        for item in ast.walk(tree):
            if isinstance(item, (ast.Import, ast.ImportFrom)):
                names = [a.name for a in item.names] if isinstance(item, ast.Import) else [item.module or '']
                assert not any(n.startswith(('rclpy', 'geometry_msgs', 'sensor_msgs', 'std_msgs', 'numpy')) for n in names)
        top_imports = [a.name for n in tree.body if isinstance(n, ast.Import) for a in n.names]
        assert 'serial' not in top_imports
    source = (PKG/'shopping_mcu_bridge/transport.py').read_text()
    assert source.index('if not use_hardware:') < source.index('import serial') < source.index('serial.Serial(')
    assert "self.declare_parameter('use_hardware', False)" in (PKG/'shopping_mcu_bridge/nodes.py').read_text()


def test_entry_points_dependencies_and_opt_in_launches():
    captured = {}
    source = (PKG/'setup.py').read_text().replace('from setuptools import setup', '')
    exec(compile(source, 'setup.py', 'exec'), dict(setup=lambda **kw: captured.update(kw), glob=lambda _: []))
    entries = captured['entry_points']['console_scripts']
    for expected in ('sim_mcu_firmware', 'mcu_link_monitor', 'serial_bridge', 'comm_check'):
        entry = next(e for e in entries if e.startswith(expected+' = '))
        target, func = entry.split(' = ')[1].split(':')
        tree = ast.parse((PKG/Path(*target.split('.'))).with_suffix('.py').read_text())
        assert func in [n.name for n in tree.body if isinstance(n, ast.FunctionDef)]
    xml = ET.parse(PKG/'package.xml')
    assert xml.find('export/build_type').text == 'ament_python'
    for name in ('follow', 'guide'):
        source = (ROOT/f'ros2_ws/src/shopping_robot_bringup/launch/{name}.launch.py').read_text()
        assert "DeclareLaunchArgument('mcu_comm', default_value='false'" in source
        assert "condition=IfCondition(LaunchConfiguration('mcu_comm'))" in source
    launch = (PKG/'launch/comm.launch.py').read_text()
    assert "DeclareLaunchArgument('use_sim_time', default_value='false')" in launch
    assert "'use_hardware': False" in launch
    assert "DeclareLaunchArgument('estop_on_link_loss', default_value='true'" in launch


def test_compose_profile_and_default_services_remain_simulation_only():
    compose = load_contract(ROOT/'docker/docker-compose.yaml')
    default = {k for k, s in compose['services'].items() if not s.get('profiles')}
    assert default == {'compute', 'display'}
    agent = compose['services']['microros_agent']
    assert agent['profiles'] == ['comm']
    assert agent['image'] == 'microros/micro-ros-agent:humble'
    assert agent['command'][:3] == ['udp4', '-p', '8888']
    assert not agent.get('ports') and not agent.get('devices') and not agent.get('privileged')
    for service in compose['services'].values():
        assert 'platform' not in service
    serial = load_contract(ROOT/'docker/docker-compose.serial.yaml')['services']['microros_agent']
    assert serial['command'][0] == 'serial' and 'devices' not in serial
    script = (ROOT/'scripts/dev.sh').read_text()
    for case in ('comm-up', 'comm-down', 'comm-sim', 'comm-check'):
        assert f'  {case})' in script
    assert 'up) "${compose[@]}" up -d compute display' in script
    assert 'standalone:=true' in script


def test_checker_never_publishes_cmd_vel_and_refuses_nonisolated_graphs():
    source = (PKG/'shopping_mcu_bridge/comm_check.py').read_text()
    assert "name='/cmd_vel/teleop'" in source
    assert "owners[0].node_name == 'cmd_vel_watchdog'" in source
    assert 'and len(readers) == 1' in source
    assert 'if not isolated:\n            return' in source
    assert 'if node.isolated:' in source


def test_node_telemetry_mapping_and_serial_errors_hold_link_lost():
    import math

    def message(_):
        return lambda **kw: NS(header=NS(stamp=None, frame_id=''), orientation=NS(w=0),
                               angular_velocity=NS(z=0), orientation_covariance=[0.0]*9,
                               linear_acceleration_covariance=[0.0]*9, **kw)

    env = dict(Node=object, message_type=message, math=math)
    cls = load_class('nodes.py', 'FirmwareNode', env)
    node = cls.__new__(cls)
    node.topics, node.spec = SPEC['topics'], SPEC
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: 'ros_stamp'))
    node.publishers_by_key = {k: Mock() for k, s in SPEC['topics'].items() if s['direction'] == 'mcu_to_pc'}
    node.publish_value('wheel', tuple(float(i) for i in range(8)))
    msg = node.publishers_by_key['wheel'].publish.call_args.args[0]
    assert msg.name == SPEC['wheel_names'] and msg.position == [0., 1., 2., 3.]
    assert msg.velocity == [4., 5., 6., 7.] and msg.header.frame_id == 'base_link'
    assert msg.header.stamp == 'ros_stamp'
    node.publish_value('imu', (0.3,))
    msg = node.publishers_by_key['imu'].publish.call_args.args[0]
    assert msg.header.frame_id == 'imu_link' and msg.angular_velocity.z == 0.3
    assert msg.orientation_covariance[0] == msg.linear_acceleration_covariance[0] == -1
    node.publishers_by_key['imu'].publish.reset_mock()
    node.publish_value('imu', (math.nan,))
    node.publishers_by_key['imu'].publish.assert_not_called()
    node.publish_value('ticks', (1, -2, 3, -4))
    assert node.publishers_by_key['ticks'].publish.call_args.args[0].data == [1, -2, 3, -4]
    node.publish_value('status', (255,))
    node.publishers_by_key['status'].publish.assert_not_called()
    node.firmware = Firmware(0.1, 0.92, SPEC['timing'], SPEC['model'])
    node.firmware.velocities = [2.0]*4
    node.failed = False
    node.get_logger = Mock()
    endpoint = Mock()
    endpoint.write.side_effect = OSError('disconnected')
    node.write(endpoint, b'test')
    assert node.failed and node.firmware.velocities == [0]*4
    assert node.publishers_by_key['status'].publish.call_args.args[0].data == 'LINK_LOST'


def test_comm_include_is_after_use_sim_time_declaration():
    for name in ('follow', 'guide'):
        source = (ROOT/f'ros2_ws/src/shopping_robot_bringup/launch/{name}.launch.py').read_text()
        assert source.index("DeclareLaunchArgument('use_sim_time'") < source.index("get_package_share_directory('shopping_mcu_bridge')")
