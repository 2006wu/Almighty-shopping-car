"""Offline checks only; Docker/ROS acceptance remains a maintainer task."""
import importlib.util
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace as NS

import pytest
import yaml
from shopping_mcu_bridge.agent_checks import isolated_graph, zero_window, advancing_counter

PKG = Path(__file__).resolve().parents[1]


@pytest.fixture
def repo():
    for root in (PKG.parents[2], Path('/repo')):
        if (root/'docs').is_dir():
            return root
    pytest.skip('Repository files unavailable: neither ancestor root nor /repo has docs/')


def graph():
    nodes = ['micro_ros_sim_client', 'mcu_link_monitor', 'cmd_vel_mux', 'cmd_vel_watchdog']
    pubs = {'/cmd_vel': ['cmd_vel_watchdog'], '/pc/heartbeat': ['mcu_link_monitor']}
    pubs.update({t: ['micro_ros_sim_client'] for t in
                 ('/mcu/heartbeat', '/mcu/wheel_state', '/mcu/status', '/mcu/sim/xrce_active')})
    return nodes, pubs


def test_graph_rejects_other_firmware_gazebo_duplicate_owners_and_stale_marker():
    nodes, pubs = graph()
    assert isolated_graph(nodes, pubs, ['micro_ros_sim_client'], [], True)
    for foreign in ('sim_mcu_firmware', 'serial_bridge', 'gazebo', 'cmd_vel_mux'):
        assert not isolated_graph(nodes+[foreign], pubs, ['micro_ros_sim_client'], [], True)
    assert not isolated_graph(nodes, pubs, ['micro_ros_sim_client', 'other'], [], True)
    assert not isolated_graph(nodes, pubs, ['micro_ros_sim_client'], ['clock'], True)
    assert not isolated_graph(nodes, pubs, ['micro_ros_sim_client'], [], False)
    for topic in pubs:
        modified = dict(pubs, **{topic: pubs[topic]+['other']})
        assert not isolated_graph(nodes, modified, ['micro_ros_sim_client'], [], True)


def test_zero_window_requires_fresh_finite_complete_samples_and_counter_advances():
    samples = [(i/10, NS(velocity=[0.0]*4)) for i in range(10)]
    assert zero_window(samples, 0.5)
    assert not zero_window(samples, 0.6)
    for bad in ([0.0]*3, [float('nan')]*4, [0.1]*4):
        assert not zero_window(samples+[(1, NS(velocity=bad))], 0.5)
    wrapped = [(i, NS(data=(0xfffffffc+i) & 0xffffffff)) for i in range(10)]
    assert advancing_counter(wrapped)
    assert not advancing_counter(wrapped+[(11, wrapped[-1][1])])
    assert not advancing_counter(list(reversed(wrapped)))


def test_generated_constants_match_contract_and_geometry(repo):
    folder = repo/'firmware/sim_client'
    spec = importlib.util.spec_from_file_location('generate_contract', folder/'generate_contract.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert (folder/'contract.h').read_text() == module.render(
        PKG/'config/interface.yaml', repo/'ros2_ws/src/shopping_robot_description/urdf/shopping_robot.urdf.xacro')
    contract = yaml.safe_load((PKG/'config/interface.yaml').read_text())
    for key in ('cmd', 'mcu_heartbeat', 'pc_heartbeat', 'status'):
        assert contract['topics'][key]['reliability'] == 'reliable'
        assert contract['topics'][key]['depth'] == 1
    assert contract['topics']['wheel']['reliability'] == 'best_effort'
    assert contract['topics']['status']['rate'] == contract['topics']['mcu_heartbeat']['rate']


def test_actual_c_model_safety_and_motion(repo, tmp_path):
    compiler = shutil.which('cc')
    if not compiler:
        pytest.skip('C compiler unavailable; run this test in the compute container')
    folder = repo/'firmware/sim_client'
    harness = tmp_path/'test.c'
    harness.write_text(r'''
#include <assert.h>
#include <math.h>
#include <string.h>
#include "model.h"
static void state(Model *m, double t, const char *s) {
  model_step(m, t); assert(!strcmp(m->status, s));
}
static void zero(Model *m) {
  for (int i=0; i<4; ++i) assert(m->velocity[i] == 0);
}
int main(void) {
  Model m; model_init(&m);
  double c[6] = {0.2,0,0,0,0,0};
  state(&m, 0, "LINK_LOST"); zero(&m);
  model_command(&m,c,0); model_heartbeat(&m,0xfffffffe,0);
  model_heartbeat(&m,0xffffffff,0.1); model_heartbeat(&m,0,0.2);
  state(&m,0.2,"CMD_TIMEOUT");
  model_command(&m,c,0.21); state(&m,0.22,"READY");
  assert(m.velocity[0]>0 && m.velocity[0]<=12*0.02+1e-10);
  assert(m.velocity[0]==m.velocity[3] && m.position[0]>0);
  model_heartbeat(&m,1,0.4);
  state(&m,0.51,"CMD_TIMEOUT"); zero(&m);
  model_command(&m,c,0.52); state(&m,0.53,"READY");
  model_heartbeat(&m,1,0.6); model_heartbeat(&m,0,0.7);
  state(&m,0.9,"LINK_LOST"); zero(&m);
  model_heartbeat(&m,5,1); model_heartbeat(&m,6,1.1); model_heartbeat(&m,7,1.2);
  state(&m,1.2,"CMD_TIMEOUT"); zero(&m);
  c[0]=NAN; model_command(&m,c,1.21); state(&m,1.22,"INVALID_COMMAND"); zero(&m);
  c[0]=0.2; c[1]=0.01; model_command(&m,c,1.23); state(&m,1.24,"INVALID_COMMAND");
  c[1]=0; c[0]=-1; model_command(&m,c,1.25); state(&m,1.26,"READY"); zero(&m);
  c[0]=100; c[5]=-100; model_command(&m,c,1.27);
  assert(m.linear==MCU_MAX_LINEAR && m.angular==-MCU_MAX_ANGULAR);
  m.drop_commands_until=2; model_command(&m,c,1.5);
  assert(m.command_time==1.27);
  state(&m,1.7,"LINK_LOST"); zero(&m);
  state(&m,0.1,"LINK_LOST"); zero(&m);
  return 0;
}
''')
    executable = tmp_path/'model-test'
    subprocess.run([compiler, '-std=c99', '-Wall', '-Wextra', '-Werror', '-I', str(folder),
                    str(harness), str(folder/'model.c'), '-lm', '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


def test_compose_build_and_launch_wiring(repo):
    services = yaml.safe_load((repo/'docker/docker-compose.yaml').read_text())['services']
    client = services['microros_client']
    assert client['profiles'] == ['comm']
    assert client['depends_on'] == ['microros_agent']
    assert client['environment']['MICROROS_AGENT_HOST'] == 'microros_agent'
    assert client['environment']['MICROROS_AGENT_PORT'] == '8888'
    assert client['environment']['RMW_IMPLEMENTATION'] == 'rmw_microxrcedds'
    assert client['networks'] == services['microros_agent']['networks']
    for forbidden in ('ports', 'devices', 'privileged', 'platform', 'volumes'):
        assert forbidden not in client
    dockerfile = (repo/client['build']['dockerfile']).read_text()
    for expected in ('FROM ros:humble-ros-base-jammy', '--branch humble',
                     'create_firmware_ws.sh host', 'build_firmware.sh', 'generate_contract.py',
                     'COPY firmware/sim_client/ src/uros/shopping_microros_client/'):
        assert expected in dockerfile
    launch = (PKG/'launch/comm.launch.py').read_text()
    assert "choices=['sim_ros', 'serial_loopback', 'agent']" in launch
    assert "transport, \"' == 'sim_ros'\"" in launch
    assert "transport, \"' == 'serial_loopback'\"" in launch
    assert "'allow_simulation_faults'" in launch
    source = (PKG/'shopping_mcu_bridge/agent_check.py').read_text()
    assert '+10.0' in source and 'self.graph_safe()' in source
    assert "node.send('mode_request', 'estop')" in source
    assert 'node.pause_pc(False)' in source


def test_dev_commands_dispatch_without_running_docker(repo, tmp_path):
    fake = tmp_path/'fake-docker'
    log = tmp_path/'calls'
    fake.write_text('#!/bin/bash\nprintf "%s\\n" "$*" >> "$FAKE_LOG"\n')
    fake.chmod(0o755)
    import os
    env = dict(os.environ, DOCKER=str(fake), FAKE_LOG=str(log))
    for command in ('comm-e2e-up', 'comm-e2e-check', 'comm-down'):
        subprocess.run(['bash', str(repo/'scripts/dev.sh'), command], env=env, check=True)
    calls = log.read_text().splitlines()
    assert len(calls) == 4
    assert '--profile comm up -d --build microros_agent microros_client' in calls[0]
    assert 'comm.launch.py standalone:=true transport:=agent use_sim_time:=false' in calls[1]
    assert 'ros2 run shopping_mcu_bridge agent_check' in calls[2]
    assert 'stop microros_client microros_agent' in calls[3]


def test_monitor_fault_is_opt_in_and_does_not_disable_stop_requests():
    import ast
    from unittest.mock import Mock
    source = (PKG/'shopping_mcu_bridge/monitor_node.py').read_text()
    tree = ast.parse(source)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    env = dict(Node=object, message_type=lambda _: lambda **kw: NS(**kw))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), 'monitor_node.py', 'exec'), env)
    node = env['MonitorNode'].__new__(env['MonitorNode'])
    node.allow_faults = False
    node.topics = {'pc_heartbeat': {}}
    node.counter = 0
    node.hb = Mock()
    node.get_parameter = Mock(return_value=NS(value=True))
    node.logic = NS(should_stop=lambda _: True)
    node.request_stop = Mock()
    env['time'] = NS(monotonic=lambda: 0)
    node.tick()
    assert node.hb.publish.call_count == 1 and node.counter == 1
    node.get_parameter.assert_not_called()
    node.allow_faults = True
    node.tick()
    assert node.hb.publish.call_count == 1 and node.counter == 1
    assert node.request_stop.call_count == 2
    node.get_parameter.return_value.value = False
    node.tick()
    assert node.hb.publish.call_count == 2 and node.counter == 2


@pytest.mark.parametrize('failure', [False, True])
def test_checker_cleanup_stops_before_restoring_heartbeat(failure):
    import ast
    from unittest.mock import Mock
    tree = ast.parse((PKG/'shopping_mcu_bridge/agent_check.py').read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    calls = []

    def checks():
        if failure:
            raise RuntimeError('injected check failure')
        yield from [('check', True)]*7

    node = NS(checks=checks, isolated=True, pause_attempted=True,
              send=lambda *args: calls.append(('send', args)),
              pause_pc=lambda value: calls.append(('pause', value)),
              get_logger=Mock(), destroy_node=Mock())
    env = dict(AgentCheck=lambda: node, rclpy=Mock())
    exec(compile(ast.Module(body=[main], type_ignores=[]), 'agent_check.py', 'exec'), env)
    with pytest.raises(SystemExit) as exc:
        env['main']()
    assert exc.value.code == (1 if failure else 0)
    assert calls == [('send', ('mode_request', 'estop')), ('pause', False)]
    node.destroy_node.assert_called_once()
    env['rclpy'].shutdown.assert_called_once()
