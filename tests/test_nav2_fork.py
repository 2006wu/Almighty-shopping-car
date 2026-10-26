"""Guard the simulation-only fork integration without ROS or private source access."""
import ast
import importlib.util
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

from shopping_robot_navigation.stack_selection import select_stack, apt_environment
from shopping_zone_manager.nav_compare_logic import goal_metrics, comparison_table
from test_mode_launch import load

ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT/'ros2_ws/src/shopping_robot_navigation'


@pytest.mark.parametrize('requested,available,expected', [
    ('auto', [], 'apt'), ('auto', ['custom_controller'], 'apt'),
    ('auto', ['custom_controller', 'custom_nav_to_pose'], 'fork'),
    ('apt', ['custom_controller', 'custom_nav_to_pose'], 'apt'),
    ('fork', ['custom_controller', 'custom_nav_to_pose'], 'fork'),
])
def test_selection(requested, available, expected):
    assert select_stack(requested, available) == expected


@pytest.mark.parametrize('requested', ['fork', 'invalid'])
def test_selection_fails_clearly(requested):
    with pytest.raises(ValueError, match='nav2-build|nav2_stack'):
        select_stack(requested, [])


def test_apt_environment_does_not_mutate_or_retain_fork():
    original = dict(AMENT_PREFIX_PATH='/ws/install/a:/nav2_ws/install/nav2_msgs:/opt/ros/humble',
                    PYTHONPATH='/nav2_ws/install/nav2_msgs/local/lib/python3.10/dist-packages:/ws/python',
                    LD_LIBRARY_PATH='/nav2_ws/install/nav2_util/lib:/opt/ros/humble/lib',
                    ROS_DOMAIN_ID='28')
    isolated = apt_environment(original)
    assert isolated['AMENT_PREFIX_PATH'] == '/ws/install/a:/opt/ros/humble'
    assert isolated['PYTHONPATH'] == '/ws/python'
    assert isolated['LD_LIBRARY_PATH'] == '/opt/ros/humble/lib'
    assert isolated['ROS_DOMAIN_ID'] == '28'
    assert '/nav2_ws/' in original['AMENT_PREFIX_PATH']


@pytest.mark.parametrize('spawn', [{}, dict(spawn_x='-2.4', spawn_y='0', spawn_yaw='1.5707963267948966')])
def test_apt_launch_reexec_isolates_binaries_and_forwards_arguments(monkeypatch, spawn):
    monkeypatch.setenv('AMENT_PREFIX_PATH', '/nav2_ws/install/nav2_msgs:/opt/ros/humble')
    env = load('shopping_robot_navigation/launch/navigation.launch.py')
    actions = env['start'](dict(nav2_stack='apt', use_mux='true', slam_mode='map_server',
                               map='/ws/map with spaces.yaml', use_sim_time='true',
                               rpp_desired_speed='0.5', **spawn))
    process = actions[-1]
    assert process.kind == 'ExecuteProcess'
    assert 'map:=/ws/map with spaces.yaml' in process.cmd
    assert 'rpp_desired_speed:=0.5' in process.cmd
    assert '/nav2_ws/' not in process.env['AMENT_PREFIX_PATH']
    assert 'local_setup.bash' in process.cmd[2]
    assert 'exec ros2 launch' in process.cmd[2]
    for name in ('spawn_x', 'spawn_y', 'spawn_yaw'):
        if name in spawn:
            assert f'{name}:={spawn[name]}' in process.cmd
        else:
            assert not any(arg.startswith(name+':=') for arg in process.cmd)


def test_fork_params_and_tree():
    params = yaml.safe_load((NAV/'config/nav2_params_fork.yaml').read_text())
    apt = yaml.safe_load((NAV/'config/nav2_params.yaml').read_text())
    tree = ET.parse(NAV/'behavior_trees/navigate_to_pose_shopping.xml').getroot()
    recovery = tree.find('.//ReactiveFallback[@name="RecoveryFallback"]')
    assert [n.tag for n in recovery] == ['GoalUpdated', 'Sequence']
    assert [n.tag for n in recovery.find('Sequence')] == ['Shrink', 'Wait']
    assert float(recovery.find('Sequence/Shrink').get('shrink_to')) >= 0.4
    assert 'Escape' not in (NAV/'behavior_trees/navigate_to_pose_shopping.xml').read_text()
    assert tree.find('.//StopController/ResetRecovery/RecoveryNode') is not None
    controller = params['controller_server']['ros__parameters']
    for selector, default, plugins in [('ControllerSelector', 'default_controller', 'controller_plugins'),
                                       ('GoalCheckerSelector', 'default_goal_checker', 'goal_checker_plugins')]:
        assert tree.find('.//'+selector).get(default) in controller[plugins]
    follow = tree.find('.//FollowPath')
    assert follow.get('controller_id') == '{controller}'
    assert follow.get('goal_checker_id') == '{goal_checker}'
    for name in controller['controller_plugins']:
        p = controller[name]
        assert p['plugin'] == 'teb_controller::TebController'
        assert p['holonomic'] is False and p['avoidance_mode'] == 'gentle'
        assert p['use_rival_topic'] is False and p['min_v'] == 0.0
        assert 0 < p['max_v'] <= 0.4 and p['max_acc_v'] <= 0.4
        assert 0 < p['gentle_stop_dist'] < p['gentle_slowdown_dist']
    assert all(name in controller for name in controller['goal_checker_plugins'])
    assert controller['NonStopGoalChecker']['plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert controller['Loose']['xy_goal_tolerance'] == 0.15
    behaviors = params['behavior_server']['ros__parameters']
    assert behaviors['behavior_plugins'] == ['shrink', 'wait']
    assert behaviors['shrink']['plugin'] == 'nav2_behaviors/Shrink'
    assert 'escape' not in str(params).lower()
    libraries = params['bt_navigator']['ros__parameters']['plugin_lib_names']
    assert {'nav2_stop_controller_bt_node', 'nav2_reset_recovery_bt_node',
            'nav2_shrink_action_bt_node', 'nav2_controller_selector_bt_node',
            'nav2_goal_checker_selector_bt_node'} <= set(libraries)
    shrink = behaviors['shrink']
    assert shrink['safe_recovery'] is True and shrink['min_inflation_radius'] >= 0.4
    assert shrink['local_inflation_radius_service'] != shrink['global_inflation_radius_service']
    assert shrink['local_inflation_service'] != shrink['global_inflation_service']
    assert params['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'].endswith(
        'navigate_through_poses_shopping.xml')
    through = (NAV/'behavior_trees/navigate_through_poses_shopping.xml').read_text()
    assert 'Escape' not in through and '<Spin' not in through and '<BackUp' not in through
    fork_only = {'mode_service_name', 'radius_service_name', 'shrink_to',
                 'min_inflation_radius', 'inscribed_radius_floor'}
    for name in ('local_costmap', 'global_costmap'):
        layer = params[name][name]['ros__parameters']['inflation_layer']
        assert layer['min_inflation_radius'] >= 0.4 and layer['inscribed_radius_floor'] >= 0.4
        assert layer['radius_service_name'] == shrink[name.split('_')[0]+'_inflation_radius_service']
        assert layer['mode_service_name'] == shrink[name.split('_')[0]+'_inflation_service']
        for key in fork_only:
            layer.pop(key)
    for key in set(apt) - {'controller_server', 'behavior_server', 'bt_navigator', 'planner_server'}:
        assert apt[key] == params[key]
    assert apt['planner_server']['ros__parameters']['GridBased']['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
    assert params['planner_server']['ros__parameters']['GridBased']['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    assert 'behavior_trees' in (NAV/'CMakeLists.txt').read_text()
    for path in (NAV/'config').glob('*.yaml'):
        for node in yaml.safe_load(path.read_text()).values():
            p = node.get('ros__parameters', node.get(next(iter(node)), {}).get('ros__parameters', {}))
            assert p.get('use_sim_time') is True


def test_build_configuration():
    compose = yaml.safe_load((ROOT/'docker/docker-compose.yaml').read_text())
    mounts = compose['services']['compute']['volumes']
    assert '../third_party/eurobot_nav2/src:/nav2_ws/src:ro' in mounts
    for name in ('build', 'install', 'log'):
        assert f'nav2_{name}:/nav2_ws/{name}' in mounts
        assert f'nav2_{name}' in compose['volumes']
    assert not any('/nav2_ws/' in v for v in compose['services']['display']['volumes'])
    entry = (ROOT/'docker/compute-entrypoint').read_text()
    assert entry.index('/opt/ros/humble') < entry.index('/nav2_ws/install') < entry.index('/ws/install')
    packages = (ROOT/'docker/nav2_fork_packages.txt').read_text().split()
    assert {'nav2_bt_navigator', 'nav2_behavior_tree', 'nav2_behaviors', 'nav2_controller',
            'nav2_core', 'nav2_util', 'nav2_msgs', 'nav2_costmap_2d', 'nav2_navfn_planner',
            'custom_controller', 'custom_nav_to_pose'} <= set(packages)
    assert not any(p.startswith(('opennav_', 'custom_layer')) for p in packages)
    assert not {'navigation2_run', 'camera_offset_publisher', 'nav2_rviz_plugins', 'nav2_amcl'} & set(packages)
    build = (ROOT/'scripts/nav2_build.sh').read_text()
    assert '--packages-up-to "${packages[@]}"' in build
    assert '-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF' in build
    assert '--parallel-workers "${NAV2_PARALLEL_WORKERS:-2}"' in build
    assert 'check_nav2_linkage.py' in build and 'check_nav2_contract.py' in build
    assert 'third_party' in (ROOT/'.dockerignore').read_text()
    dev = (ROOT/'scripts/dev.sh').read_text()
    assert 'nav2-build)' in dev and 'nav2-status)' in dev
    assert 'NAV2_FORK_COMMIT=$(git -C' in dev
    for node in ast.walk(ast.parse((NAV/'shopping_robot_navigation/stack_selection.py').read_text())):
        if isinstance(node, ast.Import):
            assert all(a.name != 'rclpy' for a in node.names)


def test_missing_submodule_fails_before_docker(tmp_path):
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    path = scripts/'dev.sh'
    path.write_text((ROOT/'scripts/dev.sh').read_text())
    result = subprocess.run(['bash', str(path), 'nav2-build'], capture_output=True, text=True)
    assert result.returncode == 2 and '未初始化 / not initialized' in result.stderr


def test_contract_preflight():
    spec = importlib.util.spec_from_file_location('contract', ROOT/'scripts/check_nav2_contract.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.missing_parameters('') == list(module.PARAMETERS)
    assert not module.missing_parameters(' '.join(f'".{p}"' for p in module.PARAMETERS))


def test_truth_metrics_and_comparison():
    metrics = goal_metrics([(0, 0, 0), (0.5, 0.3, 0.4), (1, 0.6, 0.8)], (0.6, 0.8))
    assert metrics == dict(seconds=1, path_m=1, final_error_m=0)
    a = dict(stack='apt', goals=[dict(zone='drinks', result='SUCCEEDED', **metrics)])
    f = dict(stack='fork', goals=[dict(zone='drinks', result='TIMEOUT')])
    table = comparison_table(a, f)
    assert 'SUCCEEDED / 1.000 / 1.000 / 0.000' in table
    assert 'TIMEOUT / n/a / n/a / n/a' in table
    with pytest.raises(ValueError, match='sequence'):
        comparison_table(a, dict(stack='fork', goals=[]))
    with pytest.raises(ValueError, match='apt report'):
        comparison_table(f, a)


@pytest.mark.parametrize('samples', [[], [(0, 0, 0)], [(1, 0, 0), (0, 1, 1)],
                                     [(0, 0, 0), (1, 1, 1)], [(0, 0, 0), (0.1, float('nan'), 0)]])
def test_invalid_truth_cannot_be_success_metrics(samples):
    with pytest.raises(ValueError):
        goal_metrics(samples, (0, 0))


def test_available_fork_libraries_and_nonrecovery_structure():
    fork = ROOT/'third_party/eurobot_nav2/src'
    if not (fork/'custom_controller/package.xml').exists():
        pytest.skip('Private submodule is optional')
    cmake = (fork/'Navigation2/nav2_behavior_tree/CMakeLists.txt').read_text()
    params = yaml.safe_load((NAV/'config/nav2_params_fork.yaml').read_text())
    for library in params['bt_navigator']['ros__parameters']['plugin_lib_names']:
        assert f'add_library({library} ' in cmake
    original = ET.parse(fork/'custom_bts/custom_nav_to_pose/behavior_trees/navigate_to_pose_w_selector.xml').getroot()
    shopping = ET.parse(NAV/'behavior_trees/navigate_to_pose_shopping.xml').getroot()
    for tree in (original, shopping):
        parent = tree.find('.//RecoveryNode')
        parent.remove(parent.find('ReactiveFallback'))
    def structure(node):
        return node.tag, node.attrib, [structure(child) for child in node]
    assert structure(original) == structure(shopping)
