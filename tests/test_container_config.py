"""Exercise apt-source rewriting without a network or Docker daemon."""
import importlib.util
from pathlib import Path
import shutil
import subprocess
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('configure_mirror', ROOT/'docker/configure-ros-mirror.py')
mirror = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mirror)


def test_dev_loads_repo_env_and_reports_published_ports(tmp_path):
    scripts = tmp_path/'scripts'
    scripts.mkdir()
    shutil.copy(ROOT/'scripts/dev.sh', scripts/'dev.sh')
    (tmp_path/'.env').write_text('NOVNC_HOST_PORT=6081\nVNC_HOST_PORT=5970\n')
    fake = tmp_path/'docker'
    fake.write_text('''#!/usr/bin/env bash
set -eu
[[ "$1" == compose && "$2" == --env-file && "$3" == "$EXPECTED_ENV" ]]
[[ "$4" == -f && "$5" == "$EXPECTED_COMPOSE" && "$6" == port ]]
[[ "$7" == --index && "$8" == 1 && "$9" == compute ]]
case "${10}" in
  6080) echo 127.0.0.1:6081 ;;
  5900) echo 127.0.0.1:5970 ;;
  *) exit 2 ;;
esac
''')
    fake.chmod(0o755)
    import os
    result = subprocess.run(['bash', str(scripts/'dev.sh'), 'novnc'], cwd='/',
                            env={**os.environ, 'DOCKER': str(fake),
                                 'EXPECTED_ENV': str(tmp_path/'.env'),
                                 'EXPECTED_COMPOSE': str(tmp_path/'docker/docker-compose.yaml')},
                            capture_output=True, text=True, check=True)
    assert result.stdout.strip() == 'http://localhost:6081/vnc.html (VNC: 127.0.0.1:5970)'


@pytest.mark.parametrize('output', [[], ['--output', '/repo/output/report with spaces']])
def test_eval_summary_optional_output_on_host_bash(tmp_path, output):
    import os
    fake = tmp_path/'docker'
    fake.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n')
    fake.chmod(0o755)
    result = subprocess.check_output(
        ['bash', str(ROOT/'scripts/dev.sh'), 'eval-summary', 'run_1', 'run_2', *output],
        env={**os.environ, 'DOCKER': str(fake)}, text=True)
    expected = ['eval_summary', 'run_1', 'run_2', *output]
    assert result.splitlines()[-len(expected):] == expected


@pytest.mark.parametrize('repository', ['http://packages.ros.org/ros2/ubuntu', 'https://packages.ros.org/ros2/ubuntu'])
def test_mirror_rewrites_deb822_and_legacy_only(tmp_path, repository):
    content = {
        'ros2.sources': f'Types: deb\nURIs: {repository}\nSuites: jammy\nComponents: main\nSigned-By: /usr/share/keyrings/ros-archive-keyring.gpg\n',
        'ros2-latest.list': f'deb [signed-by=/key.gpg] {repository} jammy main\n',
        'ubuntu.sources': 'URIs: http://archive.ubuntu.com/ubuntu\n',
    }
    for name, text in content.items():
        (tmp_path/name).write_text(text)
    mirror.configure('', tmp_path)
    assert all((tmp_path/name).read_text() == text for name, text in content.items())
    mirror.configure('https://mirror.example/ros2/ubuntu/', tmp_path)
    for name in ('ros2.sources', 'ros2-latest.list'):
        assert (tmp_path/name).read_text() == content[name].replace(repository, 'https://mirror.example/ros2/ubuntu')
    assert (tmp_path/'ubuntu.sources').read_text() == content['ubuntu.sources']


@pytest.mark.parametrize('url', ['file:///tmp/repo', 'https://', 'https://mirror/ bad'])
def test_bad_mirror_url(tmp_path, url):
    with pytest.raises(ValueError):
        mirror.configure(url, tmp_path)


def test_compose_mirror_arguments_and_local_ports():
    services = yaml.safe_load((ROOT/'docker/docker-compose.yaml').read_text())['services']
    for name in ('compute', 'display'):
        assert services[name]['build']['args']['ROS_APT_MIRROR'] == '${ROS_APT_MIRROR:-}'
        dockerfile = (ROOT/f'docker/Dockerfile.{name}').read_text()
        assert 'ARG ROS_APT_MIRROR=\n' in dockerfile
        assert dockerfile.index('configure-ros-mirror.py "$ROS_APT_MIRROR"') < dockerfile.index('apt-get update')
        assert '--platform' not in dockerfile
    assert services['display']['network_mode'] == 'service:compute'
    assert all(p.startswith('127.0.0.1:') for p in services['compute']['ports'])


def test_compute_installs_robot_localization():
    import xml.etree.ElementTree as ET
    assert 'ros-humble-robot-localization' in (ROOT/'docker/Dockerfile.compute').read_text()
    package = ET.parse(ROOT/'ros2_ws/src/shopping_robot_gazebo/package.xml').getroot()
    assert 'robot_localization' in [item.text for item in package.findall('exec_depend')]
