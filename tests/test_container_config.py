"""Exercise apt-source rewriting without a network or Docker daemon."""
import importlib.util
from pathlib import Path
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('configure_mirror', ROOT/'docker/configure-ros-mirror.py')
mirror = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mirror)


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
