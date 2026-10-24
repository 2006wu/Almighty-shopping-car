#!/usr/bin/env python3
"""Check repository syntax and model/config consistency without ROS or Docker."""
from pathlib import Path
import py_compile
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
import yaml
import xacro


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT/'ros2_ws/src'


def main():
    def owned(paths):
        return sorted(p for p in paths if 'third_party' not in p.relative_to(ROOT).parts)

    yaml_files = owned([*ROOT.rglob('*.yaml'), *ROOT.rglob('*.rviz')])
    for path in yaml_files:
        yaml.safe_load(path.read_text())
    xml_files = owned([*ROOT.rglob('*.xml'), *ROOT.rglob('*.sdf'),
                        *ROOT.rglob('*.urdf'), *ROOT.rglob('*.xacro')])
    for path in xml_files:
        ET.parse(path)
    python_files = owned(ROOT.rglob('*.py'))
    with tempfile.TemporaryDirectory(prefix='shopping-check-') as tmp:
        for index, path in enumerate(python_files):
            py_compile.compile(str(path), cfile=str(Path(tmp)/f'{index}.pyc'), doraise=True)
        config = yaml.safe_load((SRC/'shopping_uwb_follow/config/uwb_follow.yaml').read_text())['/**']['ros__parameters']
        model = SRC/'shopping_robot_description/urdf/shopping_robot.urdf.xacro'
        for count, mode in ((3, 'ekf'), (2, 'ekf'), (3, 'ground_truth'), (2, 'ground_truth')):
            expanded = xacro.process_file(str(model), mappings={
                'anchor_count': str(count), 'odom_source': mode}).toxml()
            robot = ET.fromstring(expanded)
            Path(tmp, f'robot_{count}_{mode}.urdf').write_text(expanded)
            assert len([link for link in robot.findall('link') if link.get('name').startswith('uwb_anchor_')]) == count
            for i in range(count):
                xyz = robot.find(f"joint[@name='uwb_anchor_{i}_joint']/origin").get('xyz')
                x, y, _ = map(float, xyz.split())
                assert (x, y) == (config['anchor_x'][i], config['anchor_y'][i])
            assert len(robot.findall("joint[@type='continuous']")) == 4
            publishers = [p for p in robot.findall('.//plugin')
                          if p.get('name').endswith('::OdometryPublisher')]
            assert len(publishers) == (2 if mode == 'ground_truth' else 1)
    sys.path.insert(0, str(SRC/'shopping_robot_gazebo'))
    from shopping_robot_sim.scenarios import load_scenario, generate_world
    config = SRC/'shopping_robot_gazebo/config'
    scenarios = sorted((config/'scenarios').glob('*.yaml'))
    for path in scenarios:
        scenario = load_scenario(config, path.stem)
        generated = ET.fromstring(generate_world(SRC/'shopping_robot_gazebo/worlds/supermarket.sdf', scenario))
        for plugin in generated.findall('.//plugin'):
            assert plugin.get('filename').startswith('ignition-gazebo-')
            assert plugin.get('name').startswith('ignition::gazebo::systems::')
    shell_files = sorted([*ROOT.glob('scripts/*.sh'), *ROOT.glob('docker/*-entrypoint')])
    for path in shell_files:
        subprocess.run(['bash', '-n', str(path)], check=True)
    print(f'PASS: {len(yaml_files)} YAML/RViz, {len(xml_files)} XML/SDF/xacro, '
          f'{len(python_files)} Python, {len(shell_files)} Bash, {len(scenarios)} scenarios; '
          '2/3-anchor xacro expansion in ekf/ground_truth modes and anchor configuration consistency')


if __name__ == '__main__':
    main()
