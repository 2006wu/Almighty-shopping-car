"""Strict scenario loading and SDF generation; no ROS dependencies."""
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


def number(value, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Expected a finite number')
    if minimum is not None and value < minimum:
        raise ValueError(f'Expected number >= {minimum}')
    return float(value)


def keys(value, required, optional=()):
    if not isinstance(value, dict) or not set(required) <= value.keys() or value.keys()-set(required)-set(optional):
        raise ValueError(f'Expected fields {required}, optional {optional}')


def validate_path(path, static=False):
    keys(path, ('speed', 'waypoints'), ('pauses',))
    speed = number(path['speed'], 0)
    points = path['waypoints']
    if not isinstance(points, list) or len(points) < (1 if static else 2):
        raise ValueError('Not enough waypoints')
    for point in points:
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError('Expected [x, y] waypoint')
        for value in point:
            number(value)
    if not static and (speed <= 0 or not any(p != points[0] for p in points)):
        raise ValueError('Person path requires positive speed and length')
    pauses = path.get('pauses', [0]*len(points))
    if not isinstance(pauses, list) or len(pauses) != len(points):
        raise ValueError('One pause duration required per waypoint')
    for duration in pauses:
        number(duration, 0)
    return path


def load_scenario(config, name):
    config = Path(config)
    if not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_]*', name):
        raise ValueError('Invalid scenario name')
    with (config/'scenarios'/f'{name}.yaml').open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    keys(data, ('robot_pose', 'person_path'), ('pedestrians',))
    pose = data['robot_pose']
    keys(pose, ('x', 'y', 'yaw'))
    for value in pose.values():
        number(value)
    path = data['person_path']
    if isinstance(path, str):
        if path != 'person_path.yaml':
            raise ValueError('Only the canonical person_path.yaml reference is allowed')
        path = yaml.safe_load((config/path).read_text(encoding='utf-8'))
    validate_path(path)
    people = data.get('pedestrians', [])
    if not isinstance(people, list):
        raise ValueError('pedestrians must be a list')
    names = set()
    for person in people:
        keys(person, ('name', 'waypoints', 'speed', 'radius', 'start_delay'), ('pauses',))
        name_value = person['name']
        if not isinstance(name_value, str) or not re.fullmatch(r'ped_[a-z0-9_]+', name_value) or name_value in names:
            raise ValueError('Expected unique ped_ model name')
        names.add(name_value)
        validate_path({k: person[k] for k in ('waypoints', 'speed', 'pauses') if k in person}, static=True)
        if not 0 < number(person['radius']) <= 0.5:
            raise ValueError('Pedestrian radius must be in (0, 0.5]')
        number(person['start_delay'], 0)
    return dict(robot_pose=pose, person_path=path, pedestrians=people)


def generate_world(world_file, scenario):
    root = ET.parse(world_file).getroot()
    world = root.find('world')
    if world is None or world.get('name') != 'supermarket':
        raise ValueError('Scenarios require world supermarket')
    for person in scenario['pedestrians']:
        name = person['name']
        if world.find(f"model[@name='{name}']") is not None:
            raise ValueError('Duplicate world model')
        model = ET.SubElement(world, 'model', name=name)
        x, y = person['waypoints'][0]
        ET.SubElement(model, 'pose').text = f'{x} {y} 0.85 0 0 0'
        link = ET.SubElement(model, 'link', name='body')
        ET.SubElement(link, 'gravity').text = 'false'
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '60'
        inertia = ET.SubElement(inertial, 'inertia')
        r = person['radius']
        for axis, value in [('ixx', 60*(3*r*r+1.7**2)/12), ('iyy', 60*(3*r*r+1.7**2)/12), ('izz', 30*r*r)]:
            ET.SubElement(inertia, axis).text = str(value)
        for tag in ('collision', 'visual'):
            shape = ET.SubElement(link, tag, name=tag)
            cylinder = ET.SubElement(ET.SubElement(shape, 'geometry'), 'cylinder')
            ET.SubElement(cylinder, 'radius').text = str(r)
            ET.SubElement(cylinder, 'length').text = '1.7'
            if tag == 'visual':
                material = ET.SubElement(shape, 'material')
                ET.SubElement(material, 'ambient').text = '0.1 0.4 0.9 1'
                ET.SubElement(material, 'diffuse').text = '0.1 0.4 0.9 1'
        plugin = ET.SubElement(model, 'plugin', filename='ignition-gazebo-velocity-control-system',
                               name='ignition::gazebo::systems::VelocityControl')
        ET.SubElement(plugin, 'topic').text = f'/model/{name}/cmd_vel'
        observer = ET.SubElement(model, 'plugin', filename='ignition-gazebo-pose-publisher-system',
                                 name='ignition::gazebo::systems::PosePublisher')
        for key, value in dict(publish_model_pose='true', publish_nested_model_pose='true',
                               publish_link_pose='false',
                               publish_visual_pose='false', publish_collision_pose='false',
                               publish_sensor_pose='false', use_pose_vector_msg='true',
                               static_publisher='false', update_frequency='30').items():
            ET.SubElement(observer, key).text = value
    return ET.tostring(root, encoding='unicode')
