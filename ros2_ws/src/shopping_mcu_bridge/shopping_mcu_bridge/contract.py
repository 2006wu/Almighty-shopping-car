"""ROS-free contract loader and geometry derived from the description."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def load_contract(path):
    return yaml.safe_load(Path(path).read_text())


def geometry(path):
    tree = ET.parse(path)
    props = {p.attrib['name']: float(p.attrib['value'])
             for p in tree.findall('{http://www.ros.org/wiki/xacro}property')
             if p.attrib['name'] in ('wheel_radius', 'width', 'wheel_width', 'skid_track_factor')}
    return props['wheel_radius'], (props['width']-props['wheel_width'])*props['skid_track_factor']
