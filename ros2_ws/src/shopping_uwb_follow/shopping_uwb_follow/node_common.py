from ament_index_python.packages import get_package_share_directory
import yaml
from .uwb_model import Anchor


def anchors_from_node(node):
    xs = node.declare_parameter('anchor_x', [0.0, 0.0, -0.25]).value
    ys = node.declare_parameter('anchor_y', [0.22, -0.22, 0.0]).value
    if len(xs) != len(ys) or len(xs) not in (2, 3):
        raise ValueError('Anchor arrays must have equal length, two or three')
    return [Anchor(x, y) for x, y in zip(xs, ys)]


def load_path(node):
    path = node.declare_parameter('person_path', '').value
    if not path:
        raise ValueError('Set person_path to the shared supermarket path YAML')
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def seconds(node):
    return node.get_clock().now().nanoseconds/1e9
