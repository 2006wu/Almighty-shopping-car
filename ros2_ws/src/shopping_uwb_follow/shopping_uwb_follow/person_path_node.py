import subprocess
import rclpy
from rclpy.node import Node
from .person_path import path_position
from .node_common import load_path, seconds


class PersonPath(Node):
    def __init__(self):
        super().__init__('person_path')
        self.path = load_path(self)
        self.pending = None
        self.create_timer(0.1, self.tick)

    def tick(self):
        if self.pending is not None:
            if self.pending.poll() is None:
                return
            if self.pending.returncode:
                self.get_logger().warning('Person pose service failed; check Gazebo world')
        x, y = path_position(self.path, seconds(self))
        request = f'name: "person", position: {{x: {x}, y: {y}, z: 0.85}}, orientation: {{w: 1.0}}'
        self.pending = subprocess.Popen(
            ['ign', 'service', '-s', '/world/supermarket/set_pose', '--reqtype',
             'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '500', '--req', request],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def destroy_node(self):
        if self.pending is not None and self.pending.poll() is None:
            self.pending.terminate()
            self.pending.wait(timeout=2)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PersonPath()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
