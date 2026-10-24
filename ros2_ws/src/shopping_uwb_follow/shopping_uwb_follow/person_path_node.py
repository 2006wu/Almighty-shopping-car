import subprocess
import rclpy
from rclpy.node import Node
from .person_path import model_command_position
from .node_common import load_path, seconds


class PersonPath(Node):
    def __init__(self):
        super().__init__('person_path')
        self.path = load_path(self)
        self.declare_parameter('model_lag_compensation_s', 0.3)
        self.pending_started = None
        self.pending = None
        self.create_timer(0.05, self.tick)

    def tick(self):
        if self.pending is not None:
            if self.pending.poll() is None:
                return
            self.get_logger().debug(
                f'Person pose service completion observed after {seconds(self)-self.pending_started:.3f} sim seconds')
            if self.pending.returncode:
                self.get_logger().warning('Person pose service failed; check Gazebo world')
        x, y = model_command_position(
            self.path, seconds(self), self.get_parameter('model_lag_compensation_s').value)
        request = f'name: "person", position: {{x: {x}, y: {y}, z: 0.85}}, orientation: {{w: 1.0}}'
        self.pending_started = seconds(self)
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
