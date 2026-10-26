import signal

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from .person_path import path_position
from .node_common import load_path, seconds


class PersonPath(Node):
    def __init__(self):
        super().__init__('person_path')
        self.path = load_path(self)
        self.client = self.create_client(SetEntityPose, '/world/supermarket/set_pose')
        self.pending = None
        self.create_timer(0.05, self.tick)

    def tick(self):
        if self.pending is not None:
            if not self.pending.done():
                return
            try:
                success = self.pending.result().success
            except Exception:
                success = False
            if not success:
                self.get_logger().warning('Person pose service failed; check Gazebo world')
            self.pending = None
        if not self.client.service_is_ready():
            return
        x, y = path_position(self.path, seconds(self))
        request = SetEntityPose.Request()
        request.entity.name = 'person'
        request.entity.type = Entity.MODEL
        request.pose.position.x, request.pose.position.y = x, y
        request.pose.position.z = 0.85
        request.pose.orientation.w = 1.0
        self.pending = self.client.call_async(request)

    def destroy_node(self):
        if self.pending is not None and not self.pending.done():
            self.pending.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = PersonPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
