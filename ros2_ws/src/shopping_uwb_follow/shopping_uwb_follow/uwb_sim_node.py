import math
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from .node_common import anchors_from_node, load_path, seconds
from .person_path import path_position
from .uwb_model import RangeSimulator


class UWBSimulator(Node):
    def __init__(self):
        super().__init__('uwb_sim')
        anchors = anchors_from_node(self)
        defaults = dict(noise_std=0.02, bias=0.0, nlos_probability=0.0, nlos_bias=0.5, dropout=0.0, seed=42)
        kwargs = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.model = RangeSimulator(anchors, **kwargs)
        self.path = load_path(self)
        self.odom = None
        self.odom_stamp = None
        self.create_subscription(TFMessage, '/simulation/poses', self.on_poses, 10)
        self.publisher = self.create_publisher(Float32MultiArray, '/uwb/ranges', 10)
        self.truth = self.create_publisher(PointStamped, '/uwb/person_ground_truth', 10)
        self.create_timer(0.1, self.tick)

    def on_poses(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'shopping_robot':
                self.odom = transform.transform
                self.odom_stamp = seconds(self)
                return

    def tick(self):
        now = seconds(self)
        if self.odom is None or not 0 <= now-self.odom_stamp < 0.5:
            return
        x, y = path_position(self.path, now)
        q = self.odom.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        dx, dy = x-self.odom.translation.x, y-self.odom.translation.y
        rx, ry = math.cos(yaw)*dx+math.sin(yaw)*dy, -math.sin(yaw)*dx+math.cos(yaw)*dy
        self.publisher.publish(Float32MultiArray(data=self.model.sample(rx, ry)))
        truth = PointStamped()
        truth.header.frame_id = 'sim_world'
        truth.header.stamp = self.get_clock().now().to_msg()
        truth.point.x, truth.point.y = x, y
        self.truth.publish(truth)


def main(args=None):
    rclpy.init(args=args)
    node = UWBSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
