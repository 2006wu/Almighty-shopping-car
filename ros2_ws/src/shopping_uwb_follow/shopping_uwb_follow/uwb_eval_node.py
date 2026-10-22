"""Evaluate estimates against simulation truth using the latest odometry pose."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from .follow_metrics import sample_metrics, synchronized_sample


def stamp(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9


class UWBEvaluation(Node):
    def __init__(self):
        super().__init__('uwb_eval')
        self.pose = self.truth = None
        self.max_skew = self.declare_parameter('max_skew', 0.15).value
        if not math.isfinite(self.max_skew) or self.max_skew < 0:
            raise ValueError('max_skew must be finite and nonnegative')
        self.publisher = self.create_publisher(Float32, '/uwb/estimate_error', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/uwb/person_ground_truth', self.on_truth, 10)
        self.create_subscription(PointStamped, '/uwb/target', self.on_target, 10)

    def on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id not in ('base_footprint', 'base_link'):
            self.pose = None
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.pose = (stamp(msg), (p.x, p.y, yaw))

    def on_truth(self, msg):
        # sim.launch publishes the identity odom -> sim_world transform.
        self.truth = ((stamp(msg), (msg.point.x, msg.point.y))
                      if msg.header.frame_id in ('sim_world', 'odom') else None)

    def on_target(self, msg):
        if msg.header.frame_id != 'base_link':
            return
        try:
            sample = synchronized_sample(
                (stamp(msg), (msg.point.x, msg.point.y)), self.pose, self.truth, self.max_skew)
            if sample is not None:
                error, _ = sample_metrics(sample)
                self.publisher.publish(Float32(data=error))
        except ValueError:
            return


def main(args=None):
    rclpy.init(args=args)
    node = UWBEvaluation()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
