"""Interpolate world truth to estimate stamps and report spawn-relative drift."""
from collections import deque
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from .odom_metrics import DriftMetrics, Pose2, interpolate_pose


def pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return Pose2(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, p.x, p.y,
                 math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))


class OdomDriftMonitor(Node):
    def __init__(self):
        super().__init__('odom_drift_monitor')
        self.truth = deque(maxlen=200)
        self.estimates = deque(maxlen=200)
        self.metrics = DriftMetrics()
        self.latest = None
        self.publisher = self.create_publisher(Float32MultiArray, '/sim/odom_drift', 10)
        self.create_subscription(Odometry, '/odom', self.on_estimate, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sim/ground_truth_odom', self.on_truth, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)

    def accept(self, msg, queue):
        try:
            value = pose(msg)
        except ValueError:
            return
        if queue and value.time < queue[-1].time:
            # Reset all history when simulation time moves backwards.
            self.truth.clear()
            self.estimates.clear()
            self.metrics = DriftMetrics()
            self.latest = None
        if not queue or value.time > queue[-1].time:
            queue.append(value)
        self.process()

    def on_estimate(self, msg):
        if msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_footprint':
            self.accept(msg, self.estimates)

    def on_truth(self, msg):
        if msg.header.frame_id == 'sim_world' and msg.child_frame_id == 'base_footprint_gt':
            self.accept(msg, self.truth)

    def process(self):
        while self.estimates and len(self.truth) >= 2:
            estimate = self.estimates[0]
            if estimate.time > self.truth[-1].time:
                return
            self.estimates.popleft()
            while len(self.truth) > 2 and self.truth[1].time <= estimate.time:
                self.truth.popleft()
            truth = interpolate_pose(self.truth[0], self.truth[1], estimate.time)
            if truth is not None:
                self.latest = self.metrics.update(estimate, truth)

    def report(self):
        if self.latest is None or self.metrics.previous is None:
            return
        age = self.get_clock().now().nanoseconds*1e-9-self.metrics.previous.time
        if not 0 <= age <= 0.5:
            return
        d = self.latest
        # Order: metres, radians, travelled metres, metres/metre, radians/metre.
        data = [d.translation, d.heading, d.distance,
                d.translation_per_metre if d.translation_per_metre is not None else float('nan'),
                d.heading_per_metre if d.heading_per_metre is not None else float('nan')]
        self.publisher.publish(Float32MultiArray(data=data))
        self.get_logger().info(f'Odometry drift: {data} [m, rad, m, m/m, rad/m]')


def main(args=None):
    rclpy.init(args=args)
    node = OdomDriftMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
