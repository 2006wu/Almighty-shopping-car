"""Simulation scan guard between arbitration and the final wall-clock watchdog."""
import json
import math
import time

import signal

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .mode_control import OutputWatchdog
from .obstacle_guard import ObstacleGuard, scan_points, scan_fresh, load_nav_footprint


class ObstacleGuardNode(Node):
    def __init__(self):
        super().__init__('obstacle_guard')
        footprint_file = self.declare_parameter('footprint_file', '').value
        half_length, half_width = load_nav_footprint(footprint_file) if footprint_file else (0.30, 0.25)
        defaults = dict(half_length=half_length, half_width=half_width, margin=0.10,
                        reaction=0.25, deceleration=0.6, angular_deceleration=1.2,
                        slow_extra=0.50, hysteresis=0.05, clear_time=0.30)
        self.guard = ObstacleGuard(**{k: self.declare_parameter(k, v).value
                                      for k, v in defaults.items()})
        self.command = OutputWatchdog()
        self.timeout = self.declare_parameter('scan_timeout', 0.5).value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.scan = None
        self.received = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel_guard/output', 1)
        self.state = self.create_publisher(String, '/safety/obstacle_state', 10)
        self.create_subscription(Twist, '/cmd_vel_mux/output', self.receive, 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        self.command.receive((msg.linear.x, msg.angular.z), time.monotonic())

    def on_scan(self, msg):
        self.scan, self.received = msg, time.monotonic()

    def tick(self):
        wall = time.monotonic()
        now = self.get_clock().now().nanoseconds/1e9
        points, fresh = None, False
        scan = self.scan
        if scan is not None:
            stamp = scan.header.stamp.sec+scan.header.stamp.nanosec/1e9
            fresh = scan_fresh(now, stamp, wall, self.received, self.timeout)
            if fresh:
                try:
                    tf = self.buffer.lookup_transform('base_footprint', scan.header.frame_id,
                                                      Time.from_msg(scan.header.stamp)).transform
                    q = tf.rotation
                    # Reject non-planar laser mounts rather than flattening them.
                    if abs(q.x) > 1e-4 or abs(q.y) > 1e-4:
                        raise ValueError('Non-planar laser transform')
                    pose = (tf.translation.x, tf.translation.y,
                            2*math.atan2(q.z, q.w))
                    points = scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                         scan.range_min, scan.range_max, pose)
                except (TransformException, ValueError):
                    fresh = False
        result = self.guard.step(points, self.command.output(wall), wall, fresh)
        msg = Twist()
        msg.linear.x, msg.angular.z = result.velocity
        self.publisher.publish(msg)
        self.state.publish(String(data=json.dumps({
            'state': result.state, 'scale': result.scale, 't': now,
            'swept_clearance': result.clearance if math.isfinite(result.clearance) else None})))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = ObstacleGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
