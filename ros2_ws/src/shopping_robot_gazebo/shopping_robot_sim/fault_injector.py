#!/usr/bin/env python3
"""Simulation-only relay; launched exclusively by the opt-in sim launch path."""
from functools import partial
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import yaml
from shopping_robot_sim.faults import FaultEngine


class FaultInjector(Node):
    def __init__(self):
        super().__init__('fault_injector')
        schedule = yaml.safe_load(self.declare_parameter('schedule', '[]').value)
        self.engine = FaultEngine(schedule or [])
        self.origin = None
        self.last_clock = None
        self.status = self.create_publisher(String, '/sim/fault_status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.previous = None
        self.publishers_by_stream = {}
        for stream, kind, topic, qos in (
                ('ranges', Float32MultiArray, '/uwb/ranges', 10),
                ('target', PointStamped, '/uwb/target', 10),
                ('scan', LaserScan, '/scan', qos_profile_sensor_data),
                ('odom', Odometry, '/odom', 10)):
            self.publishers_by_stream[stream] = self.create_publisher(kind, topic, qos)
            self.create_subscription(kind, '/sim/raw'+topic,
                                     partial(self.relay, stream), qos)
        self.create_subscription(String, '/sim/fault', self.command, 10)
        self.create_timer(0.02, self.tick)
        self.tick()

    def elapsed(self):
        now = self.get_clock().now().nanoseconds/1e9
        if self.last_clock is not None and now < self.last_clock:
            self.origin = now if now > 0 else None
        self.last_clock = now
        # Do not anchor schedules before /clock starts.
        if self.origin is None and now > 0:
            self.origin = now
        return now-self.origin if self.origin is not None else 0.0

    def tick(self):
        self.engine.advance(self.elapsed())
        status = json.dumps(sorted(self.engine.active))
        if status != self.previous:
            self.status.publish(String(data=status))
            self.previous = status

    def command(self, msg):
        self.tick()
        try:
            self.engine.command(msg.data, self.elapsed())
        except ValueError as exc:
            self.get_logger().warn(str(exc))
        self.tick()

    def relay(self, stream, msg):
        self.tick()
        output = self.engine.mutate(stream, msg)
        if output is not None:
            self.publishers_by_stream[stream].publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
