"""Independent wall-clock watchdog: the sole final simulation velocity publisher."""
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from .mode_control import OutputWatchdog, SafetyVelocityLimiter, ZERO


class WatchdogNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_watchdog')
        self.watchdog = OutputWatchdog(self.declare_parameter('timeout', 0.25).value)
        self.limiter = SafetyVelocityLimiter()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Twist, '/cmd_vel_mux/output', self.receive, 1)
        self.create_subscription(Bool, '/safety/stop', self.safety, 10)
        self.latched = False
        self.create_subscription(String, '/mode_manager/request', self.request, 10)
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def safety(self, msg):
        if msg.data:
            self.latched = True
            # Stop independently of manager scheduling.
            self.watchdog.receive(ZERO, time.monotonic())
            self.limiter.step(ZERO, time.monotonic())
            self.publisher.publish(Twist())

    def request(self, msg):
        if msg.data.lower() == 'estop':
            self.safety(Bool(data=True))
        elif msg.data.lower() == 'reset':
            self.watchdog.receive(ZERO, time.monotonic())
            self.latched = False

    def receive(self, msg):
        self.watchdog.receive((msg.linear.x, msg.angular.z), time.monotonic())

    def tick(self):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = ZERO if self.latched else self.watchdog.output(time.monotonic())
        cmd.linear.x, cmd.angular.z = self.limiter.step((cmd.linear.x, cmd.angular.z), time.monotonic())
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
