#!/usr/bin/env python3
"""Initialize AMCL once after its subscriber and the simulator are ready."""
import math
from pathlib import Path

import signal

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml


class InitialPose(Node):
    def __init__(self):
        super().__init__('scenario_initial_pose')
        file = self.declare_parameter('scenario_file', '').value
        self.pose = yaml.safe_load(Path(file).read_text())['robot_pose']
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.sent = False
        self.create_timer(0.2, self.tick)

    def tick(self):
        if self.sent or self.get_clock().now().nanoseconds <= 0 or self.publisher.get_subscription_count() == 0:
            return
        if not any(info.node_name == 'amcl' for info in self.get_subscriptions_info_by_topic('/initialpose')):
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.pose['x'])
        msg.pose.pose.position.y = float(self.pose['y'])
        msg.pose.pose.orientation.z = math.sin(self.pose['yaw']/2)
        msg.pose.pose.orientation.w = math.cos(self.pose['yaw']/2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = math.radians(2)**2
        self.publisher.publish(msg)
        self.sent = True
        self.get_logger().info('Scenario initial pose published; wait for localization before moving')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = InitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
