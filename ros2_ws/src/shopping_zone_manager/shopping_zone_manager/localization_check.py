"""Compare timestamp-aligned map TF with simulation truth at all zone goals."""
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener

from .experiment_node import ExperimentNode
from .localization_metrics import aggregate, evaluate_zone, validate_truth_frames
from .zones import load_zones


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class LocalizationCheck(ExperimentNode):
    def __init__(self):
        super().__init__('localization_check')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.truth = deque(maxlen=200)
        self.subscription = None

    def subscribe_truth(self):
        topic = self.resolve_topic_name(self.parameter('truth_topic', '/sim/ground_truth_odom'))
        odom_source = self.parameter('odom_source', 'ekf')
        if odom_source not in ('ekf', 'ground_truth'):
            raise ValueError('odom_source must be ekf or ground_truth')
        if topic == '/odom' and odom_source != 'ground_truth':
            self.get_logger().warning(
                'INVALID TRUTH SOURCE: /odom is the EKF estimate, NOT simulator truth. '
                'Use /sim/ground_truth_odom; /odom is truth only with odom_source:=ground_truth.')
            raise ValueError('Refusing /odom as truth unless odom_source:=ground_truth is explicitly set')
        self.get_logger().info(f'Truth source: {topic}; must share the initial map origin/yaw')
        self.subscription = self.create_subscription(
            Odometry, topic, lambda msg: self.truth.append((time.monotonic(), msg)), qos_profile_sensor_data)
        return topic

    def initial_pose(self, deadline):
        publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        try:
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                self.pause(0.1)
            if publisher.get_subscription_count() == 0:
                raise TimeoutError('No /initialpose subscriber; is AMCL running?')
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            from shopping_robot_sim.scenarios import load_scenario
            config = get_package_share_directory('shopping_robot_gazebo')+'/config'
            pose = load_scenario(config, self.parameter('scenario', 'default'))['robot_pose']
            msg.pose.pose.position.x = float(pose['x'])
            msg.pose.pose.position.y = float(pose['y'])
            msg.pose.pose.orientation.z = math.sin(pose['yaw']/2)
            msg.pose.pose.orientation.w = math.cos(pose['yaw']/2)
            msg.pose.covariance[0] = 0.01
            msg.pose.covariance[7] = 0.01
            msg.pose.covariance[35] = math.radians(2)**2
            publisher.publish(msg)
            self.pause(2.0)
        finally:
            self.destroy_publisher(publisher)

    def sample(self, duration, deadline):
        self.truth.clear()
        samples = {}
        seen = set()
        until = min(deadline, time.monotonic()+duration)
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.05)
            for received, msg in list(self.truth):
                stamp = Time.from_msg(msg.header.stamp)
                key = stamp.nanoseconds
                age = (self.get_clock().now().nanoseconds-key)/1e9
                if key in seen or key == 0 or not -0.1 <= age <= 0.5 or time.monotonic()-received > 0.5:
                    continue
                # Truth shares the standard map's fixed world origin/yaw, even without a connecting TF.
                # Do not transform truth with map->odom: that would mask localization error.
                validate_truth_frames(msg.header.frame_id, msg.child_frame_id)
                try:
                    tf = self.buffer.lookup_transform('map', 'base_footprint', stamp).transform
                except TransformException:
                    continue  # Retry queued samples once their TF arrives.
                p = msg.pose.pose
                estimate = (tf.translation.x, tf.translation.y, yaw(tf.rotation))
                truth = (p.position.x, p.position.y, yaw(p.orientation))
                samples[key] = (estimate, truth)
                seen.add(key)
        return [samples[key] for key in sorted(samples)]


def main():
    # Keep the ROS context alive on Ctrl-C so the active Nav2 goal can be canceled.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LocalizationCheck()
    results = []
    report = {'pass': False, 'zones': []}
    report_file = ''
    code = 1
    try:
        report_file = node.parameter('report_file', '')
        zones = load_zones(node.parameter('zones_file', get_package_share_directory('shopping_zone_manager')+'/config/zones.yaml'))
        overall = node.positive('overall_timeout', 1800.0)
        per_goal = node.positive('goal_timeout', 180.0)
        settle = node.positive('settle_seconds', 2.0)
        window = node.positive('sample_seconds', 3.0)
        publish_initial = node.parameter('publish_initial_pose', False)
        deadline = time.monotonic()+overall
        source = node.subscribe_truth()
        node.ready(min(deadline, time.monotonic()+60.0))
        if publish_initial:
            node.initial_pose(min(deadline, time.monotonic()+15.0))
        for zone in zones:
            if time.monotonic() >= deadline:
                break
            success = node.navigate(zone, min(deadline, time.monotonic()+per_goal))
            node.pause(min(settle, max(0.0, deadline-time.monotonic())))
            samples = node.sample(window, deadline)
            row = evaluate_zone(zone, samples, success)
            results.append(row)
            node.get_logger().info(json.dumps(row))
        report = aggregate(results, len(zones))
        report['truth_topic'] = source
        code = 0 if report['pass'] else 1
    except (Exception, KeyboardInterrupt) as exc:
        report = {'pass': False, 'zones': results, 'error': str(exc)}
        node.get_logger().error(f'Localization check stopped: {exc}')
    finally:
        node.get_logger().info(('PASS ' if report['pass'] else 'FAIL ')+json.dumps(report))
        try:
            if report_file:
                Path(report_file).write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        finally:
            try:
                node.close()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
    raise SystemExit(code)
