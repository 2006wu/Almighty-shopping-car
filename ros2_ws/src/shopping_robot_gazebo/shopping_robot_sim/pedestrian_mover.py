#!/usr/bin/env python3
"""Drive Fortress cylinder pedestrians and publish measured simulator poses."""
import math
from pathlib import Path

import signal

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import yaml

from shopping_robot_sim.pedestrian_yield import YieldingTrajectory


class PedestrianMover(Node):
    def __init__(self):
        super().__init__('pedestrian_mover')
        file = self.declare_parameter('scenario_file', '').value
        self.people = yaml.safe_load(Path(file).read_text())['pedestrians']
        self.yield_to_robot = self.declare_parameter('yield_to_robot', True).value
        self.trajectories = {p['name']: YieldingTrajectory() for p in self.people}
        self.robot_history = []
        self.create_subscription(Odometry, '/sim/ground_truth_odom', self.on_robot, 10)
        self.publishers_by_name = {p['name']: self.create_publisher(
            Twist, f"/model/{p['name']}/cmd_vel", 10) for p in self.people}
        self.truth = self.create_publisher(PoseArray, '/sim/pedestrians', 10)
        self.create_subscription(TFMessage, '/simulation/pedestrian_poses', self.on_poses, 10)
        self.last = None
        self.measured = {}
        self.create_timer(0.1, self.watchdog)

    def on_robot(self, message):
        if message.header.frame_id == 'sim_world':
            stamp = message.header.stamp
            p = message.pose.pose.position
            sample = (stamp.sec+stamp.nanosec*1e-9, p.x, p.y)
            if self.robot_history and sample[0] < self.robot_history[-1][0]:
                self.robot_history.clear()
            self.robot_history.append(sample)
            self.robot_history = self.robot_history[-32:]

    def on_poses(self, message):
        # Preserve simulator time; never label planned positions as ground truth.
        names = {p['name'] for p in self.people}
        for transform in message.transforms:
            stamp = transform.header.stamp
            now = stamp.sec+stamp.nanosec*1e-9
            if transform.child_frame_id in names and now > 0:
                self.measured[transform.child_frame_id] = (now, transform)
        if not names <= self.measured.keys():
            return
        now = min(self.measured[name][0] for name in names)
        latest = max(self.measured[name][0] for name in names)
        if latest-now > 0.1 or (self.last is not None and now <= self.last):
            return
        self.last = now
        # The 50 Hz odometry callback may be ahead of the 30 Hz actor poses.
        # Use a preceding stamped sample rather than accepting future truth.
        robot = next((r for r in reversed(self.robot_history) if r[0] <= now), None)
        transforms = {name: self.measured[name][1] for name in names}
        stamp = min(self.measured.values(), key=lambda item: item[0])[1].header.stamp
        array = PoseArray()
        array.header.frame_id = 'sim_world'
        array.header.stamp = stamp
        for person in self.people:
            transform = transforms[person['name']].transform
            q = transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            position = transform.translation
            vx, vy = self.trajectories[person['name']].velocity(
                person, now, (position.x, position.y), yaw, robot,
                enabled=self.yield_to_robot)
            command = Twist()
            command.linear.x, command.linear.y = vx, vy
            self.publishers_by_name[person['name']].publish(command)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = position.x, position.y, position.z
            pose.orientation = q
            array.poses.append(pose)
        self.truth.publish(array)

    def stop(self):
        for publisher in self.publishers_by_name.values():
            publisher.publish(Twist())

    def watchdog(self):
        now = self.get_clock().now().nanoseconds/1e9
        if self.last is None or not 0 <= now-self.last < 0.5:
            self.stop()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = PedestrianMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
