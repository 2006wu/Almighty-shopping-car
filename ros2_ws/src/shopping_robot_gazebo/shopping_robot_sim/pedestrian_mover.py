#!/usr/bin/env python3
"""Drive Fortress cylinder pedestrians and publish measured simulator poses."""
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Twist
from tf2_msgs.msg import TFMessage
import yaml

from shopping_robot_sim.trajectory import tracking_velocity


class PedestrianMover(Node):
    def __init__(self):
        super().__init__('pedestrian_mover')
        file = self.declare_parameter('scenario_file', '').value
        self.people = yaml.safe_load(Path(file).read_text())['pedestrians']
        self.publishers_by_name = {p['name']: self.create_publisher(
            Twist, f"/model/{p['name']}/cmd_vel", 10) for p in self.people}
        self.truth = self.create_publisher(PoseArray, '/sim/pedestrians', 10)
        self.create_subscription(TFMessage, '/simulation/poses', self.on_poses, 10)
        self.last = None
        self.create_timer(0.1, self.watchdog)

    def on_poses(self, message):
        # Preserve simulator time; never label planned positions as ground truth.
        transforms = {t.child_frame_id: t for t in message.transforms}
        if not all(p['name'] in transforms for p in self.people):
            return
        stamp = transforms[self.people[0]['name']].header.stamp
        now = stamp.sec+stamp.nanosec*1e-9
        if now <= 0 or (self.last is not None and now <= self.last):
            return
        self.last = now
        array = PoseArray()
        array.header.frame_id = 'sim_world'
        array.header.stamp = stamp
        for person in self.people:
            transform = transforms[person['name']].transform
            q = transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            position = transform.translation
            vx, vy = tracking_velocity(person, now, (position.x, position.y), yaw)
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
    rclpy.init(args=args)
    node = PedestrianMover()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
