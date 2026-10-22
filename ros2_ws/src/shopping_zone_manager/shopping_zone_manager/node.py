import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from .zones import load_zones, lookup


class ZoneNavigator(Node):
    def __init__(self, node_name='zone_navigator'):
        super().__init__(node_name)
        path = self.declare_parameter('zones_file', get_package_share_directory('shopping_zone_manager')+'/config/zones.yaml').value
        self.zones = load_zones(path)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.active = self.pending = False
        self.status = self.create_publisher(String, '/shopping/zone_status', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/shopping/zone_markers', qos)
        self.create_subscription(String, '/shopping/goto_zone', lambda msg: self.navigate(msg.data), 10)
        self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        markers = []
        for index, zone in enumerate(self.zones):
            m = Marker()
            m.header.frame_id = zone.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type = 'zones', index, Marker.TEXT_VIEW_FACING
            m.pose.position.x, m.pose.position.y, m.pose.position.z = zone.x, zone.y, 0.5
            m.pose.orientation.w = 1.0
            m.scale.z, m.color.g, m.color.a, m.text = 0.3, 1.0, 1.0, zone.name
            markers.append(m)
        self.markers.publish(MarkerArray(markers=markers))

    def navigate(self, name):
        try:
            zone = lookup(self.zones, name)
        except ValueError as exc:
            self.status.publish(String(data=str(exc)))
            return False
        if self.active or self.pending:
            self.status.publish(String(data='Busy; wait for current goal'))
            return False
        if not self.client.server_is_ready():
            self.status.publish(String(data='NavigateToPose server unavailable'))
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = zone.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = zone.x, zone.y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(zone.yaw/2), math.cos(zone.yaw/2)
        self.pending = True
        self.client.send_goal_async(goal, feedback_callback=self.feedback).add_done_callback(self.accepted)
        self.status.publish(String(data='Requested '+zone.name))
        return True

    def feedback(self, msg):
        self.status.publish(String(data=f'Distance remaining: {msg.feedback.distance_remaining:.2f} m'))

    def accepted(self, future):
        self.pending = False
        handle = future.result()
        self.active = handle.accepted
        if not handle.accepted:
            self.status.publish(String(data='Rejected'))
            self.finished(False)
            return
        handle.get_result_async().add_done_callback(self.result)

    def result(self, future):
        status = future.result().status
        self.active = False
        self.status.publish(String(data='Succeeded' if status == GoalStatus.STATUS_SUCCEEDED else f'Failed status={status}'))
        self.finished(status == GoalStatus.STATUS_SUCCEEDED)

    def finished(self, success):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ZoneNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
