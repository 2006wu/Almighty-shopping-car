import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from .node_common import anchors_from_node, seconds
from .estimator import trilaterate, TargetFilter


class UWBEstimator(Node):
    def __init__(self):
        super().__init__('uwb_estimator')
        self.anchors = anchors_from_node(self)
        self.filter = TargetFilter(self.declare_parameter('lost_timeout', 0.5).value)
        self.publisher = self.create_publisher(PointStamped, '/uwb/target', 10)
        self.quality = self.create_publisher(Float32, '/uwb/quality', 10)
        self.markers = self.create_publisher(MarkerArray, '/uwb/markers', 10)
        self.create_subscription(Float32MultiArray, '/uwb/ranges', self.on_ranges, 10)

    def on_ranges(self, msg):
        now = seconds(self)
        previous = self.filter.get(now)
        try:
            value = trilaterate(self.anchors, msg.data, (previous.x, previous.y) if previous else None)
        except ValueError:
            self.quality.publish(Float32(data=0.0))
            return
        if value.quality < 0.15:
            return
        value = self.filter.update(value, now)
        target = PointStamped()
        target.header.frame_id = 'base_link'
        target.header.stamp = self.get_clock().now().to_msg()
        target.point.x, target.point.y = value.x, value.y
        self.publisher.publish(target)
        self.quality.publish(Float32(data=value.quality))
        marker = Marker()
        marker.header = target.header
        marker.ns, marker.id, marker.type = 'uwb_target', 0, Marker.SPHERE
        marker.pose.position = target.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color.g = marker.color.a = 1.0
        marker.lifetime.sec = 1
        self.markers.publish(MarkerArray(markers=[marker]))


def main(args=None):
    rclpy.init(args=args)
    node = UWBEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
