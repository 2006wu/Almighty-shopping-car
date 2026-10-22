import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from .follow_controller import FollowController, ControllerParams, front_obstacle_distance
from .node_common import seconds


class FollowNode(Node):
    def __init__(self):
        super().__init__('follow_controller')
        defaults = ControllerParams()
        params = ControllerParams(**{k: self.declare_parameter(k, v).value for k, v in vars(defaults).items()})
        self.control = FollowController(params)
        self.enabled = self.declare_parameter('enabled', False).value
        self.target = self.target_time = self.scan_time = None
        self.obstacle = float('nan')
        self.last_time = seconds(self)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state = self.create_publisher(String, '/follow/state', 10)
        self.create_subscription(PointStamped, '/uwb/target', self.on_target, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_service(SetBool, '/follow/enable', self.enable)
        self.create_timer(0.05, self.tick)

    def enable(self, req, resp):
        self.enabled = req.data
        if not self.enabled:
            self.control.v = self.control.w = 0.0
            self.publisher.publish(Twist())
        resp.success, resp.message = True, 'Enabled' if self.enabled else 'Disabled'
        return resp

    def on_target(self, msg):
        if msg.header.frame_id != 'base_link':
            return
        self.target = (msg.point.x, msg.point.y)
        self.target_time = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def on_scan(self, msg):
        self.scan_time = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        self.obstacle = front_obstacle_distance(
            msg.ranges, msg.angle_min, msg.angle_increment,
            range_min=msg.range_min, range_max=msg.range_max)

    def tick(self):
        now = seconds(self)
        age = now-self.target_time if self.target_time is not None else float('inf')
        obstacle = self.obstacle if self.scan_time is not None and 0 <= now-self.scan_time <= 0.5 else float('nan')
        v, w, state, reason = self.control.step(self.target, age, obstacle, now-self.last_time, self.enabled)
        self.last_time = now
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.publisher.publish(cmd)
        self.state.publish(String(data=state+': '+reason))


def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
