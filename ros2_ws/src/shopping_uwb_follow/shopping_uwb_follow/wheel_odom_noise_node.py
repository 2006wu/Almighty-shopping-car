"""Add wheel velocity errors and ensure IMU measurement covariances are nonzero."""
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from .odom_noise import WheelNoise


class WheelOdomNoise(Node):
    def __init__(self):
        super().__init__('wheel_odom_noise')
        defaults = dict(seed=42, linear_scale=1.01, yaw_scale=1.02,
                        linear_stddev=0.01, yaw_stddev=0.02)
        self.noise = WheelNoise(**{key: self.declare_parameter(key, value).value
                                  for key, value in defaults.items()})
        self.wheel_pub = self.create_publisher(Odometry, '/wheel_odom_noisy', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/with_covariance', 10)
        self.create_subscription(Odometry, '/wheel_odom', self.on_wheel, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)

    def on_wheel(self, msg):
        result = deepcopy(msg)
        try:
            vx, vyaw = self.noise.sample(msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        except ValueError:
            return
        result.twist.twist.linear.x = vx
        result.twist.twist.angular.z = vyaw
        result.twist.covariance = self.noise.covariance()
        # Pose remains raw wheel integration and is explicitly excluded from EKF fusion.
        self.wheel_pub.publish(result)

    def on_imu(self, msg):
        result = deepcopy(msg)
        # Fortress bridge covariance propagation varies; never rely on zero covariance.
        result.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        result.angular_velocity_covariance = [0.005**2 if i in (0, 4, 8) else 0.0 for i in range(9)]
        result.linear_acceleration_covariance = [0.05**2 if i in (0, 4, 8) else 0.0 for i in range(9)]
        self.imu_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNoise()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
