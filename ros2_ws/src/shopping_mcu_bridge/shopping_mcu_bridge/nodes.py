"""Humble ROS boundary for direct DDS firmware and binary serial adapters."""
import importlib
import math
import time
from pathlib import Path

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory

from .contract import load_contract, geometry
from .logic import Firmware
from .codec import Codec
from .wire import FirmwareWire
from .transport import open_transport


def contract():
    return load_contract(Path(get_package_share_directory('shopping_mcu_bridge'))/'config/interface.yaml')


def message_type(spec):
    package, _, name = spec['type'].split('/')
    return getattr(importlib.import_module(package+'.msg'), name)


def qos(spec):
    return QoSProfile(depth=spec['depth'], reliability=(ReliabilityPolicy.BEST_EFFORT
                      if spec['reliability'] == 'best_effort' else ReliabilityPolicy.RELIABLE),
                      durability=(DurabilityPolicy.TRANSIENT_LOCAL
                      if spec.get('durability') == 'transient_local' else DurabilityPolicy.VOLATILE))


def publisher(node, spec):
    return node.create_publisher(message_type(spec), spec['name'], qos(spec))


def subscribe(node, spec, callback):
    return node.create_subscription(message_type(spec), spec['name'], callback, qos(spec))


def steady_timer(node, period, callback):
    return node.create_timer(period, callback, clock=Clock(clock_type=ClockType.STEADY_TIME))


def run(factory, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


class FirmwareNode(Node):
    def __init__(self, serial_mode=False):
        super().__init__('serial_bridge' if serial_mode else 'sim_mcu_firmware')
        self.spec = contract()
        self.topics = self.spec['topics']
        self.serial_mode = serial_mode
        self.hardware = self.declare_parameter('use_hardware', False).value
        port = self.declare_parameter('port', '').value
        baud = self.declare_parameter('baud', 115200).value
        if not serial_mode and (self.hardware or port):
            raise ValueError('sim_mcu_firmware only supports simulation')
        self.pc = self.peer = None
        self.failed = False
        self.codec = Codec(self.spec)
        if serial_mode:
            self.pc, self.peer = open_transport(self.hardware, port, baud)
        radius, track = geometry(Path(get_package_share_directory('shopping_robot_description'))/
                                 'urdf/shopping_robot.urdf.xacro')
        self.firmware = Firmware(radius, track, self.spec['timing'], self.spec['model'])
        self.wire = FirmwareWire(self.firmware, self.spec)
        self.publishers_by_key = {key: publisher(self, spec) for key, spec in self.topics.items()
                                  if spec['direction'] == 'mcu_to_pc'}
        for key, spec in self.topics.items():
            if spec['direction'] == 'pc_to_mcu' or (spec['direction'] == 'sim_control' and not self.hardware):
                subscribe(self, spec, lambda msg, k=key: self.receive(k, msg))
        if not self.hardware:
            self.marker = publisher(self, self.topics['sim_active'])
            steady_timer(self, 1.0, self.publish_marker)
            self.publish_marker()
        steady_timer(self, 0.01, self.tick)

    def publish_marker(self):
        self.marker.publish(message_type(self.topics['sim_active'])(data=True))

    def receive(self, key, msg):
        values = ((msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
                  if key == 'cmd' else (msg.data,))
        now = time.monotonic()
        if self.failed:
            return
        if self.serial_mode:
            self.write(self.pc, self.codec.encode(key, values))
        else:
            self.wire.receive(key, values, now)
            if key == 'button' and self.firmware.connected(now):
                self.publish_value('estop', (self.firmware.estop,))

    def write(self, endpoint, data):
        try:
            count = endpoint.write(data)
            # pyserial returns a count; FDTransport maintains its own bounded queue.
            if count is not None and count != len(data):
                raise OSError('Incomplete serial write')
        except (OSError, TimeoutError) as exc:
            self.fail(exc)

    def fail(self, exc):
        if not self.failed:
            self.get_logger().error('Serial transport failed; holding link lost: '+str(exc))
        self.failed = True
        self.firmware.velocities = [0.0]*4
        self.publish_value('status', (self.spec['status_values'].index('LINK_LOST'),))

    def tick(self):
        if self.failed:
            return
        now = time.monotonic()
        try:
            if not self.serial_mode:
                for key, values in self.wire.telemetry(now):
                    self.publish_value(key, values)
                return
            if self.peer is not None:
                self.wire.feed(self.peer.read(), now)
                for key, values in self.wire.telemetry(now):
                    self.write(self.peer, self.codec.encode(key, values))
                self.peer.write(b'')
            for key, values in self.codec.feed(self.pc.read(4096) if self.hardware else self.pc.read()):
                if key in self.publishers_by_key:
                    self.publish_value(key, values)
            if not self.hardware:
                self.pc.write(b'')
        except (OSError, ValueError) as exc:
            self.fail(exc)

    def publish_value(self, key, values):
        spec = self.topics[key]
        msg = message_type(spec)()
        if key in ('wheel', 'imu') and not all(math.isfinite(x) for x in values):
            return
        if key == 'wheel':
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = spec['frame']
            msg.name = list(self.spec['wheel_names'])
            msg.position, msg.velocity = list(values[:4]), list(values[4:])
        elif key == 'imu':
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = spec['frame']
            msg.orientation.w = 1.0
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity.z = values[0]
            msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
            msg.linear_acceleration_covariance[0] = -1.0
        elif key == 'ticks':
            msg.data = list(values)
        elif key == 'status':
            if values[0] >= len(self.spec['status_values']):
                return
            msg.data = self.spec['status_values'][values[0]]
        else:
            msg.data = values[0]
        self.publishers_by_key[key].publish(msg)

    def destroy_node(self):
        for endpoint in (self.pc, self.peer):
            if endpoint is not None:
                endpoint.close()
        return super().destroy_node()


def firmware_main(args=None):
    run(FirmwareNode, args)


def serial_main(args=None):
    run(lambda: FirmwareNode(serial_mode=True), args)
