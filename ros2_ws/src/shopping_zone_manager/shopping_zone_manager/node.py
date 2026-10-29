import math
import time
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from .goal_tracker import GoalTracker, State
from .zones import load_zones, lookup, arrived
from .localization_health import amcl_healthy, MapOdomJumpMonitor, scan_map_agreement
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


class ZoneNavigator(Node):
    def __init__(self, node_name='zone_navigator', enable_interfaces=True):
        super().__init__(node_name)
        path = self.declare_parameter('zones_file', get_package_share_directory('shopping_zone_manager')+'/config/zones.yaml').value
        self.zones = load_zones(path)
        self.pose_buffer = Buffer()
        self.pose_listener = TransformListener(self.pose_buffer, self)
        # Margin over the Nav2 goal checker (0.15 m / 0.25 rad) so coasting and
        # AMCL jitter after Nav2 SUCCEEDED do not flip a real arrival to ABORTED;
        # Covariance is checked separately: TF alone cannot detect AMCL slip.
        self.arrival_xy_tolerance = float(self.declare_parameter('arrival_xy_tolerance', 0.25).value)
        self.arrival_yaw_tolerance = float(self.declare_parameter('arrival_yaw_tolerance', 0.35).value)
        self.arrival_max_age = float(self.declare_parameter('arrival_max_tf_age', 0.5).value)
        self.require_amcl = self.declare_parameter('require_amcl', False).value
        self.amcl_xy_std = float(self.declare_parameter('arrival_max_xy_std', 0.15).value)
        self.amcl_yaw_std = float(self.declare_parameter('arrival_max_yaw_std', 0.35).value)
        self.amcl_max_age = float(self.declare_parameter('arrival_max_amcl_age', 5.0).value)
        self.amcl_sample = None
        self.jump_monitor = MapOdomJumpMonitor(
            float(self.declare_parameter('arrival_max_map_odom_step', 0.3).value),
            float(self.declare_parameter('arrival_max_map_odom_yaw_step', 0.3).value))
        self.create_timer(0.2, self.sample_map_odom)
        # Independent of AMCL covariance: do scan endpoints at the AMCL pose hit
        # mapped obstacles?  Catches a confident wrong pose (wrong aisle/rotated).
        self.min_scan_agreement = float(self.declare_parameter('arrival_min_scan_agreement', 0.5).value)
        self.map_msg = None
        self.scan_msg = None
        self.create_subscription(OccupancyGrid, '/map', self.on_map, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tracker = GoalTracker(self.declare_parameter('response_timeout', 5.0).value)
        self.declare_parameter('preempt', False)
        self.handles = {}
        self.response_futures = {}
        self.result_futures = {}
        self.cancel_futures = {}
        self.last_feedback = -float('inf')
        self.send_count = 0
        self.status = self.create_publisher(String, '/shopping/zone_status', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.controller_type = self.create_publisher(
            String, '/controller_type', QoSProfile(depth=1,
                reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.markers = self.create_publisher(MarkerArray, '/shopping/zone_markers', qos)
        if enable_interfaces:
            self.create_subscription(String, '/shopping/goto_zone', lambda msg: self.navigate(msg.data), 10)
            self.create_subscription(Empty, '/shopping/cancel', lambda msg: self.cancel(), 10)
            self.create_service(Trigger, '/shopping/cancel_goal', self.cancel_service)
        self.create_timer(0.1, lambda: self.apply(self.tracker.tick(time.monotonic())),
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(1.0, self.publish_markers)

    def on_amcl(self, message):
        if message.header.frame_id == 'map':
            self.amcl_sample = (Time.from_msg(message.header.stamp).nanoseconds/1e9,
                                tuple(message.pose.covariance))

    def on_map(self, msg):
        self.map_msg = msg

    def on_scan(self, msg):
        self.scan_msg = msg

    def scan_agreement(self):
        """Return agreement ratio, or None when it cannot be judged."""
        m, scan = getattr(self, 'map_msg', None), getattr(self, 'scan_msg', None)
        if m is None or scan is None or m.header.frame_id != 'map':
            return None
        try:
            tf = self.pose_buffer.lookup_transform('map', scan.header.frame_id, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        info = m.info
        return scan_map_agreement(m.data, info.width, info.height, info.resolution,
                                  (info.origin.position.x, info.origin.position.y), (t.x, t.y, yaw),
                                  scan.ranges, scan.angle_min, scan.angle_increment, scan.range_max)

    def sample_map_odom(self):
        try:
            tf = self.pose_buffer.lookup_transform('map', 'odom', Time())
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        was = self.jump_monitor.jumped
        if (self.jump_monitor.update(t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                and not was):
            self.get_logger().warning('map->odom jumped (AMCL relocalization); '
                                      'current goal arrival will not be trusted')

    def localization_healthy(self):
        if self.amcl_sample is None:
            return not self.require_amcl
        stamp, covariance = self.amcl_sample
        return amcl_healthy(covariance, self.get_clock().now().nanoseconds/1e9-stamp,
                            self.amcl_xy_std, self.amcl_yaw_std, self.amcl_max_age)

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

    def navigate(self, name, preempt=None):
        try:
            zone = lookup(self.zones, name)
        except ValueError:
            zone = None
        effects = self.tracker.request(
            zone, time.monotonic(), self.get_parameter('preempt').value if preempt is None else preempt,
            self.client.server_is_ready())
        self.apply(effects)
        return any(effect.kind in ('send', 'cancel') for effect in effects)

    def cancel(self):
        self.apply(self.tracker.cancel_request())

    def cancel_service(self, request, response):
        self.cancel()
        response.success = True
        response.message = str(self.tracker.status)
        return response

    def apply(self, effects):
        for effect in effects:
            generation = effect.generation
            if effect.kind == 'status':
                self.status.publish(String(data=str(effect.status)))
                self.status_changed(effect.status)
            elif effect.kind == 'send':
                goal = NavigateToPose.Goal()
                zone = effect.zone
                goal.pose.header.frame_id = zone.frame_id
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                goal.pose.pose.position.x, goal.pose.pose.position.y = zone.x, zone.y
                goal.pose.pose.orientation.z = math.sin(zone.yaw / 2)
                goal.pose.pose.orientation.w = math.cos(zone.yaw / 2)
                self.last_feedback = -float('inf')
                if getattr(self, 'jump_monitor', None) is not None:
                    self.jump_monitor.reset()
                self.send_count += 1
                try:
                    self.controller_type.publish(String(data='Fast'))
                    future = self.client.send_goal_async(
                        goal, feedback_callback=lambda msg, g=generation: self.feedback(g, msg))
                    self.response_futures[generation] = future
                    future.add_done_callback(lambda f, g=generation: self.accepted(g, f))
                except Exception as exc:
                    self.get_logger().error(f'Goal send failed: {exc}')
                    # Delivery is uncertain; retain the response timeout quarantine.
            elif effect.kind == 'cancel':
                self.cancel_handle(generation)
            elif effect.kind == 'next':
                self.navigate(effect.zone.name, preempt=False)

    def feedback(self, generation, msg):
        now = time.monotonic()
        if (generation != self.tracker.generation or self.tracker.state != State.ACTIVE
                or now - self.last_feedback < 1.0):
            return
        self.last_feedback = now
        self.status.publish(String(data=f'ACTIVE: {self.tracker.zone.name} '
                                   f'{msg.feedback.distance_remaining:.2f} m remaining'))
        self.progress(msg.feedback.distance_remaining)

    def accepted(self, generation, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Goal response failed: {exc}')
            return  # Keep the future and quarantine until a response can be resolved.
        self.response_futures.pop(generation, None)
        if handle.accepted:
            self.handles[generation] = handle
        self.apply(self.tracker.response(generation, handle.accepted))
        if handle.accepted:
            if generation != self.tracker.generation:
                self.cancel_handle(generation)
            try:
                result = handle.get_result_async()
                self.result_futures[generation] = result
                result.add_done_callback(lambda f, g=generation: self.result(g, f))
            except Exception as exc:
                self.get_logger().error(f'Result request failed; retaining goal handle: {exc}')

    def cancel_handle(self, generation):
        handle = self.handles.get(generation)
        if handle is None:
            return
        try:
            future = handle.cancel_goal_async()
            self.cancel_futures[generation] = future
            future.add_done_callback(lambda f, g=generation: self.canceled(g, f))
        except Exception as exc:
            self.get_logger().error(f'Cancel failed: {exc}')
            self.apply(self.tracker.cancel_response(generation, False))

    def canceled(self, generation, future):
        self.cancel_futures.pop(generation, None)
        try:
            accepted = bool(future.result().goals_canceling)
        except Exception as exc:
            self.get_logger().error(f'Cancel response failed: {exc}')
            accepted = False
        self.apply(self.tracker.cancel_response(generation, accepted))

    def result(self, generation, future):
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f'Result unavailable; goal remains busy: {exc}')
            return
        self.handles.pop(generation, None)
        self.result_futures.pop(generation, None)
        arrival_valid = False
        if (status == 4 and generation == self.tracker.generation
                and self.tracker.zone is not None):
            zone = self.tracker.zone
            try:
                transform = self.pose_buffer.lookup_transform(
                    zone.frame_id, 'base_footprint', Time())
                stamp = Time.from_msg(transform.header.stamp)
                age = (self.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9
                t, q = transform.transform.translation, transform.transform.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                arrival_valid = arrived(
                    zone, (t.x, t.y, yaw), age,
                    xy_tolerance=self.arrival_xy_tolerance,
                    yaw_tolerance=self.arrival_yaw_tolerance,
                    max_age=self.arrival_max_age)
                monitor = getattr(self, 'jump_monitor', None)
                if monitor is not None and monitor.jumped:
                    arrival_valid = False
                    self.get_logger().warning(
                        'Cannot verify arrival: map->odom jumped during goal '
                        f'(max step {monitor.worst[0]:.2f} m / {monitor.worst[1]:.2f} rad)')
                agreement = self.scan_agreement()
                if agreement is not None and agreement < getattr(self, 'min_scan_agreement', 0.5):
                    arrival_valid = False
                    self.get_logger().warning(
                        f'Cannot verify arrival: scan/map agreement {agreement:.2f} < '
                        f'{getattr(self, "min_scan_agreement", 0.5):.2f} (localization likely wrong)')
                elif agreement is not None:
                    self.get_logger().info(f'Arrival scan/map agreement {agreement:.2f}')
                if not self.localization_healthy():
                    arrival_valid = False
                    self.get_logger().warning('Cannot verify arrival: AMCL confidence missing, stale or excessive')
                if not arrival_valid:
                    self.get_logger().warning(
                        f'Nav2 success failed final pose/confidence verification: '
                        f'{zone.name}, distance={math.hypot(t.x-zone.x, t.y-zone.y):.3f}, '
                        f'age={age:.3f}')
            except TransformException as exc:
                self.get_logger().warning(f'Cannot verify final zone pose: {exc}')
        self.apply(self.tracker.result(generation, status, arrival_valid=arrival_valid))

    def status_changed(self, status):
        pass

    def progress(self, distance):
        pass


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = ZoneNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
