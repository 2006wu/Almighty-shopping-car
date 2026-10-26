import math
import time
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from .goal_tracker import GoalTracker, State
from .zones import load_zones, lookup


class ZoneNavigator(Node):
    def __init__(self, node_name='zone_navigator', enable_interfaces=True):
        super().__init__(node_name)
        path = self.declare_parameter('zones_file', get_package_share_directory('shopping_zone_manager')+'/config/zones.yaml').value
        self.zones = load_zones(path)
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
        self.markers = self.create_publisher(MarkerArray, '/shopping/zone_markers', qos)
        if enable_interfaces:
            self.create_subscription(String, '/shopping/goto_zone', lambda msg: self.navigate(msg.data), 10)
            self.create_subscription(Empty, '/shopping/cancel', lambda msg: self.cancel(), 10)
            self.create_service(Trigger, '/shopping/cancel_goal', self.cancel_service)
        self.create_timer(0.1, lambda: self.apply(self.tracker.tick(time.monotonic())),
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
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
                self.send_count += 1
                try:
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
        self.apply(self.tracker.result(generation, status))

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
