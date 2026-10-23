"""Humble mode manager and mux; the independent watchdog owns /cmd_vel."""
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .mode_control import ModeManager, VelocityMux, VelocityLimiter


def retained_qos():
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ModeMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.manager = ModeManager()
        self.mux = VelocityMux(
            timeouts={s: self.declare_parameter(s+'_timeout', v).value for s, v in
                      [('safety_stop', 0.5), ('teleop', 0.5), ('follow', 0.3), ('nav', 0.5)]},
            priorities={s: self.declare_parameter(s+'_priority', v).value for s, v in
                        [('safety_stop', 100), ('teleop', 30), ('follow', 20), ('nav', 10)]},
            max_linear=self.declare_parameter('max_linear', 0.6).value,
            max_angular=self.declare_parameter('max_angular', 1.2).value,
            zero_period=0.05)
        self.limiter = VelocityLimiter(
            self.declare_parameter('linear_acceleration', 0.6).value,
            self.declare_parameter('angular_acceleration', 1.2).value)
        self.manage_follow = self.declare_parameter('manage_follow', False).value
        initial = self.declare_parameter('initial_mode', 'idle').value
        self.output = self.create_publisher(Twist, '/cmd_vel_mux/output', 1)
        self.active = self.create_publisher(String, '/cmd_vel_mux/active', retained_qos())
        self.state = self.create_publisher(String, '/mode_manager/state', retained_qos())
        self.follow_enable = self.create_publisher(Bool, '/follow/enabled', retained_qos())
        self.follow_client = self.create_client(SetBool, '/follow/enable')
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.create_subscription(String, '/mode_manager/request', self.request, 10)
        self.create_subscription(Bool, '/safety/stop', self.safety, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status, retained_qos())
        # Humble subscription callbacks receive only the message.
        self.create_subscription(Twist, '/cmd_vel/nav', self.nav, 1)
        self.create_subscription(Twist, '/cmd_vel/follow', self.follow, 1)
        self.create_subscription(Twist, '/cmd_vel/teleop', self.teleop, 1)
        self.nav_active = False
        self.nav_statuses = {}
        self.cancel_wait = set()
        self.last_nav_motion = -float('inf')
        self.generation = 0
        self.follow_future = self.cancel_future = None
        self.follow_disabled = self.cancel_done = False
        self.request(String(data=initial))
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def status(self, msg):
        for status in msg.status_list:
            key = bytes(status.goal_info.goal_id.uuid)
            self.nav_statuses[key] = status.status
            if status.status in (4, 5, 6):
                self.cancel_wait.discard(key)
        self.nav_active = any(s.status in (1, 2, 3) for s in msg.status_list)

    def receive(self, source, msg):
        if source == 'nav' and (msg.linear.x != 0 or msg.angular.z != 0):
            self.last_nav_motion = time.monotonic()
        if self.manager.pending is None:
            # Reject non-planar/non-finite components rather than hide malformed input.
            extras = (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)
            v, w = (float('nan'), 0.0) if any(x != 0.0 for x in extras) else (msg.linear.x, msg.angular.z)
            self.mux.receive(source, v, w, time.monotonic())

    def nav(self, msg):
        self.receive('nav', msg)

    def follow(self, msg):
        self.receive('follow', msg)

    def teleop(self, msg):
        self.receive('teleop', msg)

    def safety(self, msg):
        if msg.data:
            self.request(String(data='estop'))

    def request(self, msg):
        if self.manager.state == 'ESTOP' and msg.data.upper() != 'RESET':
            return
        try:
            effects = self.manager.request(msg.data)
        except ValueError:
            self.get_logger().warn('Unknown mode request: '+msg.data)
            return
        self.generation += 1
        self.follow_future = self.cancel_future = None
        self.follow_disabled = not self.manage_follow
        # Remember cancellation debt across rapid requests and explicit resets.
        self.cancel_required = (getattr(self, 'cancel_required', False)
                                or 'cancel_nav_goals' in effects
                                or msg.data.lower() in ('guide', 'follow'))
        self.cancel_done = not self.cancel_required
        self.mux.clear()
        self.follow_enable.publish(Bool(data=False))
        self.output.publish(Twist())
        self.state.publish(String(data=self.manager.state))

    def disable_complete(self, future, generation):
        if generation != self.generation:
            return
        try:
            self.follow_disabled = future.result().success
        except Exception as exc:
            self.get_logger().warn(f'Follow disable failed; holding zero: {exc}')
        self.follow_future = None

    def cancel_complete(self, future, generation):
        if generation != self.generation:
            return
        try:
            response = future.result()
            self.cancel_done = response.return_code == CancelGoal.Response.ERROR_NONE
            if self.cancel_done:
                self.cancel_required = False
                for info in response.goals_canceling:
                    key = bytes(info.goal_id.uuid)
                    if self.nav_statuses.get(key) not in (4, 5, 6):
                        self.cancel_wait.add(key)
        except Exception as exc:
            self.get_logger().warn(f'Navigation cancel failed; holding zero: {exc}')
        self.cancel_future = None

    def tick(self):
        now = time.monotonic()
        if not self.follow_disabled and self.follow_future is None and self.follow_client.service_is_ready():
            self.follow_future = self.follow_client.call_async(SetBool.Request(data=False))
            generation = self.generation
            self.follow_future.add_done_callback(lambda f: self.disable_complete(f, generation))
        if (self.follow_disabled and not self.cancel_done and self.cancel_future is None
                and self.cancel_client.service_is_ready()):
            # Zero UUID and zero stamp cancel all goals (Humble CancelGoal semantics).
            self.cancel_future = self.cancel_client.call_async(CancelGoal.Request())
            generation = self.generation
            self.cancel_future.add_done_callback(lambda f: self.cancel_complete(f, generation))
        complete = (self.follow_disabled and self.cancel_done and not self.nav_active and not self.cancel_wait
                    and now-self.last_nav_motion >= 0.55)
        for effect in self.manager.step(complete):
            if effect == 'enable_follow':
                self.follow_enable.publish(Bool(data=True))
        result = self.mux.select(self.manager.state, now)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = self.limiter.step(result.velocity, now)
        self.output.publish(cmd)
        self.active.publish(String(data=result.source+(':invalid' if result.invalid else '')))
        self.state.publish(String(data=self.manager.state))


def main(args=None):
    rclpy.init(args=args)
    node = ModeMuxNode()
    try:
        rclpy.spin(node)
    finally:
        node.output.publish(Twist())
        node.follow_enable.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()
