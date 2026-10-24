import math

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from .follow_controller import (
    FollowController, ControllerParams, NavigationGuard, rotation_scan_clear,
    front_obstacle_distance, side_clearance, side_speed_factor,
)
from .follow_handoff import (
    GoalBook, VelocityDrain, HandoffParams, HandoffSupervisor,
    line_of_sight_clear, scan_valid, transform_xy, standoff_target,
)
from .node_common import seconds
from .estimator import TargetVelocityEstimator


def planar_pose(transform):
    """Extract the planar transform used by the simulation's ground robot."""
    t, q = transform.transform.translation, transform.transform.rotation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return t.x, t.y, yaw



class FollowNode(Node):
    def __init__(self):
        super().__init__('follow_controller')
        defaults = ControllerParams()
        params = ControllerParams(**{k: self.declare_parameter(k, v).value for k, v in vars(defaults).items()})
        self.control = FollowController(params)
        self.nav_guard = NavigationGuard(params)
        self.rotation_clear = False
        self.monitor_odom = self.declare_parameter('monitor_odom', False).value
        self.odom_time = None
        self.odom_pose = None
        self.target_velocity = (0.0, 0.0)
        self.velocity_estimator = TargetVelocityEstimator()
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.enabled = self.declare_parameter('enabled', False).value
        route_around = self.declare_parameter('route_around', True).value
        handoff = HandoffParams(follow_distance=params.follow_distance,
                                obstacle_stop_distance=params.obstacle_stop_distance)
        for key, value in vars(handoff).items():
            if key not in vars(defaults):
                setattr(handoff, key, self.declare_parameter(key, value).value)
        self.supervisor = HandoffSupervisor(handoff, route_around)
        self.robot_half_width = self.declare_parameter('robot_half_width', 0.4).value
        self.los_margin = self.declare_parameter('los_margin', 0.35).value
        if any(not math.isfinite(v) or v <= 0 for v in (self.robot_half_width, self.los_margin)):
            raise ValueError('LOS parameters must be positive and finite')
        self.client = self.tf_buffer = self.tf_listener = None
        if route_around:
            self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.tf_buffer = Buffer(node=self)
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target = self.target_time = self.scan_time = self.scan = None
        self.los_clear_wide = False
        self.obstacle = float('nan')
        self.last_diagnostic = -math.inf
        self.last_nav_command = None
        self.last_time = seconds(self)
        self.goal_handle = None
        self.goal_pending = self.cancel_pending = self.cancel_requested = False
        self.replacement = None
        self.nav_status = 'none'
        self.goals = GoalBook()
        self.drain = VelocityDrain()
        self.first_direct = False
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_own_publish = -math.inf
        self.state = self.create_publisher(String, '/follow/state', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/follow/handoff_goal', 10)
        self.observed_cmd_vel_topic = self.declare_parameter('observed_cmd_vel_topic', '/cmd_vel').value
        self.create_subscription(Twist, self.observed_cmd_vel_topic, self.on_cmd_vel, 10)
        self.create_subscription(Bool, '/follow/enabled', self.on_enabled,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PointStamped, '/uwb/target', self.on_target, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_service(SetBool, '/follow/enable', self.enable)
        self.create_timer(0.05, self.tick)

    def on_enabled(self, msg):
        self.set_enabled(msg.data)

    def set_enabled(self, enabled):
        self.enabled = enabled
        if not self.enabled:
            self.supervisor.step(seconds(self), 'IDLE', None, False, float('nan'), enabled=False)
            self.replacement = None
            self.cancel_goal()
            self.control.v = self.control.w = 0.0
            self.publisher.publish(Twist())
            self.last_own_publish = seconds(self)

    def enable(self, req, resp):
        self.set_enabled(req.data)
        # A disable acknowledgement must not release a pending/active Nav2 goal.
        resp.success = self.enabled or not self.busy()
        resp.message = ('Enabled' if self.enabled else
                        'Disabled' if resp.success else 'Disabled; waiting for navigation termination')
        return resp

    def on_cmd_vel(self, msg):
        # Default launch observes the separate Nav2 input, never our relay.
        # The legacy shared-topic path cannot identify publisher GIDs in Humble;
        # ignore zeros shortly after our own publication there.
        if self.supervisor.mode != 'NAV2' and not self.drain.active:
            return
        now = seconds(self)
        own_zero = msg.linear.x == 0.0 and msg.angular.z == 0.0
        if (getattr(self, 'observed_cmd_vel_topic', '/cmd_vel') == '/cmd_vel'
                and own_zero and now-self.last_own_publish <= 0.2):
            return
        self.last_nav_command = (now, msg.linear.x, msg.angular.z)
        self.drain.observe(now, msg.linear.x, msg.angular.z)

    def on_odom(self, msg):
        self.odom_time = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        position, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.odom_pose = (position.x, position.y, yaw)

    def on_target(self, msg):
        if msg.header.frame_id != 'base_link':
            return
        self.target = (msg.point.x, msg.point.y)
        self.target_time = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if (self.odom_pose is not None and self.odom_time is not None
                and -1e-9 <= self.target_time-self.odom_time <= 0.15):
            self.target_velocity = self.velocity_estimator.update(
                self.target, self.odom_pose, self.target_time)
        else:
            self.velocity_estimator.reset()
            self.target_velocity = (0.0, 0.0)

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        self.obstacle = front_obstacle_distance(
            msg.ranges, msg.angle_min, msg.angle_increment,
            range_min=msg.range_min, range_max=msg.range_max)

    def map_goal_and_los(self):
        """Use nonblocking TF lookups at the corresponding measurement times."""
        self.los_clear_wide = False
        self.rotation_clear = False
        if self.target is None or self.scan is None:
            return None, False
        try:
            robot = planar_pose(self.tf_buffer.lookup_transform(
                'map', 'base_link', Time(seconds=self.target_time)))
            laser = planar_pose(self.tf_buffer.lookup_transform(
                self.scan.header.frame_id, 'base_link', Time.from_msg(self.scan.header.stamp)))
        except TransformException:
            return None, False
        self.rotation_clear = rotation_scan_clear(
            self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
            self.scan.range_min, self.scan.range_max, laser,
            max(math.hypot(0.30, 0.25), self.robot_half_width)+0.05)
        point = transform_xy(standoff_target(self.target, self.control.p.follow_distance), robot)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = point
        yaw = robot[2]+math.atan2(self.target[1], self.target[0])
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        scan = self.scan
        los = line_of_sight_clear(scan.ranges, scan.angle_min, scan.angle_increment,
                                  transform_xy(self.target, laser), self.robot_half_width,
                                  self.los_margin, scan.range_min, scan.range_max)
        self.los_clear_wide = line_of_sight_clear(
            scan.ranges, scan.angle_min, scan.angle_increment,
            transform_xy(self.target, laser),
            self.robot_half_width+self.supervisor.p.los_clear_extra_width,
            self.los_margin, scan.range_min, scan.range_max)
        return pose, los

    def busy(self):
        return self.goal_pending or self.goal_handle is not None

    def send_goal(self, pose):
        if not self.client.server_is_ready():
            self.nav_status = 'unavailable'
            return
        generation = self.goals.begin()
        self.goal_handle = None
        self.cancel_pending = self.cancel_requested = False
        self.goal_pending = True
        self.nav_status = 'active'
        self.last_nav_command = None
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.goal_publisher.publish(pose)
        try:
            future = self.client.send_goal_async(goal)
            self.supervisor.record_goal(seconds(self),
                                        (pose.pose.position.x, pose.pose.position.y))
            future.add_done_callback(lambda f: self.goal_response(f, generation))
        except Exception as exc:
            # Delivery may have succeeded remotely; retain ownership.
            self.nav_status = 'failed'
            self.get_logger().error(f'Unable to send navigation goal; velocity held: {exc}')

    def goal_response(self, future, generation):
        if not self.goals.is_current(generation):
            return
        try:
            handle = future.result()
        except Exception as exc:
            # Ownership remains reserved: delivery may have succeeded remotely.
            self.nav_status = 'failed'
            self.get_logger().error(f'Navigation response unavailable; velocity held: {exc}')
            return
        self.goal_pending = False
        if not handle.accepted:
            self.nav_status = 'failed'
            self.drain.terminal(seconds(self))
            return
        self.goal_handle = handle
        try:
            handle.get_result_async().add_done_callback(
                lambda f: self.goal_result(f, generation))
        except Exception as exc:
            self.nav_status = 'failed'
            self.cancel_requested = True
            self.get_logger().error(f'Unable to request navigation result; velocity held: {exc}')
        if self.cancel_requested:
            self.cancel_goal()

    def cancel_goal(self):
        self.drain.start(seconds(self))
        if not self.busy():
            self.drain.terminal(seconds(self))
        self.cancel_requested = True
        if self.goal_handle is not None and not self.cancel_pending:
            self.cancel_pending = True
            try:
                generation = self.goals.generation
                self.goal_handle.cancel_goal_async().add_done_callback(
                    lambda f: self.cancel_response(f, generation))
            except Exception as exc:
                self.cancel_pending = False
                self.get_logger().error(f'Navigation cancel request failed: {exc}')

    def cancel_response(self, future, generation):
        if not self.goals.is_current(generation):
            return
        try:
            response = future.result()
            if not response.goals_canceling:
                self.get_logger().warn('Navigation cancellation not acknowledged; waiting for terminal result')
        except Exception as exc:
            self.cancel_pending = False
            self.get_logger().error(f'Navigation cancellation failed: {exc}')
        # A cancel response is not proof that the goal has stopped executing.

    def goal_result(self, future, generation):
        if not self.goals.is_current(generation):
            return
        try:
            status = future.result().status
        except Exception as exc:
            self.nav_status = 'failed'
            self.get_logger().error(f'Navigation result unavailable; velocity held: {exc}')
            return
        self.goal_handle = None
        self.cancel_pending = False
        self.nav_status = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                           GoalStatus.STATUS_CANCELED: 'canceled',
                           GoalStatus.STATUS_ABORTED: 'aborted'}.get(status, 'failed')
        self.drain.terminal(seconds(self))

    def tick(self):
        now = seconds(self)
        age = now-self.target_time if self.target_time is not None else float('inf')
        fresh = self.scan_time is not None and 0 <= now-self.scan_time <= 0.5
        obstacle = self.obstacle if fresh else float('nan')
        if getattr(self, 'monitor_odom', False):
            odom_fresh = self.odom_time is not None and 0 <= now-self.odom_time <= 0.5
            fresh = fresh and odom_fresh
            obstacle = self.obstacle if fresh else float('nan')
        pose, los = None, False
        scan = self.scan
        valid = fresh and scan_valid(scan.ranges, scan.angle_min, scan.angle_increment,
                                     scan.range_min, scan.range_max)
        if not valid:
            obstacle = float('nan')
        if self.client is not None:
            pose, los = self.map_goal_and_los()
        p = self.control.p
        clearance = (side_clearance(scan.ranges, scan.angle_min, scan.angle_increment,
                                    p.side_sector_min, p.side_sector_max) if valid else math.nan)
        factor = side_speed_factor(clearance, p.side_slow_clearance,
                                   p.side_free_clearance, p.side_min_speed_factor)
        if self.client is not None and not los:
            # Independent speed caps should not compound into a smaller cap.
            factor = min(factor, p.los_uncertain_speed_factor)
        v, w, state, reason = self.control.step(
            self.target, age, obstacle, now-self.last_time, self.enabled, speed_factor=factor,
            target_velocity=(getattr(self, 'target_velocity', (0.0, 0.0))
                             if getattr(self, 'odom_time', None) is not None and 0 <= now-self.odom_time <= 0.15
                             else (0.0, 0.0)))
        dt = now-self.last_time
        self.last_time = now
        if (not self.enabled and not self.busy()
                and getattr(self, 'observed_cmd_vel_topic', '/cmd_vel') != '/cmd_vel'):
            self.diagnose(now, 'IDLE', 0.0, 0.0, 'Disabled', factor, obstacle, age)
            self.state.publish(String(data='IDLE: Disabled'))
            return
        point = (pose.pose.position.x, pose.pose.position.y) if pose is not None else None
        decision = self.supervisor.step(now, state, self.target, los, obstacle,
                                        self.nav_status, map_target=point,
                                        enabled=self.enabled, scan_valid=valid,
                                        los_clear_wide=self.los_clear_wide)
        if (self.client is not None and self.supervisor.route_around
                and decision.mode == 'DIRECT' and state == 'FOLLOWING'
                and self.supervisor.los.blocked):
            # Driving straight at a hidden person wedges the robot against shelf
            # corners where Nav2 can no longer plan; hold until the handoff fires.
            v = w = 0.0
            self.control.v = self.control.w = 0.0
            state, reason = 'BLOCKED_WAIT', 'Line of sight blocked; holding for Nav2 handoff'
        if decision.action == 'cancel_goal':
            self.replacement = None
            self.cancel_goal()
            self.first_direct = True
        elif decision.action in ('send_goal', 'update_goal'):
            self.replacement = pose
            # Coalesce updates while acceptance is pending; preempt without
            # cancellation once the previous submission has been acknowledged.
        if decision.mode == 'DIRECT':
            self.replacement = None
        if (decision.mode == 'NAV2' and self.replacement is not None
                and not self.goal_pending and self.drain.ready(now)):
            self.drain.reset()
            # Use the newest target, rather than the position at cancel time.
            self.replacement = None
            if pose is not None:
                self.send_goal(pose)
        if self.cancel_requested and self.busy():
            self.cancel_goal()
        waiting = self.busy() or not self.drain.ready(now)
        commanded_v = commanded_w = 0.0
        if decision.direct_publish and not waiting:
            self.nav_guard.step(None, obstacle, False, dt)
            self.drain.reset()
            cmd = Twist()
            if not self.first_direct:
                cmd.linear.x, cmd.angular.z = v, w
            commanded_v, commanded_w = cmd.linear.x, cmd.angular.z
            self.publisher.publish(cmd)
            self.last_own_publish = seconds(self)
            if self.first_direct:
                self.control.v = self.control.w = 0.0
            self.first_direct = False
        else:
            self.control.v = self.control.w = 0.0
            command = getattr(self, 'last_nav_command', None)
            nav_command = (command[1:] if command is not None and 0 <= now-command[0] <= 0.2
                           else None)
            safe = (valid and state not in ('LOST', 'TOO_CLOSE', 'IDLE')
                    and decision.mode == 'NAV2' and self.nav_status == 'active'
                    and pose is not None and not self.drain.active)
            commanded_v, commanded_w = self.nav_guard.step(
                nav_command, obstacle, self.rotation_clear, dt, safe=safe)
            # Relay the guarded Nav2 command on the higher-priority follow input.
            # Raw Nav2 must never bypass the front gate or command expiry.
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = commanded_v, commanded_w
            self.publisher.publish(cmd)
            self.last_own_publish = seconds(self)
        if decision.mode == 'NAV2' and valid and state not in ('LOST', 'TOO_CLOSE', 'IDLE'):
            state = 'NAV2_HANDOFF'
            if self.nav_status == 'unavailable':
                reason = 'Nav2 unavailable; holding for rate-limited retry'
            elif pose is None or obstacle > p.obstacle_stop_distance:
                reason = decision.reason
            else:
                reason = ('Forward blocked; rotation allowed' if self.rotation_clear else
                          'Forward blocked; rotation clearance unavailable')
        elif waiting:
            reason += '; waiting for navigation stop'
        self.diagnose(now, state, commanded_v, commanded_w, reason, factor, obstacle, age)
        self.state.publish(String(data=state+': '+reason))


    def diagnose(self, now, state, v, w, reason, factor, obstacle, age):
        last = getattr(self, 'last_diagnostic', -math.inf)
        if 0 <= now-last < 1.0:
            return
        self.last_diagnostic = now
        distance = math.hypot(*self.target) if self.target is not None else math.nan
        bearing = math.atan2(self.target[1], self.target[0]) if self.target is not None else math.nan
        self.get_logger().info(
            f'state={state} mode={self.supervisor.mode} distance={distance:.3f}m '
            f'bearing={bearing:.3f}rad command_v={v:.3f} command_w={w:.3f} cap_factor={factor:.3f} '
            f'front={obstacle:.3f}m target_age={age:.3f}s '
            f'nav_status={self.nav_status} nav_pending={self.goal_pending} '
            f'los_blocked={self.supervisor.los.blocked} drain={self.drain.active} '
            f'rotation_clear={self.rotation_clear} limit={reason}')

    def stop_before_shutdown(self):
        # SIGINT may already have invalidated the context. The independent
        # watchdog expires even when a final upstream zero cannot be delivered.
        if rclpy.ok(context=self.context):
            try:
                self.cancel_goal()
                self.publisher.publish(Twist())
            except RCLError:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_before_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
