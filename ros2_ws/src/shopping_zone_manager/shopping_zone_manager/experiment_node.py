"""ROS entry support for bounded, mutually exclusive Nav2 experiments."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState


class ExperimentNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.active_goal = None
        self.pending_goal = None

    def parameter(self, name, default):
        return self.declare_parameter(name, default).value

    def positive(self, name, default):
        value = float(self.parameter(name, default))
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be positive and finite')
        return value

    def wait(self, future, deadline):
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError('ROS request timed out')
        return future.result()

    def pause(self, seconds):
        deadline = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not rclpy.ok():
            raise RuntimeError('ROS context stopped')

    def ready(self, deadline):
        # The action endpoint can exist before the navigator lifecycle is active.
        state_client = self.create_client(GetState, '/bt_navigator/get_state')
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if state_client.wait_for_service(timeout_sec=0.2):
                    state = self.wait(state_client.call_async(GetState.Request()),
                                      min(deadline, time.monotonic()+2.0))
                    if (state.current_state.id == State.PRIMARY_STATE_ACTIVE
                            and self.client.wait_for_server(timeout_sec=0.2)):
                        return
                self.pause(0.2)
        finally:
            self.destroy_client(state_client)
        raise TimeoutError('Active bt_navigator / NavigateToPose server unavailable')

    def navigate(self, goal, deadline):
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = goal.frame_id
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x = goal.x
        request.pose.pose.position.y = goal.y
        request.pose.pose.orientation.z = math.sin(goal.yaw/2)
        request.pose.pose.orientation.w = math.cos(goal.yaw/2)
        self.get_logger().info(f'Navigating to {goal.name} ({goal.x}, {goal.y})')
        self.pending_goal = self.client.send_goal_async(request)
        # A missing acceptance response is ambiguous: abort rather than retry concurrently.
        self.active_goal = self.wait(self.pending_goal, min(deadline, time.monotonic()+15.0))
        self.pending_goal = None
        if not self.active_goal.accepted:
            self.active_goal = None
            return False
        result_future = self.active_goal.get_result_async()
        try:
            result = self.wait(result_future, deadline)
        except TimeoutError:
            self.cancel_active()
            return False
        self.active_goal = None
        return result.status == GoalStatus.STATUS_SUCCEEDED

    def cancel_active(self):
        # Wait for terminal state before allowing another goal or saving a map.
        if self.pending_goal is not None:
            self.active_goal = self.wait(self.pending_goal, time.monotonic()+5.0)
            self.pending_goal = None
        if self.active_goal is None or not self.active_goal.accepted:
            self.active_goal = None
            return
        self.wait(self.active_goal.cancel_goal_async(), time.monotonic()+5.0)
        terminal = self.wait(self.active_goal.get_result_async(), time.monotonic()+5.0)
        if terminal.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                                   GoalStatus.STATUS_ABORTED):
            raise RuntimeError('Goal did not reach a terminal state after cancellation')
        self.active_goal = None

    def close(self):
        try:
            self.cancel_active()
        finally:
            self.destroy_node()
