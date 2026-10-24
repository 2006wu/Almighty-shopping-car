"""Exercise actual callback bodies with fake messages/futures, without ROS imports.

These checks cover ownership races, not ROS transport or Humble integration.
"""
import ast
import math
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from shopping_uwb_follow.follow_controller import FollowController, side_clearance, side_speed_factor
from shopping_uwb_follow.follow_handoff import GoalBook, VelocityDrain, HandoffSupervisor


def completed(value=None, error=None):
    future = Future()
    if error:
        future.set_exception(error)
    else:
        future.set_result(value)
    return future


def node_class():
    source = Path(__file__).resolve().parents[1]/'shopping_uwb_follow/follow_controller_node.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'FollowNode')
    cls.bases = []
    namespace = dict(math=math, side_clearance=side_clearance,
                     side_speed_factor=side_speed_factor, seconds=lambda node: node.now,
                     Twist=lambda: NS(linear=NS(x=0), angular=NS(z=0)),
                     String=lambda **kwargs: NS(**kwargs),
                     NavigateToPose=NS(Goal=lambda: NS()),
                     GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5),
                     scan_valid=lambda *args: True)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['FollowNode']


def make_node():
    cls = node_class()
    node = cls.__new__(cls)
    node.now, node.last_time = 0.0, -0.05
    node.control = FollowController()
    node.supervisor = HandoffSupervisor()
    node.enabled = True
    node.client = Mock()
    node.client.server_is_ready.return_value = True
    node.goal_handle = None
    node.goal_pending = node.cancel_pending = node.cancel_requested = False
    node.replacement = None
    node.nav_status = 'none'
    node.goals = GoalBook()
    node.goals.begin()
    node.drain = VelocityDrain()
    node.first_direct = False
    node.target = (3, 0)
    node.target_time = node.scan_time = 0.0
    node.scan = NS(ranges=[2], angle_min=0, angle_increment=1, range_min=0.1, range_max=12)
    node.los_clear_wide = False
    node.obstacle = 0.4
    node.publisher = Mock()
    node.state = Mock()
    node.goal_publisher = Mock()
    node.get_logger = Mock()
    pose = NS(pose=NS(position=NS(x=3, y=5)))
    node.map_goal_and_los = lambda: (pose, False)
    return node


def test_cancel_before_acceptance_waits_for_handle_and_terminal_result():
    node = make_node()
    node.goal_pending = True
    node.cancel_goal()
    assert node.busy() and node.cancel_requested
    handle = Mock(accepted=True)
    node.goal_response(completed(handle), node.goals.generation)
    assert node.goal_handle is handle and not node.goal_pending
    handle.cancel_goal_async.assert_called_once()
    node.cancel_response(completed(NS(goals_canceling=[1])), node.goals.generation)
    assert node.busy()  # Cancel acknowledgement does not transfer ownership.
    node.goal_result(completed(NS(status=5)), node.goals.generation)
    assert not node.busy() and node.drain.terminal_at == 0.0
    assert not node.drain.ready(0.29) and node.drain.ready(0.3)


def test_rejection_and_transport_error():
    node = make_node()
    node.goal_pending = True
    node.goal_response(completed(NS(accepted=False)), node.goals.generation)
    assert not node.busy() and node.nav_status == 'failed'
    node.goal_pending = True
    node.goal_response(completed(error=RuntimeError('response lost')), node.goals.generation)
    assert node.busy() and node.nav_status == 'failed'
    node.goal_handle = Mock()
    node.goal_result(completed(error=RuntimeError('result lost')), node.goals.generation)
    assert node.busy()


def test_nav2_ownership_and_zero_first_direct_command_after_drain():
    node = make_node()
    node.tick()
    node.publisher.reset_mock()
    node.now = node.target_time = node.scan_time = 1.0
    node.tick()
    assert node.supervisor.mode == 'NAV2' and node.goal_pending
    node.publisher.publish.assert_not_called()
    node.goal_pending = False
    node.goal_handle = Mock()
    node.goal_result(completed(NS(status=4)), node.goals.generation)
    node.now = node.target_time = node.scan_time = 1.1
    node.tick()
    node.publisher.publish.assert_not_called()
    node.now = node.target_time = node.scan_time = 1.41
    node.tick()
    command = node.publisher.publish.call_args.args[0]
    assert command.linear.x == command.angular.z == 0
    assert node.control.v == node.control.w == 0


def test_disable_cancels_pending_request_and_stops_immediately():
    node = make_node()
    node.goal_pending = True
    node.control.v = node.control.w = 0.5
    node.enable(NS(data=False), NS())
    assert node.cancel_requested and not node.enabled
    assert node.control.v == node.control.w == 0
    command = node.publisher.publish.call_args.args[0]
    assert command.linear.x == command.angular.z == 0


def test_update_preempts_without_cancel_or_drain_and_ignores_old_callbacks():
    node = make_node()
    first_response, second_response = Future(), Future()
    node.client.send_goal_async.side_effect = [first_response, second_response]
    node.tick()
    node.now = node.target_time = node.scan_time = 1.0
    node.tick()
    old_generation = node.goals.generation
    old_handle = Mock(accepted=True)
    old_result = Future()
    old_handle.get_result_async.return_value = old_result
    first_response.set_result(old_handle)
    new_pose = NS(pose=NS(position=NS(x=4, y=5)))
    node.map_goal_and_los = lambda: (new_pose, False)
    node.now = node.target_time = node.scan_time = 1.5
    node.publisher.reset_mock()
    node.tick()
    assert node.client.send_goal_async.call_count == 2
    old_handle.cancel_goal_async.assert_not_called()
    assert not node.drain.active and not node.cancel_requested
    node.publisher.publish.assert_not_called()
    new_handle = Mock(accepted=True)
    second_response.set_result(new_handle)
    old_result.set_result(NS(status=5))
    node.goal_result(completed(error=RuntimeError('old result lost')), old_generation)
    node.goal_response(completed(error=RuntimeError('old response lost')), old_generation)
    node.cancel_pending = True
    node.cancel_response(completed(NS(goals_canceling=[1])), old_generation)
    assert node.cancel_pending and node.goal_handle is new_handle
    assert node.nav_status == 'active' and not node.first_direct
    assert node.supervisor.retry_after == -float('inf')
    assert node.supervisor.last_goal == (4, 5)
    assert node.supervisor.last_goal_time == 1.5


@pytest.mark.parametrize("older_first", [False, True])
def test_late_superseded_acceptance_does_not_cancel_current_goal(older_first):
    node = make_node()
    older, newer = Future(), Future()
    node.client.send_goal_async.side_effect = [older, newer]
    pose, _ = node.map_goal_and_los()
    node.send_goal(pose)
    node.send_goal(pose)
    current = Mock(accepted=True)
    late = Mock(accepted=True)
    if older_first:
        older.set_result(late)
        assert node.goal_pending and node.goal_handle is None
    newer.set_result(current)
    if not older_first:
        older.set_result(late)
    assert node.goal_handle is current and node.nav_status == 'active'
    current.cancel_goal_async.assert_not_called()
    late.cancel_goal_async.assert_not_called()
    late.get_result_async.assert_not_called()


def test_invalid_scan_during_navigation_cancels_and_publishes_zero():
    node = make_node()
    node.tick()
    node.now = node.target_time = node.scan_time = 1.0
    node.tick()
    node.goal_pending = False
    node.goal_handle = Mock()
    node.tick.__globals__['scan_valid'] = lambda *args: False
    node.now = node.target_time = node.scan_time = 1.1
    node.publisher.reset_mock()
    node.tick()
    assert node.supervisor.mode == 'DIRECT' and node.cancel_requested
    node.goal_handle.cancel_goal_async.assert_called_once()
    cmd = node.publisher.publish.call_args.args[0]
    assert cmd.linear.x == cmd.angular.z == 0
    assert node.state.publish.call_args.args[0].data.startswith('OBSTACLE_STOP:')


def test_cmd_vel_ignores_own_safety_zero_including_delayed_delivery():
    # Humble callbacks have no publisher GID: zero twists right after our own
    # publication are treated as ours; everything else is Nav2's smoother.
    node = make_node()
    node.drain.start(0)
    moving = NS(linear=NS(x=0.3), angular=NS(z=0))
    node.on_cmd_vel(moving)
    node.now = 0.1
    node.last_own_publish = 0.1
    node.on_cmd_vel(NS(linear=NS(x=0.0), angular=NS(z=0.0)))
    assert node.drain.last_message == 0 and not node.drain.zero
    node.drain.terminal(0.1)
    assert not node.drain.ready(0.2)
    node.now = 0.5
    node.on_cmd_vel(NS(linear=NS(x=0.0), angular=NS(z=0.0)))
    assert node.drain.ready(0.5)


@pytest.mark.parametrize('safety', ['lost', 'too_close', 'disabled', 'invalid_scan'])
@pytest.mark.parametrize('draining', [False, True])
def test_safety_zero_bypasses_navigation_and_drain(safety, draining):
    node = make_node()
    node.tick()
    node.now = node.target_time = node.scan_time = 1.0
    node.tick()
    handle = Mock(accepted=True)
    node.goal_response(completed(handle), node.goals.generation)
    if draining:
        node.supervisor.mode = 'DIRECT'
        node.cancel_goal()
    node.now = node.target_time = node.scan_time = 1.1
    if safety == 'lost':
        node.target_time = 0
    elif safety == 'too_close':
        node.target = (0.2, 0)
    elif safety == 'disabled':
        node.enabled = False
    else:
        node.tick.__globals__['scan_valid'] = lambda *args: False
    node.publisher.reset_mock()
    node.tick()
    assert node.cancel_requested and node.busy()
    assert handle.cancel_goal_async.call_count == 1
    cmd = node.publisher.publish.call_args.args[0]
    assert cmd.linear.x == cmd.angular.z == 0


def test_cancel_ack_is_not_reissued_each_tick():
    node = make_node()
    node.goal_handle = Mock()
    node.cancel_goal()
    node.cancel_response(completed(NS(goals_canceling=[1])), node.goals.generation)
    node.cancel_goal()
    node.goal_handle.cancel_goal_async.assert_called_once()
    assert node.busy() and not node.drain.ready(100)


def test_blocked_line_of_sight_holds_instead_of_driving_into_shelf():
    node = make_node()
    node.obstacle = 3.0  # FOLLOWING state, but the person is hidden behind a shelf.
    node.tick()
    assert node.publisher.publish.call_args.args[0].linear.x > 0
    assert node.state.publish.call_args.args[0].data.startswith('FOLLOWING')
    node.now = node.target_time = node.scan_time = 0.3
    node.tick()
    command = node.publisher.publish.call_args.args[0]
    assert command.linear.x == command.angular.z == 0
    assert node.state.publish.call_args.args[0].data.startswith('BLOCKED_WAIT')
    node.supervisor.route_around = False
    node.now = node.target_time = node.scan_time = 0.4
    node.tick()
    assert node.publisher.publish.call_args.args[0].linear.x > 0


def test_aisle_edge_flicker_keeps_direct_following_without_handoffs():
    node = make_node()
    node.obstacle = 3.0
    pose, _ = node.map_goal_and_los()
    for index in range(40):
        raw_clear = index % 2 == 1
        node.map_goal_and_los = lambda: (pose, raw_clear)
        node.los_clear_wide = raw_clear
        node.now = node.target_time = node.scan_time = index*0.05
        node.tick()
        assert node.state.publish.call_args.args[0].data.startswith('FOLLOWING:')
        assert node.publisher.publish.call_args.args[0].linear.x > 0
        assert node.supervisor.mode == 'DIRECT' and not node.busy()
    node.client.send_goal_async.assert_not_called()


def test_separate_nav_observation_keeps_external_zero_after_own_publish():
    node = make_node()
    node.observed_cmd_vel_topic = '/cmd_vel/nav'
    node.drain.start(0)
    node.last_own_publish = node.now = 0.1
    node.on_cmd_vel(NS(linear=NS(x=0.0), angular=NS(z=0.0)))
    assert node.drain.zero and node.drain.last_message == 0.1


def test_mux_disabled_controller_does_not_mask_navigation_with_idle_zeros():
    node = make_node()
    node.observed_cmd_vel_topic = '/cmd_vel/nav'
    node.set_enabled(False)
    node.publisher.reset_mock()
    node.now = node.target_time = node.scan_time = 0.1
    node.tick()
    node.publisher.publish.assert_not_called()


def test_disable_ack_waits_for_late_navigation_acceptance_and_terminal_result():
    node = make_node()
    node.goal_pending = True
    response = node.enable(NS(data=False), NS())
    assert not response.success and not node.enabled
    handle = Mock(accepted=True)
    node.goal_response(completed(handle), node.goals.generation)
    handle.cancel_goal_async.assert_called_once()
    assert not node.enable(NS(data=False), NS()).success
    node.goal_result(completed(NS(status=5)), node.goals.generation)
    assert node.enable(NS(data=False), NS()).success


@pytest.mark.parametrize('nav', [False, True])
@pytest.mark.parametrize('stamp', [None, -1.0, 1.0])
def test_opt_in_odom_watchdog_stops_and_cancels(nav, stamp):
    node = make_node()
    node.monitor_odom = True
    node.odom_time = stamp
    if nav:
        node.supervisor.mode = 'NAV2'
        node.goal_pending = True
    node.tick()
    cmd = node.publisher.publish.call_args.args[0]
    assert cmd.linear.x == cmd.angular.z == 0
    if nav:
        assert node.cancel_requested


def test_odom_watchdog_recovery_uses_fresh_header():
    node = make_node()
    node.monitor_odom = True
    node.odom_time = -1
    node.tick()
    node.on_odom(NS(header=NS(stamp=NS(sec=0, nanosec=0)),
                    pose=NS(pose=NS(position=NS(x=0, y=0),
                                    orientation=NS(x=0, y=0, z=0, w=1)))))
    node.obstacle = 3
    node.map_goal_and_los = lambda: (None, True)
    node.now = 0.05
    node.tick()
    assert node.publisher.publish.call_args.args[0].linear.x > 0


@pytest.mark.parametrize('fault', ['stale_target', 'stale_scan'])
@pytest.mark.parametrize('nav', [False, True])
def test_frozen_measurement_headers_stop_despite_recent_delivery(fault, nav):
    node = make_node()
    node.now = 0.55
    node.last_time = 0.5
    node.target_time = 0.0 if fault == 'stale_target' else 0.55
    node.scan_time = 0.0 if fault == 'stale_scan' else 0.55
    if nav:
        node.supervisor.mode = 'NAV2'
        node.goal_pending = True
    node.tick()
    cmd = node.publisher.publish.call_args.args[0]
    assert cmd.linear.x == cmd.angular.z == 0
    if nav:
        assert node.cancel_requested


def test_target_velocity_uses_fresh_odometry_and_drops_stale_history():
    from shopping_uwb_follow.estimator import TargetVelocityEstimator
    node = make_node()
    node.velocity_estimator = TargetVelocityEstimator()
    node.odom_pose = (0, 0, 0)
    node.odom_time = 0
    for i in range(20):
        t = i*0.1
        node.odom_time = t
        node.odom_pose = (0.2*t, 0, 0)
        node.on_target(NS(header=NS(frame_id='base_link',
                                   stamp=NS(sec=int(t), nanosec=round((t-int(t))*1e9))),
                          point=NS(x=2+0.15*t, y=0)))
    assert node.target_velocity[0] == pytest.approx(0.35, abs=0.006)
    node.on_target(NS(header=NS(frame_id='base_link', stamp=NS(sec=3, nanosec=0)),
                      point=NS(x=3, y=0)))
    assert node.target_velocity == (0, 0)
    assert node.velocity_estimator.previous is None
    node.odom_time = 3.1
    node.on_target(NS(header=NS(frame_id='base_link', stamp=NS(sec=3, nanosec=100000000)),
                      point=NS(x=3.1, y=0)))
    assert node.target_velocity == (0, 0)  # First fresh sample reinitializes.
