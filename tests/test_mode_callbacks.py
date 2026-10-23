"""Execute safety-critical callback bodies using fake Humble messages/services."""
import ast
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

from shopping_zone_manager.mode_control import ModeManager, VelocityMux, VelocityLimiter, OutputWatchdog

SRC = Path(__file__).resolve().parents[1]/'ros2_ws/src/shopping_zone_manager/shopping_zone_manager'


def twist():
    return NS(linear=NS(x=0.0, y=0.0, z=0.0), angular=NS(x=0.0, y=0.0, z=0.0))


def make_class(filename, name):
    tree = ast.parse((SRC/filename).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    cls.bases = []
    clock = NS(now=0.0)
    env = dict(time=NS(monotonic=lambda: clock.now), Twist=twist,
               String=lambda **kw: NS(**kw), Bool=lambda **kw: NS(**kw),
               SetBool=NS(Request=lambda **kw: NS(**kw)), ZERO=(0.0, 0.0),
               CancelGoal=NS(Request=lambda: NS(), Response=NS(ERROR_NONE=0)))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), filename, 'exec'), env)
    return env[name], clock


def goal_info(identifier):
    return NS(goal_id=NS(uuid=[identifier]+[0]*15))


def make_mux_node():
    cls, clock = make_class('mode_manager_node.py', 'ModeMuxNode')
    node = cls.__new__(cls)
    node.manager, node.mux, node.limiter = ModeManager(), VelocityMux(), VelocityLimiter()
    node.manage_follow = True
    node.output, node.active, node.state, node.follow_enable = (Mock() for _ in range(4))
    node.follow_client, node.cancel_client = Mock(), Mock()
    node.follow_client.service_is_ready.return_value = False
    node.cancel_client.service_is_ready.return_value = False
    node.follow_future = node.cancel_future = None
    node.follow_disabled = node.cancel_done = False
    node.nav_active = False
    node.nav_statuses = {}
    node.cancel_wait = set()
    node.last_nav_motion = -float('inf')
    node.generation = 0
    node.get_logger = Mock()
    return node, clock


def assert_zero(node):
    cmd = node.output.publish.call_args.args[0]
    assert cmd.linear.x == cmd.angular.z == 0.0


def test_transition_waits_for_disable_cancel_terminal_status_and_nav_drain():
    node, clock = make_mux_node()
    node.request(NS(data='follow'))
    assert_zero(node)
    node.tick()
    node.tick()
    assert node.manager.state == 'IDLE'
    disable = Future()
    node.follow_client.service_is_ready.return_value = True
    node.follow_client.call_async.return_value = disable
    node.tick()
    assert node.follow_client.call_async.call_args.args[0].data is False
    assert node.cancel_client.call_async.call_count == 0
    disable.set_result(NS(success=True))
    cancel = Future()
    node.cancel_client.service_is_ready.return_value = True
    node.cancel_client.call_async.return_value = cancel
    node.tick()
    node.status(NS(status_list=[NS(status=3, goal_info=goal_info(1))]))
    cancel.set_result(NS(return_code=0, goals_canceling=[goal_info(1)]))
    node.tick()
    assert node.manager.state == 'IDLE'
    moving = twist()
    moving.linear.x = 0.4
    node.nav(moving)
    node.status(NS(status_list=[NS(status=5, goal_info=goal_info(1))]))
    clock.now = 0.54
    node.tick()
    assert node.manager.state == 'IDLE'
    clock.now = 0.56
    node.tick()
    assert node.manager.state == 'FOLLOW'
    assert node.follow_enable.publish.call_args.args[0].data is True
    assert_zero(node)  # Commands received during the transition were discarded.


def test_estop_immediate_and_old_async_responses_cannot_unlock_transition():
    node, clock = make_mux_node()
    node.request(NS(data='guide'))
    old_generation = node.generation
    node.manager.step()
    node.manager.step()
    moving = twist()
    moving.linear.x = 0.4
    node.nav(moving)
    node.safety(NS(data=True))
    assert_zero(node)
    assert node.manager.state == 'ESTOP'
    future = Future()
    future.set_result(NS(success=True))
    node.disable_complete(future, old_generation)
    assert not node.follow_disabled
    node.request(NS(data='guide'))
    node.safety(NS(data=False))
    node.tick()
    assert node.manager.state == 'ESTOP'
    node.request(NS(data='reset'))
    assert node.manager.state == 'IDLE'
    assert node.manager.pending == 'IDLE'
    assert not node.mux.inputs


def test_cancel_failure_and_disable_failure_hold_zero():
    node, clock = make_mux_node()
    node.request(NS(data='guide'))
    failed = Future()
    failed.set_exception(RuntimeError('service unavailable'))
    node.disable_complete(failed, node.generation)
    assert not node.follow_disabled
    rejected = Future()
    rejected.set_result(NS(return_code=1))
    node.cancel_complete(rejected, node.generation)
    assert not node.cancel_done
    for _ in range(4):
        node.tick()
        assert_zero(node)
    assert node.manager.state == 'IDLE'


def test_watchdog_emergency_latch_survives_commands_until_explicit_reset():
    cls, clock = make_class('output_watchdog_node.py', 'WatchdogNode')
    node = cls.__new__(cls)
    node.watchdog, node.publisher, node.latched = OutputWatchdog(), Mock(), False
    moving = twist()
    moving.linear.x = 0.4
    node.receive(moving)
    node.safety(NS(data=True))
    node.receive(moving)
    node.safety(NS(data=False))
    node.tick()
    assert node.publisher.publish.call_args.args[0].linear.x == 0
    node.request(NS(data='reset'))
    node.tick()
    assert node.publisher.publish.call_args.args[0].linear.x == 0
    node.receive(moving)
    node.tick()
    assert node.publisher.publish.call_args.args[0].linear.x == 0.4
    clock.now = 0.25
    node.tick()
    assert node.publisher.publish.call_args.args[0].linear.x == 0


def test_teleop_from_idle_needs_no_navigation_server():
    node, clock = make_mux_node()
    node.manage_follow = False
    node.request(NS(data='teleop'))
    node.tick()
    node.tick()
    assert node.manager.state == 'TELEOP'
    node.cancel_client.call_async.assert_not_called()


def test_rapid_reset_does_not_erase_outstanding_navigation_cancel():
    node, clock = make_mux_node()
    node.manage_follow = False
    node.request(NS(data='guide'))
    node.request(NS(data='estop'))
    node.request(NS(data='reset'))
    node.request(NS(data='teleop'))
    for _ in range(3):
        node.tick()
    assert node.cancel_required and node.manager.state == 'IDLE'


def test_cancel_ack_requires_matching_terminal_status_even_if_status_list_is_empty():
    node, clock = make_mux_node()
    node.manage_follow = False
    node.request(NS(data='guide'))
    ack = Future()
    ack.set_result(NS(return_code=0, goals_canceling=[goal_info(7)]))
    node.cancel_complete(ack, node.generation)
    node.status(NS(status_list=[]))
    clock.now = 2
    node.tick()
    node.tick()
    assert node.manager.state == 'IDLE'
    assert node.cancel_wait
    node.status(NS(status_list=[NS(status=5, goal_info=goal_info(7))]))
    node.tick()
    assert node.manager.state == 'GUIDE'
