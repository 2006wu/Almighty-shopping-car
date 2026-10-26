"""Keep model updates ordered, nonblocking and based on the newest shared-path time."""
import ast
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

from shopping_uwb_follow.person_path import path_position


def make_node():
    path = Path(__file__).parents[1]/'shopping_uwb_follow/person_path_node.py'
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    scope = dict(Node=object, Entity=NS(MODEL=2), path_position=path_position,
                 seconds=lambda node: node.now,
                 SetEntityPose=NS(Request=lambda: NS(entity=NS(),
                     pose=NS(position=NS(), orientation=NS()))))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), scope)
    node = scope['PersonPath'].__new__(scope['PersonPath'])
    node.path = dict(speed=1.0, waypoints=[[0, 0], [10, 0]])
    node.now = 1.0
    node.pending = None
    node.client = Mock()
    node.client.service_is_ready.return_value = True
    node.client.call_async.side_effect = lambda request: Future()
    node.logger = Mock()
    node.get_logger = lambda: node.logger
    return node


def test_pose_requests_use_shared_path_and_model_identity():
    node = make_node()
    node.tick()
    request = node.client.call_async.call_args.args[0]
    assert request.entity.name == 'person' and request.entity.type == 2
    assert (request.pose.position.x, request.pose.position.y, request.pose.position.z) == (1., 0., .85)
    assert request.pose.orientation.w == 1.


def test_slow_service_never_queues_or_blocks_then_uses_latest_position():
    node = make_node()
    node.tick()
    pending = node.pending
    node.now = 4.0
    for _ in range(20):
        node.tick()
    assert node.pending is pending and node.client.call_async.call_count == 1
    pending.set_result(NS(success=True))
    node.tick()
    assert node.client.call_async.call_count == 2
    assert node.client.call_async.call_args.args[0].pose.position.x == 4.


def test_missing_service_does_not_send_requests():
    node = make_node()
    node.client.service_is_ready.return_value = False
    node.tick()
    node.client.call_async.assert_not_called()


def test_failed_service_reports_error_and_retries_latest_pose():
    node = make_node()
    node.tick()
    node.pending.set_exception(RuntimeError('transport unavailable'))
    node.now = 5.0
    node.tick()
    node.logger.warning.assert_called_once()
    assert node.client.call_async.call_args.args[0].pose.position.x == 5.
