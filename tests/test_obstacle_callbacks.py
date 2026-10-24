"""Run scan-guard callback bodies with fake clocks, TF and messages."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

from shopping_zone_manager.mode_control import OutputWatchdog
from shopping_zone_manager.obstacle_guard import ObstacleGuard, scan_points, scan_fresh


def test_guard_actual_callbacks_expire_commands_scan_headers_and_tf():
    source = Path(__file__).resolve().parents[1]/(
        'ros2_ws/src/shopping_zone_manager/shopping_zone_manager/obstacle_guard_node.py')
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    cls.bases = []
    clock = NS(now=10.)
    def twist():
        return NS(linear=NS(x=0.), angular=NS(z=0.))
    env = dict(time=NS(monotonic=lambda: clock.now), math=math, json=json,
               Twist=twist, String=lambda **kwargs: NS(**kwargs),
               Time=NS(from_msg=lambda stamp: stamp), TransformException=LookupError,
               scan_points=scan_points, scan_fresh=scan_fresh)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), env)
    node = env[cls.name].__new__(env[cls.name])
    node.guard, node.command = ObstacleGuard(), OutputWatchdog()
    node.timeout, node.scan, node.received = .5, None, None
    node.publisher, node.state = Mock(), Mock()
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(clock.now*1e9)))
    node.buffer = Mock()
    node.buffer.lookup_transform.return_value = NS(transform=NS(
        translation=NS(x=.18, y=0), rotation=NS(x=0, y=0, z=0, w=1)))
    def output():
        msg = node.publisher.publish.call_args.args[0]
        return msg.linear.x, msg.angular.z
    node.tick()
    assert output() == (0, 0)
    def refresh():
        node.on_scan(NS(header=NS(stamp=NS(sec=int(clock.now),
            nanosec=int((clock.now-int(clock.now))*1e9)), frame_id='laser_link'),
            ranges=[math.inf]*720, angle_min=-math.pi,
            angle_increment=2*math.pi/720, range_min=.12, range_max=12))
        msg = twist()
        msg.linear.x = .4
        node.receive(msg)
    refresh()
    node.tick()
    clock.now += .31
    refresh()
    node.tick()
    assert output() == (.4, 0)
    clock.now += .26
    node.tick()
    assert output() == (0, 0)  # Input mux died while the scan is still fresh.
    clock.now += .3
    node.on_scan(node.scan)  # Repeated messages must not refresh acquisition time.
    node.tick()
    assert output() == (0, 0)
    assert json.loads(node.state.publish.call_args.args[0].data)['state'] == 'STALE'
    refresh()
    node.buffer.lookup_transform.side_effect = LookupError('missing TF')
    node.tick()
    assert output() == (0, 0)
