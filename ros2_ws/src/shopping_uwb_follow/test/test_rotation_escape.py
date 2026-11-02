"""Bounded forward escape behind a shelf corner, without ROS transport."""
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from shopping_uwb_follow.follow_controller import rotation_escape_ok, rotation_scan_clear
from shopping_uwb_follow.follow_handoff import scan_valid
from test_follow_handoff_callbacks import make_node


RADIUS = 0.4905


def corner_scan(base=(0, 0, 0), bearing=-1.83, distance=0.49, reverse=False):
    # Place an exact return in base coordinates, then express it in laser axes.
    x, y, yaw = base
    px = x+distance*math.cos(bearing+yaw)
    py = y+distance*math.sin(bearing+yaw)
    increment = (-1 if reverse else 1)*2*math.pi/720
    angle_min = math.atan2(py, px)-360*increment
    ranges = [3.0]*720
    ranges[360] = math.hypot(px, py)
    return NS(ranges=ranges, angle_min=angle_min, angle_increment=increment,
              range_min=0.05, range_max=12.0, header=NS(frame_id='laser', stamp=0))


def scan_args(scan, base=(0, 0, 0)):
    return (scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, base, RADIUS)


@pytest.mark.parametrize('base', [(0, 0, 0), (-0.18, 0, 0),
                                  (0, -0.18, math.pi/2), (0.1, 0.12, -math.pi/2),
                                  (0.1, 0, math.pi)])
@pytest.mark.parametrize('reverse', [False, True])
def test_only_rear_corner_inside_guard_disk_allows_escape(base, reverse):
    scan = corner_scan(base, reverse=reverse)
    assert not rotation_scan_clear(*scan_args(scan, base))
    assert rotation_escape_ok(*scan_args(scan, base), 3.0, 1.15)


@pytest.mark.parametrize('bearing', [-math.pi/3, 0, math.pi/3])
@pytest.mark.parametrize('base', [(0, 0, 0), (-0.18, 0, math.pi)])
def test_front_blocker_forbids_escape_even_when_another_blocker_is_behind(bearing, base):
    scan = corner_scan(base, bearing=bearing)
    scan.ranges[0] = 0.1
    assert not rotation_escape_ok(*scan_args(scan, base), 3.0, 1.15)


def test_clear_rotation_disk_does_not_request_escape():
    scan = corner_scan(distance=0.6)
    assert rotation_scan_clear(*scan_args(scan))
    assert not rotation_escape_ok(*scan_args(scan), 3.0, 1.15)


@pytest.mark.parametrize('front', [0.65, 1.0, 1.15, math.nan, math.inf, -math.inf])
def test_escape_requires_finite_raw_front_clearance_strictly_above_minimum(front):
    assert not rotation_escape_ok(*scan_args(corner_scan()), front, 1.15)


@pytest.mark.parametrize('fault', ['partial', 'nan', 'negative', 'subminimum', 'overrange',
                                  'zero_increment', 'coarse', 'short_range', 'outside_origin',
                                  'invalid_pose', 'invalid_radius'])
def test_escape_fails_closed_for_unknown_scan_geometry(fault):
    scan = corner_scan()
    base, radius = (0, 0, 0), RADIUS
    if fault == 'partial':
        scan.ranges = scan.ranges[:400]
    elif fault in ('nan', 'negative', 'subminimum', 'overrange'):
        scan.ranges[0] = dict(nan=math.nan, negative=-1, subminimum=0.01, overrange=13)[fault]
    elif fault == 'zero_increment':
        scan.angle_increment = 0
    elif fault == 'coarse':
        scan.angle_increment = 0.1
    elif fault == 'short_range':
        scan.ranges = [math.inf]*720
        scan.range_max = 0.4
    elif fault == 'outside_origin':
        base = (0.5, 0, 0)
    elif fault == 'invalid_pose':
        base = (0, 0, math.nan)
    elif fault == 'invalid_radius':
        radius = math.nan
    args = (*scan_args(scan, base)[:-1], radius)
    assert not rotation_escape_ok(*args, 3.0, 1.15)


def escape_node(mode='DIRECT'):
    node = make_node()
    node.rotation_escape_enabled = True
    node.guard_rotation_radius = RADIUS
    node.rotation_escape_speed = 0.15
    node.rotation_escape_max_distance = 0.5
    node.robot_half_width = 0.4
    node.scan = corner_scan()
    node.obstacle = 3.0
    node.target = (-3, -1)
    node.target_velocity = lambda now: (0, 0)
    node.last_time = -0.1
    node.tf_buffer.lookup_transform.side_effect = None
    node.tf_buffer.lookup_transform.return_value = NS(pose=(0, 0, 0))
    node.update_rotation_clearance = type(node).update_rotation_clearance.__get__(node)
    node.tick.__globals__.update(rotation_escape_ok=rotation_escape_ok, scan_valid=scan_valid)
    node.supervisor.route_around = False
    if mode == 'DIRECT':
        node.client = None
    else:
        pose, _ = node.map_goal_and_los()

        def map_and_los():
            node.update_rotation_clearance()
            return pose, True

        node.map_goal_and_los = map_and_los
        node.nav_status = 'active'
        node.goal_handle = Mock()
        node.supervisor.mode = 'NAV2'
        node.supervisor.step = Mock(return_value=NS(
            mode='NAV2', action='none', direct_publish=False, blocked_wait=False, reason='Tracking'))
    return node


def tick(node, now, guard='CLEAR'):
    node.now = node.target_time = node.scan_time = now
    node.guard_time, node.guard_state = now, guard
    node.last_nav_command = (now, 0, -0.6)
    node.tick()
    cmd = node.publisher.publish.call_args.args[0]
    return cmd.linear.x, cmd.angular.z


@pytest.mark.parametrize('mode', ['DIRECT', 'NAV2'])
def test_escape_publishes_accelerated_forward_speed_and_stops_at_budget(mode):
    node = escape_node(mode)
    speeds = []
    for i in range(80):
        v, w = tick(node, i*0.1)
        speeds.append(v)
        assert w == 0
        assert 0 <= v <= 0.15
        if v > 0:
            assert node.state.publish.call_args.args[0].data == (
                'ROTATION_ESCAPE: Rotation disk blocked behind; creeping forward')
    assert speeds[0] == pytest.approx(0.08)
    assert speeds[1] == pytest.approx(0.15)
    assert sum(speeds)*0.1 == pytest.approx(0.5)
    assert speeds[-1] == 0 and node.escape_exhausted
    assert node.escape_distance == pytest.approx(0.5)
    if mode == 'NAV2':
        assert node.nav_status == 'active'
        node.goal_handle.cancel_goal_async.assert_not_called()
        node.client.send_goal_async.assert_not_called()


@pytest.mark.parametrize('mode', ['DIRECT', 'NAV2'])
@pytest.mark.parametrize('guard', ['STOP', 'STALE'])
def test_guard_stop_during_forward_escape_ends_and_latches(mode, guard):
    node = escape_node(mode)
    assert tick(node, 0)[0] > 0
    if guard == 'STOP':
        # Within the grace the STOP is still the guard's rotation STOP (clear_time
        # hysteresis and latency); the downstream guard keeps the base stopped.
        assert tick(node, 0.1, guard)[0] > 0
        assert tick(node, 1.2, guard) == (0, 0)
    else:
        assert tick(node, 0.1, guard) == (0, 0)
    assert node.escape_exhausted and not node.escape_active
    assert tick(node, 1.3) == (0, 0)
    assert tick(node, 1.4, 'STOP') == (0, 0)
    # A clear disk releases the latch; a new rear obstruction can escape again.
    node.scan.ranges = [3.0]*720
    tick(node, 1.5)
    assert not node.escape_exhausted and node.escape_distance == 0
    node.scan = corner_scan()
    assert tick(node, 1.6)[0] > 0


@pytest.mark.parametrize('mode', ['DIRECT', 'NAV2'])
def test_stop_for_previous_rotation_can_begin_escape_but_not_override_persistent_stop(mode):
    node = escape_node(mode)
    node.rotation_escape_command((0, -0.6), (0, -0.6), False, mode, 0, True, 3, 0.1, False)
    assert not node.escape_active and node.last_rotation_requested
    assert tick(node, 0, 'STOP')[0] > 0
    assert tick(node, 0.1, 'STOP')[0] > 0
    assert tick(node, 1.2, 'STOP') == (0, 0)


@pytest.mark.parametrize('fault', ['disabled', 'lost', 'near', 'front', 'partial', 'tf_missing',
                                  'feature_disabled', 'front_blocker', 'stale_scan'])
@pytest.mark.parametrize('mode', ['DIRECT', 'NAV2'])
def test_safety_conditions_never_publish_forward_escape(fault, mode):
    node = escape_node(mode)
    if fault == 'disabled':
        node.enabled = False
    elif fault == 'lost':
        node.target = None
    elif fault == 'near':
        node.target = (0.1, 0)
    elif fault == 'front':
        node.obstacle = 1.0
    elif fault == 'partial':
        node.scan.ranges = node.scan.ranges[:400]
    elif fault == 'tf_missing':
        node.tf_buffer.lookup_transform.side_effect = RuntimeError('Missing TF')
    elif fault == 'feature_disabled':
        node.rotation_escape_enabled = False
    elif fault == 'front_blocker':
        node.scan = corner_scan(bearing=0)
    elif fault == 'stale_scan':
        node.now = 1.0
    node.guard_state, node.guard_time = 'CLEAR', node.now
    node.last_nav_command = (node.now, 0, -0.6)
    node.tick()
    cmd = node.publisher.publish.call_args.args[0]
    assert cmd.linear.x == 0
    assert not node.state.publish.call_args.args[0].data.startswith('ROTATION_ESCAPE:')


@pytest.mark.parametrize('event', ['aligned', 'lost', 'mode', 'enable'])
def test_episode_resets_without_releasing_exhausted_budget(event):
    node = escape_node()
    assert tick(node, 0)[0] > 0
    node.escape_exhausted = True
    if event == 'aligned':
        node.target = (3, 0)
    elif event == 'lost':
        node.target = None
    elif event == 'mode':
        node.escape_context = ('NAV2', True, True)
    elif event == 'enable':
        node.set_enabled(False)
    tick(node, 0.1)
    assert node.escape_distance == 0
    assert node.escape_exhausted


def test_escape_obeys_configured_linear_cap():
    node = escape_node()
    node.control.p.max_linear = 0.05
    assert tick(node, 0) == (0.05, 0)


def wall_scan(offset, slope, laser_x=0.18):
    """Straight wall right of the cart, y = -(offset+slope*x) in base coordinates."""
    increment = 2*math.pi/720
    ranges = [3.0]*720
    for i in range(720):
        angle = -math.pi+i*increment
        c, s = math.cos(angle), math.sin(angle)
        # Ray from the laser (laser_x, 0): solve laser_x+t*c = x, t*s = -(offset+slope*x).
        denominator = s+slope*c
        if abs(denominator) < 1e-9:
            continue
        t = -(offset+slope*laser_x)/denominator
        if 0 < t < ranges[i] and -1.0 <= laser_x+t*c <= 1.5:
            ranges[i] = t
    return NS(ranges=ranges, angle_min=-math.pi, angle_increment=increment,
              range_min=0.05, range_max=12.0, header=NS(frame_id='laser', stamp=0))


def test_t99_diverging_shelf_beside_cart_allows_escape():
    # Gazebo t99: cart about 15 deg off a shelf on its right, 0.45-0.48 m from
    # base_link near x=0 and diverging ahead; guard disk 0.4905 m blocks turns.
    scan = wall_scan(0.45, 0.25)
    base = (-0.18, 0, 0)
    assert not rotation_scan_clear(*scan_args(scan, base))
    assert rotation_escape_ok(*scan_args(scan, base), 3.0, 1.15)


def test_parallel_shelf_beside_cart_does_not_escape():
    scan = wall_scan(0.45, 0.0)
    assert not rotation_escape_ok(*scan_args(scan, (-0.18, 0, 0)), 3.0, 1.15)


def test_converging_shelf_ahead_does_not_escape():
    scan = wall_scan(0.45, -0.25)
    assert not rotation_escape_ok(*scan_args(scan, (-0.18, 0, 0)), 3.0, 1.15)


@pytest.mark.parametrize('mode', ['DIRECT', 'NAV2'])
def test_budget_uses_odom_travel_when_guard_slows_creep(mode):
    node = escape_node(mode)
    position = [0.0]
    node.escape_odom_xy = lambda: (position[0], 0.0)
    # Guard-scaled creep: 0.01 m per tick although 0.15 m/s is commanded.
    for i in range(30):
        assert tick(node, i*0.1)[0] > 0
        position[0] += 0.01
    assert node.escape_distance == pytest.approx(0.29)
    assert not getattr(node, 'escape_exhausted', False)


@pytest.mark.parametrize('mode', ['DIRECT', 'NAV2'])
def test_escape_times_out_without_progress(mode):
    node = escape_node(mode)
    node.escape_odom_xy = lambda: (0.0, 0.0)
    assert tick(node, 0)[0] > 0
    assert tick(node, 6.0)[0] > 0
    tick(node, 12.5)
    assert node.escape_exhausted
    assert tick(node, 12.6) == (0, 0)
