"""Safety and geometry regressions without ROS."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from shopping_uwb_follow.follow_handoff import (
    GoalBook, VelocityDrain, HandoffParams, HandoffSupervisor, line_of_sight_clear, transform_xy,
)


def los(ranges, **kwargs):
    params = dict(angle_min=-0.5, angle_increment=0.5, target_xy=(3, 0),
                  robot_half_width=0.25, margin=0.15, range_min=0.12, range_max=12.0)
    params.update(kwargs)
    return line_of_sight_clear(ranges, **params)


@pytest.mark.parametrize('ranges,clear', [([4, 4, 4], True), ([math.inf]*3, True),
    ([4, 1, 4], False), ([4, 0.01, 4], False), ([4, math.nan, 4], False),
    ([math.nan, 4, 4], False), ([], False), ([4, -1, 4], False),
    ([4, -math.inf, 4], False), ([4, 13, 4], False)])
def test_los_readings(ranges, clear):
    assert los(ranges) is clear


@pytest.mark.parametrize('kwargs', [dict(angle_increment=0), dict(angle_min=math.nan),
    dict(range_max=0.1), dict(range_min=-1), dict(target_xy=None),
    dict(target_xy=(math.nan, 0)), dict(target_xy=(0, 0)),
    dict(robot_half_width=0), dict(margin=math.inf)])
def test_los_invalid_metadata(kwargs):
    assert not los([4]*3, **kwargs)


def test_los_bearing_resolution_wrap_and_strip():
    assert not los([0.5, 4], angle_min=-0.01, angle_increment=0.1,
                   robot_half_width=0.001)
    assert los([math.inf]*3, angle_min=math.pi-0.1, angle_increment=0.1,
               target_xy=(-3, 0))
    assert not los([math.inf]*3, angle_min=1.0, angle_increment=0.1)
    # A close off-bearing return inside the strip must stop the handoff.
    assert not los([0.3, 4, 4])
    assert los([1, 4, 4])  # Outside the swept width.


def tick(supervisor, now, **kwargs):
    inputs = dict(state='OBSTACLE_STOP', target=(3, 0), los_clear=False,
                  front_clearance=0.4, map_target=(3, 5))
    inputs.update(kwargs)
    decision = supervisor.step(now, **inputs)
    assert decision.direct_publish == (decision.mode == 'DIRECT')
    return decision


def navigating():
    supervisor = HandoffSupervisor()
    tick(supervisor, 0)
    assert tick(supervisor, 1).action == 'send_goal'
    return supervisor


def test_continuous_blocking_delay_and_ownership():
    s = HandoffSupervisor(HandoffParams(los_block_time=0, los_clear_time=0))
    assert tick(s, 0).mode == 'DIRECT'
    assert tick(s, 0.9).action == 'none'
    tick(s, 0.95, state='FOLLOWING', los_clear=True, front_clearance=2)
    assert tick(s, 1).mode == 'DIRECT'
    assert tick(s, 1.99).mode == 'DIRECT'
    d = tick(s, 2)
    assert d.mode == 'NAV2' and d.action == 'send_goal' and not d.direct_publish
    assert tick(s, 2.1, nav_status='active').action == 'none'


def test_blocked_los_alone_triggers_navigation():
    s = HandoffSupervisor()
    tick(s, 0, state='FOLLOWING', front_clearance=2)
    tick(s, 0.3, state='FOLLOWING', front_clearance=2)
    assert tick(s, 1.3, state='FOLLOWING', front_clearance=2).mode == 'NAV2'


@pytest.mark.parametrize('kwargs', [dict(target=(1.3, 0)), dict(state='LOST'),
    dict(state='TOO_CLOSE'), dict(state='IDLE'), dict(enabled=False),
    dict(scan_valid=False), dict(target=None), dict(target=(math.nan, 0)),
    dict(map_target=None), dict(front_clearance=math.nan)])
def test_no_unsafe_handoff(kwargs):
    s = HandoffSupervisor()
    tick(s, 0, **kwargs)
    assert tick(s, 10, **kwargs).mode == 'DIRECT'


@pytest.mark.parametrize('kwargs', [dict(state='LOST'), dict(state='TOO_CLOSE'),
    dict(enabled=False), dict(scan_valid=False), dict(target=None),
    dict(front_clearance=math.nan), dict(map_target=None)])
def test_immediate_cancel(kwargs):
    d = tick(navigating(), 1.01, **kwargs)
    assert d.mode == 'DIRECT' and d.action == 'cancel_goal' and d.direct_publish


def test_clear_resume_delay_resets_with_obstruction():
    s = navigating()
    clear = dict(los_clear=True, front_clearance=1.0, target=(1.3, 0))
    assert tick(s, 2, **clear).mode == 'NAV2'
    assert tick(s, 2.49, **clear).mode == 'NAV2'
    tick(s, 2.5)
    assert tick(s, 3, **clear).mode == 'NAV2'
    assert tick(s, 3.5, **clear).mode == 'NAV2'
    tick(s, 3.6, **clear)
    assert tick(s, 4.11, **clear).action == 'cancel_goal'


def test_clearance_threshold_and_sustained_close_resume():
    s = navigating()
    tick(s, 2, los_clear=True, front_clearance=0.8)
    assert tick(s, 3, los_clear=True, front_clearance=0.8).mode == 'NAV2'
    assert tick(s, 3.1, los_clear=True, target=(1.3, 0), front_clearance=1).mode == 'NAV2'
    assert tick(s, 3.61, los_clear=True, target=(1.3, 0), front_clearance=1).action == 'cancel_goal'
    assert tick(navigating(), 2, nav_status='succeeded').action == 'update_goal'


def test_goal_updates_use_map_displacement_and_rate_limit():
    s = navigating()
    s.p.goal_update_distance, s.p.goal_update_period = 0.5, 1.0
    assert tick(s, 1.5, map_target=(4, 5)).action == 'none'
    assert tick(s, 2, map_target=(3.5, 5)).action == 'none'
    assert tick(s, 2.1, map_target=(3.51, 5)).action == 'update_goal'
    assert tick(s, 2.9, map_target=(6, 5)).action == 'none'
    assert tick(s, 3.1, map_target=(6, 5)).action == 'update_goal'


@pytest.mark.parametrize('status', ['failed', 'canceled'])
def test_failure_cooldown(status):
    s = navigating()
    assert tick(s, 2, nav_status=status).action == 'cancel_goal'
    assert tick(s, 4.99).mode == 'DIRECT'
    assert tick(s, 5).mode == 'DIRECT'
    assert tick(s, 6).action == 'send_goal'


@pytest.mark.parametrize('key', vars(HandoffParams()))
@pytest.mark.parametrize('value', [0, -1, math.nan, math.inf])
def test_parameter_validation(key, value):
    p = HandoffParams()
    setattr(p, key, value)
    if value == 0 and key in ('los_block_time', 'los_clear_time', 'los_clear_extra_width',
                              'min_nav_hold', 'min_direct_hold'):
        HandoffSupervisor(p)
    else:
        with pytest.raises(ValueError):
            HandoffSupervisor(p)


def test_direct_only_and_clock_reset():
    s = HandoffSupervisor(route_around=False)
    for now in (0, 1, 100):
        assert tick(s, now).mode == 'DIRECT'
    s = navigating()
    assert tick(s, 0).action == 'cancel_goal'
    with pytest.raises(ValueError):
        HandoffSupervisor(route_around='false')


def shelf_scan(robot_y):
    """Ray-box intersections in world coordinates, robot facing +y."""
    world = ET.parse(Path(__file__).resolve().parents[2]/
                     'shopping_robot_gazebo/worlds/supermarket.sdf')
    boxes = []
    for model in world.findall('.//world/model'):
        if not model.get('name').startswith('shelf_'):
            continue
        pose = list(map(float, model.findtext('pose').split()))
        size = list(map(float, model.findtext('link/collision/geometry/box/size').split()))
        boxes.append([(pose[i]-size[i]/2, pose[i]+size[i]/2) for i in (0, 1)])
    ranges = []
    increment = math.pi/360
    for i in range(721):
        angle = -math.pi+i*increment+math.pi/2
        direction = (math.cos(angle), math.sin(angle))
        best = math.inf
        for box in boxes:
            near, far = 0.0, math.inf
            for origin, delta, (low, high) in zip((2.4, robot_y), direction, box):
                if abs(delta) < 1e-12:
                    if not low <= origin <= high:
                        far = -1
                        break
                else:
                    a, b = sorted(((low-origin)/delta, (high-origin)/delta))
                    near, far = max(near, a), min(far, b)
            if far >= near:
                best = min(best, near)
        ranges.append(best)
    return ranges, -math.pi, increment


def test_store_shelf_end_geometry():
    assert transform_xy((1.4, 1.4), (2.4, 4.5, math.pi/2)) == pytest.approx((1.0, 5.9))
    # SDF shelves end at y=5: use y=4.5 to intersect the robot-width corridor.
    for robot_y, expected in ((4.5, False), (5.0, True)):
        ranges, start, increment = shelf_scan(robot_y)
        target = (5.9-robot_y, 2.4-1.0)
        assert line_of_sight_clear(ranges, start, increment, target,
                                   0.25, 0.15, 0.12, 12) is expected


def test_person_surface_is_excluded_by_configured_endpoint_margin():
    import yaml
    config = Path(__file__).resolve().parents[1]/'config/uwb_follow.yaml'
    margin = yaml.safe_load(config.read_text())['/**']['ros__parameters']['los_margin']
    assert los([math.inf, 3-0.25, math.inf], margin=margin)
    assert not los([math.inf, 1.0, math.inf], margin=margin)


def test_goal_book_supersedes_pending_or_active_generation():
    book = GoalBook()
    old = book.begin()
    current = book.begin()
    assert not book.is_current(old) and book.is_current(current)


def test_default_preemption_thresholds():
    s = navigating()
    assert tick(s, 1.49, map_target=(3.41, 5)).action == 'none'
    assert tick(s, 1.5, map_target=(3.41, 5)).action == 'update_goal'


def test_drain_requires_terminal_even_past_cap():
    d = VelocityDrain()
    d.start(0)
    d.observe(0, 0, 0)
    assert not d.ready(20)
    d.terminal(20)
    assert d.ready(20)


def test_drain_quiet_period_and_latest_zero():
    d = VelocityDrain()
    d.start(0)
    d.observe(0.1, 0.3, 0)
    d.terminal(0.2)
    assert not d.ready(0.39)
    assert d.ready(0.41)
    d.observe(0.42, 0.019, 0.049)
    assert d.ready(0.42)
    d.observe(0.43, 0.02, 0)
    assert not d.ready(0.43)
    d.observe(0.44, 0, 0.05)
    assert not d.ready(0.44)


def test_drain_caps_continuous_foreign_velocity_after_terminal():
    d = VelocityDrain()
    d.start(0)
    d.terminal(1)
    for now in (1, 1.5, 2, 2.99):
        d.observe(now, 0.3, 0)
        assert not d.ready(now)
    d.observe(3, 0.3, 0)
    assert d.ready(3)
    d.reset()
    assert not d.active and d.ready(3)


def test_drain_no_messages_and_repeated_cancel_do_not_restart_wait():
    d = VelocityDrain()
    d.start(0)
    d.terminal(0.1)
    d.start(0.2)
    d.terminal(0.2)
    assert not d.ready(0.29) and d.ready(0.3)
    assert d.terminal_at == 0.1
    assert not d.ready(-1)


def test_visible_far_person_does_not_cancel_detour_during_goal_updates():
    s = navigating()
    for i in range(1, 101):
        t = 1+i*0.2
        decision = tick(s, t, state='FOLLOWING', los_clear=True,
                        los_clear_wide=True, front_clearance=2,
                        target=(3, 0), map_target=(3+0.18*t, 5))
        assert decision.mode == 'NAV2'
        assert decision.action in ('none', 'update_goal')
    # Near, but across a corner: visibility alone still cannot release Nav2.
    for t in (22, 23, 24):
        assert tick(s, t, los_clear=True, los_clear_wide=True,
                    front_clearance=2, target=(0.8, 0.8)).mode == 'NAV2'
    tick(s, 25, los_clear=True, los_clear_wide=True,
         front_clearance=2, target=(1.2, 0))
    assert tick(s, 25.51, los_clear=True, los_clear_wide=True,
                front_clearance=2, target=(1.2, 0)).action == 'cancel_goal'


def test_navigation_standoff_is_in_front_of_person_and_transforms_to_map():
    from shopping_uwb_follow.follow_handoff import standoff_target
    assert standoff_target((3, 4), 1) == pytest.approx((2.4, 3.2))
    assert math.dist(standoff_target((3, 4), 1), (3, 4)) == pytest.approx(1)
    assert transform_xy(standoff_target((3, 0), 1), (10, 20, math.pi/2)) == pytest.approx((10, 22))
    assert standoff_target((0.5, 0), 1) == (0, 0)
    assert standoff_target((0, 0), 1) == (0, 0)
    for target, distance in [(None, 1), ((math.nan, 0), 1), ((2, 0), 0), ((2, 0), math.inf)]:
        with pytest.raises(ValueError):
            standoff_target(target, distance)


def test_resume_bearing_cannot_allow_a_backward_handoff():
    with pytest.raises(ValueError):
        HandoffSupervisor(HandoffParams(resume_max_bearing=math.pi))


def test_configured_safety_envelope_and_speed_budget():
    import yaml
    root = Path(__file__).resolve().parents[4]
    if not (root/'docs').is_dir():
        root = Path('/repo')
    if not (root/'docs').is_dir():
        pytest.skip('repository configuration not mounted')
    follow = yaml.safe_load((root/'ros2_ws/src/shopping_uwb_follow/config/uwb_follow.yaml').read_text())['/**']['ros__parameters']
    nav = yaml.safe_load((root/'ros2_ws/src/shopping_robot_navigation/config/nav2_params.yaml').read_text())
    person = yaml.safe_load((root/'ros2_ws/src/shopping_robot_gazebo/config/person_path.yaml').read_text())
    assert follow['robot_half_width'] >= math.hypot(0.30, 0.25)
    controller = nav['controller_server']['ros__parameters']['FollowPath']
    smoother = nav['velocity_smoother']['ros__parameters']
    assert person['speed'] == 0.35
    # A sustained narrow/turn cap can still cause lag at a realistic pace.
    assert follow['max_linear']*follow['side_min_speed_factor'] < person['speed']
    assert controller['regulated_linear_scaling_min_speed'] < person['speed']
    assert person['speed'] < controller['desired_linear_vel'] <= follow['max_linear']
    assert smoother['max_velocity'][0] == follow['max_linear']
    assert smoother['max_accel'][0] == follow['max_linear_accel']
