"""Actor yielding does not modify robot commands or use planned poses as truth."""
import math

import pytest

from shopping_robot_sim.pedestrian_yield import YieldingTrajectory
from shopping_robot_sim.scenarios import load_scenario
from shopping_robot_sim.trajectory import tracking_velocity
from test_scenarios import CONFIG


PERSON = dict(speed=.3, radius=.25, start_delay=0, waypoints=[[0., 0.], [4., 0.]])


@pytest.mark.parametrize('robot', [None, (0., 3., 0.), (2., 3., 0.), (1., math.nan, 0.)])
def test_missing_stale_future_or_invalid_robot_stops_actor(robot):
    actor = YieldingTrajectory()
    assert actor.velocity(PERSON, 1., (0., 0.), 0., robot) == (0., 0.)
    assert actor.blocked


def test_yield_before_contact_and_resume_without_schedule_catchup():
    actor = YieldingTrajectory()
    # Current separation is outside the margin, but the swept trajectory enters it.
    assert actor.velocity(PERSON, 1., (.3, 0.), 0., (1., 1.2, 0.)) == (0., 0.)
    for t in range(2, 61):
        assert actor.velocity(PERSON, float(t), (.3, 0.), 0., (float(t), 1.2, 0.)) == (0., 0.)
    assert actor.elapsed == 1.
    command = actor.velocity(PERSON, 60.1, (.3, 0.), 0., (60.1, 3., 3.))
    assert command == pytest.approx((.3, 0.))
    assert actor.elapsed == pytest.approx(1.1)
    assert not actor.blocked


def test_hysteresis_and_body_frame_sweep():
    actor = YieldingTrajectory()
    actor.velocity(PERSON, 1., (0., 0.), math.pi/2, (1., .8, 0.))
    assert actor.blocked
    actor.velocity(PERSON, 1.1, (0., 0.), math.pi/2, (1.1, .9, 0.))
    assert actor.blocked
    assert actor.velocity(PERSON, 1.2, (0., 0.), math.pi/2, (1.2, 2., 0.)) == pytest.approx((0., -.3))


def test_aggressive_flag_exactly_restores_old_tracking():
    actor = YieldingTrajectory()
    for t in (1., 2., 30., 5.):
        assert actor.velocity(PERSON, t, (0., 0.), .7, None, enabled=False) == tracking_velocity(PERSON, t, (0., 0.), .7)


def test_frozen_robot_on_t71_contact_approach_is_never_pushed_by_actor():
    person = dict(PERSON, waypoints=[[0., 4.8], [0., 1.2]], start_delay=3.)
    actor = YieldingTrajectory()
    y = 2.7
    minimum = float('inf')
    for i in range(300):
        t = 106.+i*.02
        _, vy = actor.velocity(person, t, (0., y), 0., (t, 0., 1.6046))
        y += vy*.02
        minimum = min(minimum, y-1.6046-.30-.25)
    assert actor.blocked
    assert minimum > .20


def test_crossing_exits_encounter_segment_and_dwell_is_valid():
    person = load_scenario(CONFIG, 'aisle_crossing')['pedestrians'][0]
    assert max(y for x, y in person['waypoints']) == 5.8
    assert person['pauses'] == [0, 0, 8, 8]
