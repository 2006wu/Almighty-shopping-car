"""Safety and arbitration contracts, independent of ROS transport."""
import math

import pytest

from shopping_zone_manager.mode_control import (
    ModeManager, VelocityMux, VelocityLimiter, OutputWatchdog, ZERO,
)


def activate(manager, mode):
    effects = manager.request(mode)
    assert manager.state in ('IDLE', 'ESTOP')
    assert manager.step() == ()
    assert manager.state in ('IDLE', 'ESTOP')
    entered = manager.step()
    return effects, entered


def test_restart_and_mode_permissions():
    manager, mux = ModeManager(), VelocityMux()
    for source in ('nav', 'follow', 'teleop'):
        mux.receive(source, 0.4, 0.1, 0)
    assert manager.state == 'IDLE'
    assert mux.select(manager.state, 0).velocity == ZERO
    assert mux.select('GUIDE', 0).source == 'nav'
    assert mux.select('FOLLOW', 0).source == 'follow'
    assert mux.select('TELEOP', 0).source == 'teleop'
    activate(manager, 'follow')
    assert ModeManager().state == 'IDLE'


def test_zero_step_effects_and_transition_gate():
    manager = ModeManager()
    assert activate(manager, 'guide') == (('disable_follow',), ())
    assert manager.state == 'GUIDE'
    assert manager.request('follow') == ('cancel_nav_goals', 'disable_follow')
    for _ in range(5):
        assert manager.step(False) == () and manager.state == 'IDLE'
    assert manager.step() == ('enable_follow',)
    assert manager.state == 'FOLLOW'
    assert activate(manager, 'teleop') == (('cancel_nav_goals', 'disable_follow'), ())
    assert manager.state == 'TELEOP'
    activate(manager, 'idle')
    assert manager.state == 'IDLE'


def test_pending_transition_replaced_and_estop_latched():
    manager = ModeManager()
    manager.request('follow')
    assert 'cancel_nav_goals' in manager.request('estop')
    for mode in ('guide', 'follow', 'teleop', 'idle'):
        assert manager.request(mode) == ()
        manager.step()
        assert manager.state == 'ESTOP'
    activate(manager, 'reset')
    assert manager.state == 'IDLE'
    activate(manager, 'guide')
    assert manager.state == 'GUIDE'
    with pytest.raises(ValueError):
        manager.request('unknown')


def test_priority_expiry_and_no_cached_inputs_after_clear():
    mux = VelocityMux()
    mux.receive('nav', 0.3, 0, 0)
    mux.receive('follow', 0.2, 0, 0)
    assert mux.select('FOLLOW', 0.29).source == 'follow'
    assert mux.select('FOLLOW', 0.3).velocity == ZERO
    assert mux.select('GUIDE', 0.3).source == 'nav'
    assert mux.select('FOLLOW', 0.5).velocity == ZERO
    mux.receive('nav', 0.4, 0, 1)
    mux.receive('safety_stop', 9, 9, 1)
    assert mux.select('GUIDE', 1).source == 'safety_stop'
    assert mux.select('GUIDE', 1).velocity == ZERO
    mux.clear()
    assert mux.select('GUIDE', 1).velocity == ZERO


def test_zero_once_then_optional_low_rate():
    mux = VelocityMux(zero_period=None)
    assert mux.output('GUIDE', 0).velocity == ZERO
    assert mux.output('GUIDE', 1) is None
    mux.receive('nav', 0.1, 0, 2)
    assert mux.output('GUIDE', 2).velocity != ZERO
    assert mux.output('GUIDE', 2.5).velocity == ZERO
    assert mux.output('GUIDE', 3) is None
    periodic = VelocityMux(zero_period=0.2)
    assert periodic.output('IDLE', 0) is not None
    assert periodic.output('IDLE', 0.19) is None
    assert periodic.output('IDLE', 0.2) is not None


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('axis', [0, 1])
def test_nonfinite_input_is_flagged_zero(value, axis):
    mux = VelocityMux()
    velocity = [0.4, 0.5]
    velocity[axis] = value
    mux.receive('nav', *velocity, 0)
    result = mux.select('GUIDE', 0)
    assert result.invalid and result.velocity == ZERO


def test_clamp_reverse_and_bad_configuration():
    mux = VelocityMux()
    mux.receive('nav', 3, -4, 0)
    assert mux.select('GUIDE', 0).velocity == (1.1, -1.5)
    mux.receive('nav', -3, 4, 1)
    assert mux.select('GUIDE', 1).velocity == (0, 1.5)
    assert mux.select('GUIDE', 0).velocity == ZERO  # Future stamps fail closed.
    with pytest.raises(ValueError):
        VelocityMux(max_linear=math.nan)
    with pytest.raises(ValueError):
        VelocityMux(timeouts={'nav': 0.5})
    with pytest.raises(ValueError):
        mux.receive('unknown', 0, 0, 0)


def test_acceleration_bounded_and_safety_zero_immediate():
    limiter = VelocityLimiter()
    assert limiter.step((0.6, 1.2), 0) == ZERO
    assert limiter.step((0.6, 1.2), 0.1) == pytest.approx((0.08, 0.15))
    assert limiter.step(ZERO, 0.11) == ZERO
    assert limiter.step((0.6, 1.2), 0.21) == pytest.approx((0.08, 0.15))
    assert limiter.step((0.6, 1.2), 2) == ZERO


def test_watchdog_stops_without_mux_ticks_or_sim_time():
    watchdog = OutputWatchdog(0.25)
    assert watchdog.output(0) == ZERO
    watchdog.receive((0.4, 0.2), 1)
    assert watchdog.output(1.24) == (0.4, 0.2)
    assert watchdog.output(1.25) == ZERO
    assert watchdog.output(10) == ZERO
    assert watchdog.output(0) == ZERO
    watchdog.receive((math.nan, 0), 11)
    assert watchdog.output(11) == ZERO


def test_linear_stop_bypasses_ramp_during_in_place_turn():
    limiter = VelocityLimiter()
    limiter.step((0.6, 1.2), 0)
    limiter.step((0.6, 1.2), 0.1)
    assert limiter.step((0.0, 1.2), 0.15)[0] == 0.0


def test_follow_requires_guarded_relay_even_after_its_command_expires():
    mux = VelocityMux()
    mux.receive('nav', 0.5, 0.8, 0)
    assert mux.select('FOLLOW', 0).velocity == ZERO
    mux.receive('follow', 0, 0.075, 0)
    assert mux.select('FOLLOW', 0.1).velocity == (0, 0.075)
    # An alive Nav2 publisher must never bypass a crashed/stalled safety guard.
    mux.receive('nav', 0.5, 0.8, 0.31)
    assert mux.select('FOLLOW', 0.31).velocity == ZERO
    assert mux.select('GUIDE', 0.31).velocity == (0.5, 0.8)



def test_watchdog_independently_clamps_fast_commands_and_stops_invalid_input():
    watchdog = OutputWatchdog()
    watchdog.receive((100., -100.), 0.)
    assert watchdog.output(0.) == (1.1, -1.5)
    # Rear visibility requires an independent conservative reverse limit.
    watchdog.receive((-1., 0.), .1)
    assert watchdog.output(.1) == (-0.3, 0.)
    watchdog.receive((math.nan, 1.), .2)
    assert watchdog.output(.2) == ZERO
