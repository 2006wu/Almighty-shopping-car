"""Safety invariants shared by both simulated transports."""
from pathlib import Path
import math
import pytest

from shopping_mcu_bridge.contract import load_contract, geometry
from shopping_mcu_bridge.logic import Firmware, Heartbeat, LinkMonitor

PKG = Path(__file__).resolve().parents[1]
# In colcon (/ws/src) the repo root is not a parent; the container mounts it at /repo.
ROOT = PKG.parents[2] if (PKG.parents[2]/'docs').is_dir() else Path('/repo')
SPEC = load_contract(PKG/'config/interface.yaml')
MODEL = ROOT/'ros2_ws/src/shopping_robot_description/urdf/shopping_robot.urdf.xacro'


def firmware():
    return Firmware(*geometry(MODEL), SPEC['timing'], SPEC['model'])


def prime(f, base=0):
    for i in range(3):
        f.heartbeat(i, base+i*0.1)
    f.receive_command((0.4, 0, 0, 0, 0, 0), base+0.2)
    f.step(base+0.2)
    f.step(base+0.22)
    assert min(f.velocities) > 0


def test_geometry_matches_skid_steer_description():
    assert geometry(MODEL) == pytest.approx((0.1, 0.92))


def test_startup_and_timeout_are_immediate_zero():
    f = firmware()
    f.receive_command((0.4, 0, 0, 0, 0, 0), 0)
    f.step(0)
    assert f.status == 'LINK_LOST' and f.velocities == [0]*4
    prime(f)
    f.heartbeat(3, 0.4)
    f.step(0.501)
    assert f.status == 'CMD_TIMEOUT' and f.velocities == [0]*4


def test_link_timeout_requires_three_new_heartbeats_and_new_command():
    f = firmware()
    prime(f)
    f.step(0.71)
    assert f.status == 'LINK_LOST' and f.velocities == [0]*4
    for i in range(2):
        f.heartbeat(i, 0.8+i*0.1)
        f.receive_command((0.5, 0, 0, 0, 0, 0), 0.8+i*0.1)
        f.step(0.8+i*0.1)
        assert f.status == 'LINK_LOST'
    f.heartbeat(2, 1.0)
    f.step(1.0)
    assert f.status == 'CMD_TIMEOUT'
    f.receive_command((0.3, 0, 0, 0, 0, 0), 1.01)
    f.step(1.02)
    assert f.status == 'READY' and min(f.velocities) > 0


def test_estop_latch_requires_released_button_explicit_reset_and_fresh_cmd():
    f = firmware()
    prime(f)
    f.set_button(True)
    assert f.velocities == [0]*4
    f.reset(True, 0.25)
    f.step(0.25)
    assert f.estop and f.status == 'ESTOP'
    f.set_button(False)
    f.reset(False, 0.26)
    assert f.estop
    f.reset(True, 0.27)
    f.step(0.27)
    assert not f.estop and f.status == 'CMD_TIMEOUT'
    f.set_button(True)
    f.set_button(False)
    f.reset(True, 1)
    assert f.estop  # A dead link cannot reset a latch.


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('index', range(6))
def test_nonfinite_components_rejected(bad, index):
    f = firmware()
    prime(f)
    values = [0.2, 0, 0, 0, 0, 0]
    values[index] = bad
    f.receive_command(values, 0.23)
    f.step(0.24)
    assert f.status == 'INVALID_COMMAND' and f.velocities == [0]*4


@pytest.mark.parametrize('index', [1, 2, 3, 4])
def test_nonplanar_commands_rejected(index):
    f = firmware()
    prime(f)
    values = [0.2, 0, 0, 0, 0, 0]
    values[index] = 0.01
    f.receive_command(values, 0.23)
    f.step(0.24)
    assert f.status == 'INVALID_COMMAND'


def test_saturation_no_reverse_acceleration_and_response():
    f = firmware()
    prime(f)
    f.receive_command((-1e30, 0, 0, 0, 0, 1e30), 0.23)
    assert f.command == (0, 1.2)
    f.receive_command((1e30, 0, 0, 0, 0, -1e30), 0.23)
    assert f.command == (0.6, -1.2)
    before = f.velocities[:]
    f.step(0.24)
    assert all(abs(a-b) <= 12*0.02+1e-9 for a, b in zip(before, f.velocities))
    for i in range(1, 501):
        t = 0.24+i*0.01
        f.heartbeat(i+10, t)
        f.receive_command((0.2, 0, 0, 0, 0, 0.3), t)
        f.step(t)
    assert f.velocities[:2] == pytest.approx([(0.2-0.3*0.92/2)/0.1]*2)
    assert f.velocities[2:] == pytest.approx([(0.2+0.3*0.92/2)/0.1]*2)
    assert f.yaw_rate == pytest.approx(0.3)
    assert f.ticks == [round(p*4096/(2*math.pi)) for p in f.positions]


def test_disconnect_drops_both_directions_and_command_loss_is_a_timeout():
    f = firmware()
    prime(f)
    f.drop(1, 0.23, commands_only=True)
    f.receive_command((0.6, 0, 0, 0, 0, 0), 0.24)
    f.step(0.25)
    assert f.status == 'READY' and min(f.velocities) > 0
    f.heartbeat(3, 0.45)
    f.step(0.501)
    assert f.status == 'CMD_TIMEOUT'
    f.drop(1, 0.6)
    assert not f.connected(0.61) and f.velocities == [0]*4
    f.heartbeat(4, 0.7)
    assert not f.link.fresh(0.7)


@pytest.mark.parametrize('value', [-1, math.nan, math.inf, 61])
def test_invalid_drop_durations_ignored(value):
    f = firmware()
    prime(f)
    f.drop(value, 0.23)
    assert f.connected(0.24) and f.command_time == 0.2


def test_counter_replay_wrap_reboot_and_time_regression():
    h = Heartbeat()
    for value, now in [(0xfffffffe, 0), (0xffffffff, 0.1), (0, 0.2)]:
        assert h.receive(value, now)
    assert h.fresh(0.3)
    assert not h.receive(0, 0.4)
    assert not h.receive(0xffffffff, 0.4)
    assert h.last == 0.2
    assert not h.fresh(0.71)
    assert h.receive(0, 0.8)
    assert not h.fresh(0.81)
    assert not h.fresh(0.7)


def test_monitor_grace_replays_status_loss_estop_and_recovery():
    m = LinkMonitor(0, SPEC['timing'])
    m.receive_status('LINK_LOST', 0.1)
    assert not m.should_stop(0.2)
    assert m.should_stop(0.51)
    for i in range(3):
        m.heartbeat(i, 0.6+i*0.1)
    m.receive_status('READY', 0.8)
    assert not m.should_stop(0.85)
    m.estop = True
    assert m.should_stop(0.86)
    m.estop = False
    m.receive_status('ESTOP', 0.87)
    assert m.should_stop(0.87)
    m.receive_status('READY', 0.9)
    assert not m.should_stop(0.91)
    assert m.should_stop(1.41)
    # A live heartbeat without fresh status is also unsafe.
    for i in range(3, 10):
        m.heartbeat(i, 1.4+i*0.1)
    assert m.should_stop(2.31)


def test_paused_model_clock_cannot_override_wall_time_timeout():
    f = firmware()
    prime(f)
    f.step(100)
    assert f.velocities == [0]*4 and f.status == 'LINK_LOST'
