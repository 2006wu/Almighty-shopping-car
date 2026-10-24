"""Framing, stream recovery, gated hardware access and actual Linux PTY traffic."""
import binascii
import random
import select
import time
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import pytest

from shopping_mcu_bridge.codec import Codec
from shopping_mcu_bridge.contract import load_contract, geometry
from shopping_mcu_bridge.logic import Firmware, LinkMonitor
from shopping_mcu_bridge.transport import open_transport
from shopping_mcu_bridge.wire import FirmwareWire

PKG = Path(__file__).resolve().parents[1]
SPEC = load_contract(PKG/'config/interface.yaml')
KEYS = [k for k, s in SPEC['topics'].items() if 'wire_id' in s]


def sample(key):
    fmt = SPEC['topics'][key]['format']
    count = struct.calcsize('<'+fmt)//struct.calcsize('<'+fmt[-1])
    return ([True] if fmt == '?' else [-42]*count if fmt.endswith('q') else
            [0xffffffff] if fmt == 'I' else [2] if fmt == 'B' else [0.125]*count)


@pytest.mark.parametrize('key', KEYS)
def test_roundtrip_and_every_split(key):
    codec = Codec(SPEC)
    values = sample(key)
    frame = codec.encode(key, values)
    for split in range(len(frame)+1):
        parser = Codec(SPEC)
        result = parser.feed(frame[:split])+parser.feed(frame[split:])
        assert result == [(key, tuple(values))]
    parser = Codec(SPEC)
    result = []
    for b in frame:
        result += parser.feed(bytes([b]))
    assert result == [(key, tuple(values))]


@pytest.mark.parametrize('key', KEYS)
def test_single_bit_corruption_and_resynchronization(key):
    codec = Codec(SPEC)
    frame = codec.encode(key, sample(key))
    for i in range(len(frame)):
        for bit in range(8):
            bad = bytearray(frame)
            bad[i] ^= 1 << bit
            parser = Codec(SPEC)
            assert parser.feed(bad) == []
            assert parser.feed(frame*3)[-1] == (key, tuple(sample(key)))
            assert len(parser.buffer) < 70


def test_crc_known_vector_noise_multiple_frames_unknown_id_and_length():
    assert binascii.crc_hqx(b'123456789', 0xffff) == 0x29b1
    codec = Codec(SPEC)
    hb = codec.encode('pc_heartbeat', [123])
    assert hb.hex() == 'a507047b000000'+struct.pack('<H', binascii.crc_hqx(hb[1:-2], 0xffff)).hex()
    parser = Codec(SPEC)
    assert parser.feed(b'garbage\xa5\xff\xff\xa5\x01\xff'+hb+hb) == [('pc_heartbeat', (123,))]*2
    parser.feed(random.Random(42).randbytes(10000))
    parser.feed(b'\x00'*100)
    assert parser.feed(hb)[-1] == ('pc_heartbeat', (123,))
    assert len(parser.buffer) == 0


def test_default_uses_pty_without_importing_serial_and_never_uses_supplied_port(monkeypatch):
    monkeypatch.setitem(sys.modules, 'serial', None)
    with pytest.raises(ValueError, match='use_hardware'):
        open_transport(False, '/dev/ttyACM0')
    pc, peer = open_transport()
    try:
        pc.write(b'hello')
        assert select.select([peer.fd], [], [], 1)[0]
        assert peer.read() == b'hello'
        peer.write(b'world')
        assert select.select([pc.fd], [], [], 1)[0]
        assert pc.read() == b'world'
    finally:
        pc.close()
        peer.close()


def test_hardware_requires_both_gates_and_only_then_imports_serial(monkeypatch):
    calls = []
    monkeypatch.setitem(sys.modules, 'serial', SimpleNamespace(Serial=lambda **kw: calls.append(kw)))
    for port in ('', 'auto', '/dev/pts/1'):
        with pytest.raises(ValueError):
            open_transport(True, port)
    assert not calls
    open_transport(True, '/dev/explicit-test-port')
    assert calls == [dict(port='/dev/explicit-test-port', baudrate=115200, timeout=0, write_timeout=0)]


def test_pty_firmware_protocol_safety_disconnect_and_recovery():
    pc, peer = open_transport()
    codec = Codec(SPEC)
    f = Firmware(*geometry(PKG.parent/'shopping_robot_description/urdf/shopping_robot.urdf.xacro'),
                 SPEC['timing'], SPEC['model'])
    wire = FirmwareWire(f, SPEC)
    monitor = LinkMonitor(0, SPEC['timing'])
    received = {}

    def read_exact(endpoint, count):
        data = bytearray()
        deadline = time.monotonic()+1
        while len(data) < count:
            assert time.monotonic() < deadline, 'PTY transfer deadline exceeded'
            select.select([endpoint.fd], [], [], max(0, deadline-time.monotonic()))
            data.extend(endpoint.read())
        assert len(data) == count
        return data

    def cycle(t, heartbeat=None, command=False):
        if heartbeat is not None:
            frame = codec.encode('pc_heartbeat', [heartbeat])
            pc.write(frame[:2])
            wire.feed(read_exact(peer, 2), t)
            pc.write(frame[2:])
            wire.feed(read_exact(peer, len(frame)-2), t)
        if command:
            frame = codec.encode('cmd', [0.2, 0, 0, 0, 0, 0.1])
            pc.write(frame)
            wire.feed(read_exact(peer, len(frame)), t)
        telemetry = wire.telemetry(t)
        frames = b''.join(codec.encode(key, values) for key, values in telemetry)
        if frames:
            peer.write(frames)
        for key, values in codec.feed(read_exact(pc, len(frames))):
            received[key] = values
            if key == 'mcu_heartbeat':
                monitor.heartbeat(values[0], t)
            if key == 'status':
                monitor.receive_status(SPEC['status_values'][values[0]], t)
        return telemetry

    def control(key, value, t):
        frame = codec.encode(key, [value])
        pc.write(frame)
        wire.feed(read_exact(peer, len(frame)), t)

    try:
        for i in range(10):
            cycle(i*0.1, i, command=True)
        assert min(received['wheel'][4:]) > 0
        assert received['ticks'] != (0, 0, 0, 0)
        assert received['imu'][0] > 0
        assert not monitor.should_stop(0.91)
        for i in range(10, 15):
            cycle(i*0.1, i)
        assert received['wheel'][4:] == (0, 0, 0, 0)
        assert SPEC['status_values'][received['status'][0]] == 'CMD_TIMEOUT'
        control('drop', 1.5, 1.5)
        assert cycle(1.5, 15) == []
        assert cycle(2.1, 21, command=True) == []
        assert monitor.should_stop(2.1)
        for i in range(30, 33):
            cycle(i*0.1, i)
        assert received['wheel'][4:] == (0, 0, 0, 0)
        cycle(3.3, 33, command=True)
        assert min(received['wheel'][4:]) > 0
        assert not monitor.should_stop(3.31)
        control('button', True, 3.4)
        cycle(3.4, 34, command=True)
        assert received['estop'] == (True,) and received['wheel'][4:] == (0, 0, 0, 0)
        control('reset', True, 3.5)
        cycle(3.5, 35, command=True)
        assert received['estop'] == (True,)
        control('button', False, 3.6)
        control('reset', True, 3.6)
        cycle(3.6, 36, command=True)
        assert received['estop'] == (False,)
    finally:
        pc.close()
        peer.close()


def test_telemetry_rate_keeps_phase_under_timer_jitter_without_catchup_burst():
    f = Firmware(0.1, 0.92, SPEC['timing'], SPEC['model'])
    wire = FirmwareWire(f, SPEC)
    counts = {}
    # Alternating early/late callbacks must not reduce a 50 Hz stream to ~33 Hz.
    for i in range(1000):
        t = i*0.01 + (0.0004 if i % 2 == 0 else -0.0004)
        for key, _ in wire.telemetry(t):
            counts[key] = counts.get(key, 0)+1
    assert 499 <= counts['wheel'] <= 501
    assert 99 <= counts['mcu_heartbeat'] <= 101
    assert len(wire.telemetry(100)) == 6
    assert wire.telemetry(100.0001) == []
