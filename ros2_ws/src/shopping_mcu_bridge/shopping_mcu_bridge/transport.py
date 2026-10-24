"""ROS-free nonblocking PTY and explicitly gated real-serial transports."""
import os
import pty
import tty


class FDTransport:
    def __init__(self, fd):
        self.fd = fd
        os.set_blocking(fd, False)
        self.pending = bytearray()

    def read(self):
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b''

    def write(self, data):
        if len(self.pending)+len(data) > 16384:
            raise OSError('Transport backpressure; refusing stale command queue')
        self.pending.extend(data)
        try:
            count = os.write(self.fd, self.pending) if self.pending else 0
            del self.pending[:count]
        except BlockingIOError:
            pass

    def close(self):
        os.close(self.fd)


def loopback_pair():
    master, slave = pty.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    return FDTransport(master), FDTransport(slave)


def open_transport(use_hardware=False, port='', baud=115200):
    if not use_hardware:
        if port:
            raise ValueError('port requires explicit use_hardware:=true')
        return loopback_pair()
    if not port or not port.startswith('/dev/') or port.startswith('/dev/pts/'):
        raise ValueError('Explicit real /dev port required for hardware transport')
    # This is the only hardware-opening path. No probing or automatic retries.
    import serial
    return serial.Serial(port=port, baudrate=baud, timeout=0, write_timeout=0), None
