"""ROS-free firmware wire adapter shared by the PTY test and serial node."""
import math
from .codec import Codec


class FirmwareWire:
    def __init__(self, firmware, contract):
        self.firmware = firmware
        self.codec = Codec(contract)
        self.contract = contract
        self.counter = 0
        self.next_due = {}

    def receive(self, key, values, now):
        f = self.firmware
        if not f.connected(now):
            return
        if key == 'cmd':
            f.receive_command(values, now)
        elif key == 'pc_heartbeat':
            f.heartbeat(values[0], now)
        elif key == 'reset':
            f.reset(values[0], now)
        elif key == 'button':
            f.set_button(values[0])
        elif key in ('drop', 'drop_commands'):
            f.drop(values[0], now, commands_only=key == 'drop_commands')

    def feed(self, data, now):
        for key, values in self.codec.feed(data):
            self.receive(key, values, now)

    def telemetry(self, now):
        f = self.firmware
        f.step(now)
        if not f.connected(now):
            return []
        values = {'wheel': (*f.positions, *f.velocities), 'ticks': f.ticks,
                  'imu': (f.yaw_rate,), 'estop': (f.estop,),
                  'status': (self.contract['status_values'].index(f.status),),
                  'mcu_heartbeat': (self.counter,)}
        output = []
        for key, value in values.items():
            rate = self.contract['topics'][key]['rate']
            period = 1/rate
            due = self.next_due.get(key, now)
            if now+1e-9 >= due:
                # Keep phase despite timer jitter; never burst missed telemetry.
                self.next_due[key] = due+max(1, math.floor((now-due)/period)+1)*period
                output.append((key, value))
                if key == 'mcu_heartbeat':
                    self.counter = (self.counter+1) & 0xffffffff
        return output
