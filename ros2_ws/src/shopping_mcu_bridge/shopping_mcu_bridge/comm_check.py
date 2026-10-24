"""Isolated simulation acceptance checker; routes motion through mux and watchdog."""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .nodes import contract, publisher, subscribe, message_type


class Check(Node):
    def __init__(self):
        super().__init__('mcu_comm_check')
        self.topics = contract()['topics']
        self.values = {}
        self.samples = {}
        self.requests = []
        self.isolated = False
        self.pubs = {k: publisher(self, self.topics[k]) for k in
                     ('mode_request', 'drop', 'drop_commands', 'button', 'reset')}
        # The existing mux and watchdog remain the only route to /cmd_vel.
        cmd_spec = dict(self.topics['cmd'], name='/cmd_vel/teleop')
        self.teleop = publisher(self, cmd_spec)
        for key in ('wheel', 'mcu_heartbeat', 'pc_heartbeat', 'status', 'estop', 'sim_active'):
            subscribe(self, self.topics[key], lambda msg, k=key: self.receive(k, msg))
        subscribe(self, self.topics['mode_request'], lambda msg: self.requests.append((time.monotonic(), msg.data)))
        self.create_subscription(String, '/mode_manager/state', lambda msg: self.values.update(mode=msg.data), 10)

    def receive(self, key, msg):
        self.values[key] = msg
        self.samples.setdefault(key, []).append((time.monotonic(), msg))

    def send(self, key, value):
        self.pubs[key].publish(message_type(self.topics[key])(data=value))

    def wait(self, duration, motion=False):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            if motion:
                msg = message_type(self.topics['cmd'])()
                msg.linear.x = 0.2
                self.teleop.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wheel_zero(self):
        msg = self.values.get('wheel')
        return msg is not None and len(msg.velocity) == 4 and all(abs(v) < 1e-6 for v in msg.velocity)

    def moving(self):
        msg = self.values.get('wheel')
        return msg is not None and len(msg.velocity) == 4 and max(abs(v) for v in msg.velocity) > 0.2

    def status(self):
        msg = self.values.get('status')
        return msg.data if msg is not None else ''

    def activate(self):
        self.send('mode_request', 'reset')
        self.wait(0.2)
        self.send('mode_request', 'teleop')
        self.wait(1.2, motion=True)

    def checks(self):
        # DDS discovery of a fresh checker can take a few seconds; poll before deciding.
        deadline = time.monotonic()+10.0
        while True:
            self.wait(1.0)
            owners = self.get_publishers_info_by_topic('/cmd_vel')
            readers = self.get_subscriptions_info_by_topic('/cmd_vel')
            marker = self.values.get('sim_active')
            isolated = (marker is not None and marker.data and len(owners) == 1
                        and owners[0].node_name == 'cmd_vel_watchdog'
                        and len(readers) == 1
                        and readers[0].node_name in ('sim_mcu_firmware', 'serial_bridge'))
            if isolated or time.monotonic() > deadline:
                break
        self.isolated = isolated
        if not isolated:
            self.get_logger().error(
                f'Not isolated: sim_active={None if marker is None else marker.data}, '
                f'/cmd_vel publishers={[o.node_name for o in owners]}, '
                f'subscribers={[r.node_name for r in readers]}')
        yield 'isolated simulation + watchdog owner', isolated
        if not isolated:
            return  # Never inject motion into Gazebo or a real transport.
        start = time.monotonic()
        self.wait(2.0)
        rates = {k: len([t for t, _ in self.samples.get(k, []) if t >= start])/2
                 for k in ('wheel', 'mcu_heartbeat', 'pc_heartbeat')}
        advancing = all(len({m.data for t, m in self.samples.get(k, []) if t >= start}) >= 10
                        for k in ('mcu_heartbeat', 'pc_heartbeat'))
        yield 'heartbeats counters + 50 Hz wheel rates', (advancing and 35 <= rates['wheel'] <= 65
                                                        and all(7 <= rates[k] <= 13 for k in rates if k != 'wheel'))
        self.activate()
        yield 'mux command reaches firmware', self.moving()
        self.send('drop_commands', 1.5)
        injected = time.monotonic()
        self.wait(0.65, motion=True)
        zero_samples = [m for t, m in self.samples['wheel'] if t >= injected+0.4]
        yield 'command timeout holds zero', (self.status() == 'CMD_TIMEOUT' and len(zero_samples) >= 5
                                             and all(all(abs(v) < 1e-6 for v in m.velocity) for m in zero_samples))
        self.wait(1.0, motion=True)
        yield 'command stream recovers', self.moving()
        self.send('drop', 1.4)
        injected = time.monotonic()
        self.wait(1.0)
        yield 'link loss requests estop', any(t >= injected and s == 'estop' for t, s in self.requests)
        self.wait(1.4)
        recent = [m.data for t, m in self.samples.get('mcu_heartbeat', []) if t >= injected+1.4]
        yield 'fresh heartbeat recovery remains stopped', (len(set(recent)) >= 3
                                                           and self.status() in ('CMD_TIMEOUT', 'READY')
                                                           and self.wheel_zero() and self.values.get('mode') == 'ESTOP')
        self.activate()
        self.send('button', True)
        self.wait(0.25, motion=True)
        yield 'MCU button latches stop', self.wheel_zero() and self.values['estop'].data
        self.send('reset', True)
        self.wait(0.2)
        yield 'pressed button rejects reset', self.values['estop'].data
        self.send('button', False)
        self.wait(0.2)  # Different topics have no cross-writer ordering guarantee.
        self.send('reset', True)
        self.wait(0.4)
        self.activate()
        yield 'explicit resets permit recovery', not self.values['estop'].data and self.moving()


def main(args=None):
    rclpy.init(args=args)
    node = Check()
    results = []
    try:
        print('RESULT | CHECK', flush=True)
        for label, passed in node.checks():
            results.append(passed)
            print(f"{'PASS' if passed else 'FAIL':4} | {label}", flush=True)
    finally:
        # Always leave the isolated mux stopped, including exceptions/interrupts.
        if node.isolated:
            node.send('mode_request', 'estop')
            node.wait(0.15)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if len(results) == 10 and all(results) else 1)
