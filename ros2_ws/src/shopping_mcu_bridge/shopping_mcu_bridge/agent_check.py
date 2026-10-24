"""ROS acceptance entry point for the Linux rclc client through the XRCE agent."""
import time
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool
from .comm_check import Check
from .agent_checks import isolated_graph, zero_window, advancing_counter


class AgentCheck(Check):
    def __init__(self):
        super().__init__()
        self.xrce_marker_time = -float('inf')
        self.create_subscription(Bool, '/mcu/sim/xrce_active', self.marker, 1)
        self.parameters = self.create_client(SetParameters, '/mcu_link_monitor/set_parameters')
        self.guard_at = 0.0
        self.pause_attempted = False

    def marker(self, msg):
        if msg.data:
            self.xrce_marker_time = time.monotonic()

    def graph_safe(self):
        topics = ('/cmd_vel', '/pc/heartbeat', '/mcu/heartbeat', '/mcu/wheel_state',
                  '/mcu/status', '/mcu/sim/xrce_active')
        owners = {t: [p.node_name for p in self.get_publishers_info_by_topic(t)] for t in topics}
        return isolated_graph(self.get_node_names(), owners,
                              [s.node_name for s in self.get_subscriptions_info_by_topic('/cmd_vel')],
                              self.get_publishers_info_by_topic('/clock'),
                              time.monotonic()-self.xrce_marker_time < 2.5)

    def wait(self, duration, motion=False):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            # Recheck isolation during injections, not only during initial discovery.
            if self.isolated and time.monotonic() >= self.guard_at:
                self.guard_at = time.monotonic()+0.2
                if not self.graph_safe():
                    raise RuntimeError('Graph isolation lost; refusing further motion injection')
            super().wait(min(0.04, max(0, end-time.monotonic())), motion)

    def pause_pc(self, value):
        self.pause_attempted |= value
        if not self.parameters.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('Monitor parameter service unavailable')
        request = SetParameters.Request(parameters=[Parameter(
            name='simulate_pc_heartbeat_loss',
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value))])
        future = self.parameters.call_async(request)
        deadline = time.monotonic()+2
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done() or not future.result() or not all(r.successful for r in future.result().results):
            raise RuntimeError('Monitor heartbeat fault parameter failed')

    def checks(self):
        deadline = time.monotonic()+10.0
        while True:
            self.wait(0.5)
            safe = self.graph_safe()
            if safe or time.monotonic() >= deadline:
                break
        yield 'isolated XRCE client + sole watchdog /cmd_vel publisher', safe
        if not safe:
            self.get_logger().error('Refusing injection: stop Gazebo and all other firmware/communication launches')
            return
        self.isolated = True
        start = time.monotonic()
        self.wait(2.0)
        samples = {k: [(t, m) for t, m in self.samples.get(k, []) if t >= start]
                   for k in ('wheel', 'mcu_heartbeat', 'pc_heartbeat')}
        yield 'advancing heartbeats and wheel ~50 Hz', (
            70 <= len(samples['wheel']) <= 130 and all(
                14 <= len(samples[k]) <= 26 and advancing_counter(samples[k])
                for k in ('mcu_heartbeat', 'pc_heartbeat')))
        self.activate()
        yield 'teleop through mux/watchdog moves wheels', self.moving() and self.status() == 'READY'
        self.send('drop_commands', 1.5)
        injected = time.monotonic()
        self.wait(0.75, motion=True)
        yield 'MCU 0.3 s command timeout holds zero', (
            self.status() == 'CMD_TIMEOUT' and zero_window(self.samples.get('wheel', []), injected+0.45))
        self.wait(1.2, motion=True)
        yield 'fresh commands recover motion', self.moving() and self.status() == 'READY'
        self.pause_pc(True)
        injected = time.monotonic()
        self.wait(0.9, motion=True)
        yield 'PC heartbeat loss >0.5 s gives LINK_LOST and zero', (
            not any(t >= injected+0.15 for t, _ in self.samples.get('pc_heartbeat', []))
            and self.status() == 'LINK_LOST'
            and zero_window(self.samples.get('wheel', []), injected+0.65))
        self.pause_pc(False)
        self.wait(0.8)
        yield 'heartbeat recovery keeps ESTOP and wheels zero', (
            self.status() in ('READY', 'CMD_TIMEOUT') and self.wheel_zero()
            and self.values.get('mode') == 'ESTOP')


def main(args=None):
    rclpy.init(args=args)
    node = AgentCheck()
    results = []
    try:
        print('RESULT | XRCE-DDS CHECK', flush=True)
        for label, passed in node.checks():
            results.append(passed)
            print(f"{'PASS' if passed else 'FAIL':4} | {label}", flush=True)
    except Exception as exc:
        node.get_logger().error(str(exc))
        results.append(False)
    finally:
        # Stop before restoring heartbeat, including interrupts and failing checks.
        if node.isolated:
            node.send('mode_request', 'estop')
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pause_attempted:
                try:
                    node.pause_pc(False)
                except Exception as exc:
                    node.get_logger().error(f'Could not restore monitor heartbeat: {exc}')
                    results.append(False)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if len(results) == 7 and all(results) else 1)
