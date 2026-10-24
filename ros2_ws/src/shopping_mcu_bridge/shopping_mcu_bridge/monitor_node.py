"""PC heartbeat and fail-safe mode request; never publishes chassis velocity."""
import time
from rclpy.node import Node
from .nodes import contract, publisher, subscribe, message_type, steady_timer, run
from .logic import LinkMonitor


class MonitorNode(Node):
    def __init__(self):
        super().__init__('mcu_link_monitor')
        self.stop_enabled = self.declare_parameter('estop_on_link_loss', False).value
        spec = contract()
        self.topics = spec['topics']
        self.logic = LinkMonitor(time.monotonic(), spec['timing'])
        self.counter = 0
        self.hb = publisher(self, self.topics['pc_heartbeat'])
        self.mode = publisher(self, self.topics['mode_request'])
        subscribe(self, self.topics['mcu_heartbeat'], lambda m: self.logic.heartbeat(m.data, time.monotonic()))
        subscribe(self, self.topics['status'], lambda m: self.logic.receive_status(m.data, time.monotonic()))
        subscribe(self, self.topics['estop'], self.estop)
        steady_timer(self, 1/self.topics['pc_heartbeat']['rate'], self.tick)

    def estop(self, msg):
        self.logic.estop = msg.data
        if msg.data:
            self.request_stop()

    def request_stop(self):
        if self.stop_enabled:
            self.mode.publish(message_type(self.topics['mode_request'])(data='estop'))

    def tick(self):
        self.hb.publish(message_type(self.topics['pc_heartbeat'])(data=self.counter))
        self.counter = (self.counter+1) & 0xffffffff
        if self.logic.should_stop(time.monotonic()):
            self.request_stop()  # Repeat so late/restarted mode managers receive the stop.


def main(args=None):
    run(MonitorNode, args)
