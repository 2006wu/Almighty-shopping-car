"""Exercise real signal handling and the last watchdog command in an isolated DDS domain."""
import os
import random
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def check_watchdog_shutdown(stop_signal):
    domain = random.SystemRandom().randint(120, 200)
    context = Context()
    rclpy.init(context=context, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('shutdown_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    received = []
    subscription = node.create_subscription(Twist, '/cmd_vel', received.append, 10)
    publisher = node.create_publisher(Twist, '/cmd_vel_mux/output', 10)
    process = subprocess.Popen(
        [sys.executable, '-c', 'from shopping_zone_manager.output_watchdog_node import main; main()'],
        env={**os.environ, 'ROS_DOMAIN_ID': str(domain)}, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 10
        command = Twist()
        command.linear.x = 0.2
        while time.monotonic() < deadline and not any(m.linear.x > 0 for m in received):
            publisher.publish(command)
            rclpy.spin_once(node, executor=executor, timeout_sec=0.02)
            assert process.poll() is None, process.communicate()[0]
        assert any(m.linear.x > 0 for m in received), 'watchdog never forwarded motion'
        cursor = len(received)
        process.send_signal(stop_signal)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, executor=executor, timeout_sec=0.02)
            if process.poll() is not None and any(
                    m.linear.x == m.angular.z == 0 for m in received[cursor:]):
                break
        output, _ = process.communicate(timeout=2)
        assert process.returncode == 0, output
        assert 'Traceback' not in output
        assert received[cursor:] and received[-1].linear.x == received[-1].angular.z == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=5)
        node.destroy_subscription(subscription)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown(context=context)


if __name__ == '__main__':
    for stop_signal in (signal.SIGINT, signal.SIGTERM):
        check_watchdog_shutdown(stop_signal)
        print(f'PASS: watchdog {stop_signal.name} exits cleanly after publishing zero', flush=True)
