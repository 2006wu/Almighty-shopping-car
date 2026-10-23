"""Command entry points for the single, persistent navigation owner."""
import argparse
import math
import sys
import time
from .goal_tracker import TERMINAL


def positive_seconds(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('Value must be positive and finite')
    return value


def list_main():
    from ament_index_python.packages import get_package_share_directory
    from .zones import load_zones
    path = get_package_share_directory('shopping_zone_manager') + '/config/zones.yaml'
    for zone in load_zones(path):
        print(f'{zone.name}: {", ".join(zone.aliases)} ({zone.x}, {zone.y}) [{zone.frame_id}]')


def wait_until(node, predicate, timeout):
    import rclpy
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return predicate()


def cancel_and_wait(node, client, timeout=5.0):
    """Trigger acknowledges dispatch, not the final action cancellation."""
    from std_srvs.srv import Trigger
    if not client.wait_for_service(timeout_sec=timeout):
        print('UNAVAILABLE: cancel service unavailable; cancellation unconfirmed', file=sys.stderr)
        return False
    future = client.call_async(Trigger.Request())
    if not wait_until(node, future.done, timeout):
        print('CANCEL_FAILED: cancel service response lost; cancellation unconfirmed', file=sys.stderr)
        return False
    try:
        response = future.result()
        print(response.message)
        return response.success
    except Exception as exc:
        print(f'CANCEL_FAILED: {exc}', file=sys.stderr)
        return False


def goto_main():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import remove_ros_args
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    parser = argparse.ArgumentParser(description='Navigate through the persistent zone manager')
    parser.add_argument('zone')
    parser.add_argument('--timeout', type=positive_seconds, default=300.0)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('goto_zone')
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    cancel = node.create_client(Trigger, '/shopping/cancel_goal')
    publisher = node.create_publisher(String, '/shopping/goto_zone', 10)
    statuses = []
    node.create_subscription(String, '/shopping/zone_status', lambda msg: statuses.append(msg.data), 10)
    submitted = False
    exit_code = 1
    try:
        if not action.wait_for_server(timeout_sec=15.0):
            print('UNAVAILABLE: NavigateToPose server not ready', file=sys.stderr)
            return 1
        if not wait_until(node, lambda: publisher.get_subscription_count() > 0
                          and node.count_publishers('/shopping/zone_status') > 0, 5.0):
            print('UNAVAILABLE: zone manager not ready; start guide.launch', file=sys.stderr)
            return 1
        publisher.publish(String(data=args.zone))
        submitted = True
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            while statuses:
                message = statuses.pop(0)
                print(message, flush=True)
                code = message.split(':', 1)[0]
                if code in TERMINAL or code in ('BUSY', 'CANCEL_FAILED'):
                    submitted = False
                    return 0 if code == 'SUCCEEDED' else 1
        print('TIMEOUT: navigation deadline exceeded; requesting cancellation', file=sys.stderr)
        exit_code = 1
    except KeyboardInterrupt:
        print('CANCELED: interrupted; requesting cancellation', file=sys.stderr)
        exit_code = 130
    finally:
        if submitted:
            try:
                if cancel_and_wait(node, cancel):
                    settled = lambda: any(s.split(':', 1)[0] in TERMINAL for s in statuses)
                    if not wait_until(node, settled, 5.0):
                        print('CANCEL_FAILED: final cancellation unconfirmed; inspect zone_status', file=sys.stderr)
            except KeyboardInterrupt:
                print('CANCEL_FAILED: interrupted during cleanup; cancellation unconfirmed', file=sys.stderr)
        action.destroy()
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


def cancel_main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import Trigger
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('cancel_zone')
    try:
        return 0 if cancel_and_wait(node, node.create_client(Trigger, '/shopping/cancel_goal')) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
