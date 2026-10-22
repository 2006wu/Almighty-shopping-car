import sys
from ament_index_python.packages import get_package_share_directory
from .zones import load_zones


def list_main():
    path = get_package_share_directory('shopping_zone_manager')+'/config/zones.yaml'
    for zone in load_zones(path):
        print(f'{zone.name}: {", ".join(zone.aliases)} ({zone.x}, {zone.y}) [{zone.frame_id}]')


def goto_main():
    import time
    import rclpy
    from .node import ZoneNavigator
    rclpy.init()
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(args) != 1:
        raise SystemExit('Usage: goto_zone NAME')
    node = ZoneNavigator()
    result = []
    node.finished = lambda success: result.append(success)
    node.create_subscription(__import__('std_msgs.msg', fromlist=['String']).String,
                             '/shopping/zone_status', lambda m: print(m.data), 10)
    try:
        if not node.client.wait_for_server(timeout_sec=15.0) or not node.navigate(args[0]):
            raise SystemExit('Goal could not be submitted')
        deadline = time.monotonic()+300
        while rclpy.ok() and not result and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not result or not result[0]:
            raise SystemExit('Navigation failed or timed out; inspect Nav2 before retrying')
    finally:
        node.destroy_node()
        rclpy.shutdown()
