"""Run a fixed Nav2 mapping tour and save grid plus serialized pose graph."""
import json
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from slam_toolbox.srv import SerializePoseGraph

from .experiment_node import ExperimentNode
from .tour_logic import Sequence, load_tour, map_prefix


def save_maps(node, prefix):
    # Match dev.sh save-map, including simulation time and both required formats.
    subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', prefix,
                    '--ros-args', '-p', 'use_sim_time:=true'], check=True, timeout=60)
    client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('SLAM serialization service unavailable; grid may already exist')
        request = SerializePoseGraph.Request()
        request.filename = prefix
        response = node.wait(client.call_async(request), time.monotonic()+30.0)
        if response.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            raise RuntimeError(f'Pose graph serialization failed: {response.result}')
        for suffix in ('.yaml', '.pgm', '.posegraph', '.data'):
            if not Path(prefix+suffix).is_file():
                raise RuntimeError(f'Missing saved artifact: {prefix+suffix}')
    finally:
        node.destroy_client(client)


def main():
    # Keep the ROS context alive on Ctrl-C so the active Nav2 goal can be canceled.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ExperimentNode('map_tour')
    sequence = None
    saved = False
    code = 1
    try:
        config = node.parameter('tour_file', get_package_share_directory('shopping_zone_manager')+'/config/map_tour.yaml')
        name = node.parameter('map_name', 'supermarket')
        directory = Path(node.parameter('maps_dir', '/ws/src/shopping_robot_navigation/maps'))
        prefix = map_prefix(directory, name)
        if not directory.is_dir():
            raise ValueError('maps_dir must be an existing writable maps directory')
        timeout = node.positive('overall_timeout', 1800.0)
        goal_timeout = node.positive('goal_timeout', 150.0)
        failures = node.parameter('max_failures', 2)
        sequence = Sequence(load_tour(config), time.monotonic(), timeout, failures)
        deadline = sequence.started+timeout
        node.ready(min(deadline, time.monotonic()+60.0))
        while (goal := sequence.current(time.monotonic())) is not None:
            success = node.navigate(goal, min(deadline, time.monotonic()+goal_timeout))
            sequence.record(success)
        # Save completed tours even if some waypoints were skipped, but return failure.
        # Never advertise a timed-out or interrupted tour as a canonical complete map.
        if not sequence.timed_out:
            node.pause(3.0)  # Allow the final scans and grid update to be processed.
            save_maps(node, prefix)
            saved = True
        code = 0 if sequence.summary()['success'] and saved else 1
    except (Exception, KeyboardInterrupt) as exc:
        node.get_logger().error(f'Tour stopped: {exc}')
    finally:
        summary = sequence.summary() if sequence else {'success': False}
        summary['maps_saved'] = saved
        node.get_logger().info(json.dumps(summary))
        try:
            node.close()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    raise SystemExit(code)
