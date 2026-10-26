"""Run a fixed Nav2 mapping tour and save grid plus serialized pose graph."""
import json
import os
import tempfile

import yaml
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from slam_toolbox.srv import SerializePoseGraph

from .experiment_node import ExperimentNode
from .tour_logic import Sequence, load_tour, map_prefix


def _save_map_artifacts(node, prefix):
    # A localization node may acknowledge serialization without writing a graph.
    # Existing artifacts must never turn that response into a false success.
    suffixes = ('.yaml', '.pgm', '.posegraph', '.data')
    def signature(path):
        if not path.is_file():
            return None
        stat = path.stat()
        return stat.st_mtime_ns, stat.st_ctime_ns, stat.st_size
    previous = {suffix: signature(Path(prefix+suffix)) for suffix in suffixes}
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
        for suffix in suffixes:
            artifact = Path(prefix+suffix)
            current = signature(artifact)
            if current is None or current[-1] == 0:
                raise RuntimeError(f'Missing or empty saved artifact: {artifact}')
            if current == previous[suffix]:
                raise RuntimeError(f'Saved artifact was not refreshed: {artifact}')
    finally:
        node.destroy_client(client)


def save_maps(node, prefix):
    # Validate a fresh complete set before replacing any previous map files.
    destination = Path(prefix)
    with tempfile.TemporaryDirectory(prefix='.shopping-map-', dir=destination.parent) as directory:
        staged = str(Path(directory)/destination.name)
        _save_map_artifacts(node, staged)
        metadata_path = Path(staged+'.yaml')
        metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
        if not isinstance(metadata, dict):
            raise RuntimeError('Saved map metadata is not a mapping')
        metadata['image'] = destination.name+'.pgm'
        metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
        for suffix in ('.yaml', '.pgm', '.posegraph', '.data'):
            os.replace(staged+suffix, prefix+suffix)


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


def save_main():
    """Save both map formats and reject unsuccessful serialization responses."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ExperimentNode('save_map')
    code = 1
    try:
        name = node.parameter('map_name', 'supermarket')
        directory = Path(node.parameter('maps_dir', '/ws/src/shopping_robot_navigation/maps'))
        prefix = map_prefix(directory, name)
        if not directory.is_dir():
            raise ValueError('maps_dir must be an existing writable maps directory')
        save_maps(node, prefix)
        node.get_logger().info(f'Saved grid and serialized pose graph: {prefix}')
        code = 0
    except (Exception, KeyboardInterrupt) as exc:
        node.get_logger().error(f'Map save failed; artifacts may be incomplete: {exc}')
    finally:
        try:
            node.close()
        finally:
            rclpy.try_shutdown()
    raise SystemExit(code)
