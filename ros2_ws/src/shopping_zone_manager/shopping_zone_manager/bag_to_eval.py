"""Thin Humble rosbag reader and bag_to_eval command entry point."""
import argparse
import json
from pathlib import Path

import yaml

from .bag_converter import convert_records


def plain_message(value):
    """Preserve ROS field names while removing generated message classes."""
    if hasattr(value, 'get_fields_and_field_types'):
        return {name: plain_message(getattr(value, name))
                for name in value.get_fields_and_field_types()}
    if hasattr(value, 'tolist'):
        # numpy arrays/scalars and array.array: tolist() yields plain Python values.
        value = value.tolist()
    if isinstance(value, (list, tuple)):
        return [plain_message(item) for item in value]
    return value


def read_records(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    classes = {}
    previous_clock = None
    while reader.has_next():
        topic, data, received_ns = reader.read_next()
        if topic not in classes:
            classes[topic] = get_message(types[topic])
        msg = plain_message(deserialize_message(data, classes[topic]))
        if topic == '/clock':
            clock = msg['clock']['sec']+msg['clock']['nanosec']*1e-9
            if previous_clock is not None and clock < previous_clock:
                raise ValueError('Simulation clock reset; split recordings before reset')
            previous_clock = clock
        stamp = msg.get('header', {}).get('stamp')
        t = stamp['sec']+stamp['nanosec']*1e-9 if stamp is not None else received_ns*1e-9
        yield topic, t, msg


def main(argv=None):
    parser = argparse.ArgumentParser(description='Convert a simulation sqlite3 bag into evaluation JSONL')
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--zones', type=Path)
    parser.add_argument('--map-world-transform', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--pedestrian-radius', type=float, default=0.25)
    parser.add_argument('--max-skew', type=float, default=0.15)
    args = parser.parse_args(argv)
    try:
        zones = yaml.safe_load(args.zones.read_text())['zones'] if args.zones else []
        rows, info = convert_records(read_records(args.bag), zones,
                                     args.map_world_transform, args.max_skew, args.pedestrian_radius)
        info.update(bag=str(args.bag), zones=str(args.zones) if args.zones else None,
                    timestamp_policy='header stamp when present; otherwise bag receive time')
        payload = ''.join(json.dumps(row, allow_nan=False)+'\n' for row in rows)
        metadata = json.dumps(info, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
        args.output.write_text(payload, encoding='utf-8')
        Path(str(args.output)+'.meta.json').write_text(metadata, encoding='utf-8')
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
