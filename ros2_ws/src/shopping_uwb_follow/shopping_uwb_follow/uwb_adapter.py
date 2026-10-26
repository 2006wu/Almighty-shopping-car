"""Map legacy planar simulator samples to message-shaped dictionaries, without ROS."""
import math


REASON_OK = 0
REASON_DROPOUT = 1
REASON_NONFINITE = 2
REASON_OUT_OF_RANGE = 3


def adapt_sim_ranges(ranges, anchor_ids, *, stamp, tag_id='person', range_std_m=-1.0):
    """Use the caller-supplied ROS acquisition stamp; never invent module metadata.

    Legacy ranges are planar, not physical slant distances. This adapter is for
    migration tests/offline export only until a height-aware estimator is added.
    """
    ranges, anchor_ids = list(ranges), list(anchor_ids)
    if (len(ranges) != len(anchor_ids) or not anchor_ids
            or any(not isinstance(a, str) or not a for a in anchor_ids)
            or len(set(anchor_ids)) != len(anchor_ids)):
        raise ValueError('Ranges require unique, nonempty anchor IDs with matching length')
    if not isinstance(tag_id, str) or not tag_id:
        raise ValueError('A nonempty tag ID is required')
    if (not isinstance(stamp, dict) or set(stamp) != {'sec', 'nanosec'}
            or type(stamp['sec']) is not int or type(stamp['nanosec']) is not int
            or not 0 <= stamp['sec'] < 2**31 or not 0 <= stamp['nanosec'] < 10**9):
        raise ValueError('Stamp must be a nonnegative ROS sec/nanosec dictionary')
    if not math.isfinite(range_std_m) or (range_std_m < 0 and range_std_m != -1):
        raise ValueError('Standard deviation must be nonnegative or -1 (unknown)')
    result = []
    for anchor_id, raw in zip(anchor_ids, ranges):
        value = float(raw)
        reason = (REASON_DROPOUT if math.isnan(value) else
                  REASON_NONFINITE if not math.isfinite(value) else
                  REASON_OUT_OF_RANGE if value <= 0 else REASON_OK)
        result.append({'header': {'stamp': dict(stamp), 'frame_id': anchor_id},
                       'anchor_id': anchor_id, 'tag_id': tag_id,
                       'range_m': value, 'range_std_m': float(range_std_m),
                       'quality': -1.0, 'valid': reason == REASON_OK,
                       'reason': reason, 'nlos': False, 'nlos_known': False})
    return {'header': {'stamp': dict(stamp), 'frame_id': 'base_link'}, 'ranges': result}
