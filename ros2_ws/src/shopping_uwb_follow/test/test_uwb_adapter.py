"""Legacy migration preserves validity and does not fabricate sensor metadata."""
import math
import pytest
from shopping_uwb_follow.uwb_adapter import adapt_sim_ranges


def test_fields_and_unknown_metadata():
    stamp = {'sec': 0, 'nanosec': 5}
    batch = adapt_sim_ranges([1.2, float('nan'), float('inf'), -1, 0],
                             ['a', 'b', 'c', 'd', 'e'], stamp=stamp, range_std_m=0.02)
    samples = batch['ranges']
    assert [r['reason'] for r in samples] == [0, 1, 2, 3, 3]
    assert [r['valid'] for r in samples] == [True, False, False, False, False]
    assert samples[0]['range_m'] == 1.2
    assert math.isnan(samples[1]['range_m'])
    assert samples[0]['range_std_m'] == 0.02
    assert samples[0]['quality'] == -1
    assert not samples[0]['nlos_known']
    assert samples[0]['header']['frame_id'] == 'a'
    stamp['sec'] = 99
    assert samples[0]['header']['stamp']['sec'] == 0
    assert batch['header']['stamp']['sec'] == 0


@pytest.mark.parametrize('ranges,ids', [([1], []), ([1, 2], ['a', 'a']), ([1], ['']), ([], [])])
def test_reject_ambiguous_anchor_mapping(ranges, ids):
    with pytest.raises(ValueError):
        adapt_sim_ranges(ranges, ids, stamp={'sec': 0, 'nanosec': 0})


@pytest.mark.parametrize('stamp', [None, {'sec': 1}, {'sec': -1, 'nanosec': 0},
                                  {'sec': 0, 'nanosec': 10**9}, {'sec': True, 'nanosec': 0}])
def test_reject_invalid_clock(stamp):
    with pytest.raises(ValueError):
        adapt_sim_ranges([1], ['a'], stamp=stamp)


@pytest.mark.parametrize('std', [-2, float('nan'), float('inf')])
def test_reject_invalid_uncertainty(std):
    with pytest.raises(ValueError):
        adapt_sim_ranges([1], ['a'], stamp={'sec': 1, 'nanosec': 0}, range_std_m=std)


def test_adapter_fields_and_reasons_match_message_contract():
    from pathlib import Path
    from shopping_uwb_follow import uwb_adapter
    root = Path(__file__).resolve().parents[4]
    if not (root/'docs').is_dir():
        root = Path('/repo')
    if not (root/'docs').is_dir():
        pytest.skip('Repository contracts are unavailable outside the source checkout and /repo')
    message = root/'ros2_ws/src/shopping_interfaces/msg/UwbRange.msg'
    fields = set()
    for raw in message.read_text().splitlines():
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        kind, name = line.split()
        if '=' in name:
            name, value = name.split('=')
            if hasattr(uwb_adapter, name):
                assert getattr(uwb_adapter, name) == int(value)
        else:
            fields.add(name)
    sample = adapt_sim_ranges([1], ['a'], stamp={'sec': 0, 'nanosec': 0})['ranges'][0]
    assert set(sample) == fields
