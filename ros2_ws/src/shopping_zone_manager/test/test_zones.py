from pathlib import Path
import pytest
import yaml
from shopping_zone_manager.zones import load_zones, lookup, nearest


CONFIG = Path(__file__).parents[1]/'config/zones.yaml'


def test_lookup_and_nearest():
    zones = load_zones(CONFIG)
    assert len(zones) == 6
    assert lookup(zones, 'DRINKS') == lookup(zones, '飲料區')
    assert lookup(zones, ' beverages ').name == 'drinks'
    assert nearest(zones, 0, -5).name == 'entrance'
    with pytest.raises(ValueError, match='drinks.*snacks'):
        lookup(zones, 'missing')
    with pytest.raises(ValueError):
        nearest([], 0, 0)


@pytest.mark.parametrize('entry', [
    {'name': 'a', 'x': float('nan'), 'y': 0, 'yaw': 0, 'frame_id': 'map'},
    {'name': 'a', 'x': 0, 'y': 0, 'yaw': 0, 'frame_id': ''},
    {'name': 'a', 'x': 0, 'y': 0, 'yaw': 0, 'frame_id': 'map', 'aliases': 'bad'},
    {'name': 'a', 'x': 0, 'y': 0, 'yaw': 0, 'frame_id': 'map', 'aliases': ['A']},
    {'name': 'a'}, None])
def test_invalid_zone(tmp_path, entry):
    path = tmp_path/'zones.yaml'
    path.write_text(yaml.safe_dump({'zones': [entry]}))
    with pytest.raises(ValueError):
        load_zones(path)


def test_duplicate_alias_and_empty_file(tmp_path):
    data = yaml.safe_load(CONFIG.read_text())
    data['zones'][1]['aliases'].append('DRINKS')
    path = tmp_path/'zones.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_zones(path)
    path.write_text('')
    with pytest.raises(ValueError):
        load_zones(path)
