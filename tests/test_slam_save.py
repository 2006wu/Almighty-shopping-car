"""Reject false-positive map saves without requiring ROS imports."""
import ast
import os
import tempfile
import yaml
from pathlib import Path
from types import SimpleNamespace as NS
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'ros2_ws/src/shopping_zone_manager/shopping_zone_manager/map_tour.py'


@pytest.mark.parametrize('failure', [None, 'service', 'response', 'missing', 'empty', 'stale'])
def test_save_maps_checks_service_response_and_all_artifacts(tmp_path, failure):
    prefix = str(tmp_path/'map')
    destroyed = []
    if failure == 'stale':
        for suffix in ('.yaml', '.pgm', '.posegraph', '.data'):
            Path(prefix+suffix).write_bytes(b'previous snapshot')

    def save_grid(command, **kwargs):
        staged = command[command.index('-f')+1]
        for suffix in ('.yaml', '.pgm', '.posegraph', '.data'):
            if failure == 'stale' and suffix in ('.posegraph', '.data'):
                continue
            if failure == 'missing' and suffix == '.data':
                continue
            Path(staged+suffix).write_bytes(b'' if failure == 'empty' and suffix == '.data' else
                                          b'image: test.pgm\n' if suffix == '.yaml' else b'test')

    client = NS(wait_for_service=lambda **kwargs: failure != 'service', call_async=lambda req: req)
    node = NS(create_client=lambda *args: client, destroy_client=destroyed.append,
              wait=lambda future, deadline: NS(result=1 if failure == 'response' else 0))
    service = NS(Request=NS, Response=NS(RESULT_SUCCESS=0))
    tree = ast.parse(SOURCE.read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in ('save_maps', '_save_map_artifacts')]
    scope = dict(Path=Path, time=time, os=os, tempfile=tempfile, yaml=yaml,
                 SerializePoseGraph=service, subprocess=NS(run=save_grid))
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(SOURCE), 'exec'), scope)
    if failure:
        with pytest.raises(RuntimeError):
            scope['save_maps'](node, prefix)
    else:
        scope['save_maps'](node, prefix)
    assert destroyed == [client]
    assert not list(tmp_path.glob('.shopping-map-*'))
    if failure == 'stale':
        assert all(Path(prefix+suffix).read_bytes() == b'previous snapshot'
                   for suffix in ('.yaml', '.pgm', '.posegraph', '.data'))
    elif not failure:
        assert yaml.safe_load(Path(prefix+'.yaml').read_text())['image'] == 'map.pgm'
