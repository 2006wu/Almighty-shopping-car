"""Pure reproducibility, provenance, CLI and multi-run regression tests."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest

from shopping_zone_manager.eval_repro import (
    bag_name, canonical_report, changed_keys, json_text,
)
from shopping_zone_manager.eval_summary import aggregate, main as summary_main
from shopping_zone_manager.eval_verify import main as verify_main, verify


def repo_root():
    root = Path(__file__).resolve().parents[4]
    if (root/'docs').is_dir():
        return root
    if Path('/repo/docs').is_dir():
        return Path('/repo')
    pytest.skip('Repository assets are not mounted')


def recorder():
    spec = importlib.util.spec_from_file_location('record_bag', repo_root()/'scripts/record_bag.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_manifest_metadata_and_git_fallback(monkeypatch):
    module = recorder()
    assert module.metadata_pair('launch_args=a:=1 b:=2') == ('launch_args', 'a:=1 b:=2')
    for value in ('bad-key=x', '1key=x', '=x', 'key', 'a.b=x'):
        with pytest.raises(argparse.ArgumentTypeError):
            module.metadata_pair(value)
    def unavailable(*args, **kwargs):
        raise FileNotFoundError('git')
    monkeypatch.setattr(module.subprocess, 'run', unavailable)
    git = module.git_provenance()
    assert git == dict(commit='unknown', dirty='unknown', submodule_commit='unknown')
    result = module.build_manifest(['/clock', '/clock', '/odom'], 10,
                                   [('scenario', 'follow'), ('scenario', 'guide')],
                                   '2026-10-24T00:00:00+00:00', git)
    assert result['topics'] == ['/clock', '/odom']
    assert result['metadata'] == {'scenario': 'guide'}
    assert result['duration'] == 10
    assert result['start_time_utc'].endswith('+00:00')


def test_git_provenance_uses_repo_and_handles_partial_failure(monkeypatch):
    module = recorder()
    responses = iter(['abc', ' M file', 'def', ' def third_party/eurobot_nav2'])
    def run(command, **kwargs):
        assert command[:3] == ['git', '-C', '/repo']
        return subprocess.CompletedProcess(command, 0, next(responses))
    monkeypatch.setattr(module.subprocess, 'run', run)
    assert module.git_provenance() == dict(commit='abc', dirty=True, submodule_commit='def')
    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired('git', 3)
    monkeypatch.setattr(module.subprocess, 'run', timeout)
    assert module.git_provenance()['commit'] == 'unknown'


def test_canonicalisation_rounding_provenance_and_diff():
    left = dict(b=1.12345678912, a={'count': 2}, source={'log': '/old'}, generated_at_utc='old')
    right = dict(a={'count': 2}, b=1.12345678914, source={'log': '/tmp/new'})
    assert canonical_report(left) == canonical_report(right)
    assert json_text({'z': -0.0, 'a': 1.00000000001}) == '{\n  "a": 1.0,\n  "z": 0.0\n}\n'
    assert changed_keys(left, dict(right, a={'count': 3}, new=True)) == ['a.count', 'new']
    with pytest.raises(ValueError):
        canonical_report({'bad': float('nan')})


def test_aggregation_counts_statistics_verdicts_and_warnings():
    manifest = dict(git={'commit': 'abc'}, metadata={'scenario': 'guide', 'nav2_stack': 'stock'})
    other = dict(git={'commit': 'def'}, metadata={'scenario': 'follow', 'nav2_stack': 'fork'})
    runs = [('a', dict(metric=1, missing=None, verdict='PASS', flag=True), manifest),
            ('b', dict(metric=3, verdict='fail'), other),
            ('c', dict(metric=None, verdict=True), None)]
    result = aggregate(runs)
    assert result['metrics'] == {'metric': dict(count=2, mean=2, min=1, max=3,
                                               stdev=1.414213562)}
    assert result['verdicts']['verdict'] == dict(pass_count=2, fail_count=1)
    assert len(result['warnings']) == 4
    assert aggregate(list(reversed(runs))) == result
    assert aggregate([('a', {'x': 1}, manifest)])['metrics']['x']['stdev'] == 0
    assert len(aggregate([('a', {}, {})])['warnings']) == 3
    with pytest.raises(ValueError):
        aggregate([('a', {'x': float('inf')}, manifest)])


def test_summary_cli_files_and_validation(tmp_path):
    bag = tmp_path/'run'
    bag.mkdir()
    (bag/'report.json').write_text('{"metric": 2}')
    output = tmp_path/'output'
    assert summary_main(['run', '--bags-dir', str(tmp_path), '--output', str(output)]) == 0
    assert json.loads((output/'summary.json').read_text())['metrics']['metric']['mean'] == 2
    assert '缺少 manifest.json' in (output/'summary.md').read_text()
    for args in ([], ['../bad'], ['run', 'run'], ['absent', '--bags-dir', str(tmp_path)]):
        with pytest.raises(SystemExit) as exc:
            summary_main(args)
        assert exc.value.code == 2
    for value in ('../x', '/tmp/x', '', 'a.b', 'a b'):
        with pytest.raises(argparse.ArgumentTypeError):
            bag_name(value)
        with pytest.raises(SystemExit) as exc:
            verify_main([value])
        assert exc.value.code == 2
    assert bag_name('run_01-a') == 'run_01-a'


def test_verify_reconverts_in_temp_with_stored_settings_and_no_writes(tmp_path):
    report = dict(source={'world': '/world.sdf', 'log': '/original'},
                  parameters={'target': 1.2, 'too_close': 0.5}, metric={'mean': 2.0})
    stored = tmp_path/'report.json'
    stored.write_text(json.dumps(report))
    (tmp_path/'observations.jsonl.meta.json').write_text(json.dumps(dict(
        zones='/zones.yaml', max_skew=0.1, map_world_transform=[1, 2, 3])))
    calls = []
    def convert(arguments):
        calls.append(arguments)
        assert arguments[0] == str(tmp_path)
        assert arguments[arguments.index('--zones')+1] == '/zones.yaml'
        assert arguments[-4:] == ['--map-world-transform', '1', '2', '3']
        assert arguments[arguments.index('--max-skew')+1] == '0.1'
        Path(arguments[arguments.index('--output')+1]).write_text('')
    def evaluate(arguments):
        assert arguments[-4:] == ['--target', '1.2', '--too-close', '0.5']
        output = Path(arguments[arguments.index('--output')+1]+'.json')
        output.write_text(json.dumps(dict(report, source={'log': 'temp'})))
    original = stored.read_bytes()
    assert verify(tmp_path, convert, evaluate) == (True, [])
    assert stored.read_bytes() == original
    assert not Path(calls[0][calls[0].index('--output')+1]).exists()
    def changed(arguments):
        evaluate(arguments)
        output = Path(arguments[arguments.index('--output')+1]+'.json')
        output.write_text(json.dumps(dict(report, metric={'mean': 4.0})))
    assert verify(tmp_path, convert, changed) == (False, ['metric.mean'])


def test_dev_cli_rejects_invalid_names_without_docker():
    script = repo_root()/'scripts/dev.sh'
    for args in (['eval-verify', '../bad'], ['eval-verify', 'a', 'b'],
                 ['eval-summary'], ['eval-summary', '../bad'],
                 ['eval-summary', 'good', '--output'], ['eval-summary', '--output', '/tmp']):
        result = subprocess.run(['bash', str(script), *args], capture_output=True)
        assert result.returncode == 2


def test_recorder_writes_manifest_after_graceful_stop(tmp_path, monkeypatch):
    module = recorder()
    root = repo_root()
    # Give the command its own repository tree, without touching real bags.
    script = tmp_path/'scripts/record_bag.py'
    script.parent.mkdir()
    script.write_text('')
    monkeypatch.setattr(module, '__file__', str(script))
    config = tmp_path/'ros2_ws/src/shopping_zone_manager/config'
    config.mkdir(parents=True)
    (config/'record_topics.yaml').write_text(
        (root/'ros2_ws/src/shopping_zone_manager/config/record_topics.yaml').read_text())
    git = dict(commit='unknown', dirty='unknown', submodule_commit='unknown')
    monkeypatch.setattr(module, 'git_provenance', lambda: git)
    signals = []
    class Process:
        def __init__(self, command):
            Path(command[command.index('--output')+1]).mkdir()
        def wait(self, timeout=None):
            if timeout:
                raise subprocess.TimeoutExpired('ros2', timeout)
            return 0
        def poll(self):
            return None
        def send_signal(self, signal):
            signals.append(signal)
    monkeypatch.setattr(module.subprocess, 'Popen', Process)
    assert module.main(['run', '--duration', '1', '--meta', 'scenario=guide',
                        '--meta', 'nav2_stack=stock', '/extra']) == 0
    manifest = json.loads((tmp_path/'bags/run/manifest.json').read_text())
    assert manifest['metadata'] == dict(scenario='guide', nav2_stack='stock')
    assert manifest['git'] == git
    assert manifest['topics'][-1] == '/extra'
    assert manifest['duration'] == 1
    assert signals == [module.signal.SIGINT]
    with pytest.raises(SystemExit):
        module.main(['another', '--meta', 'invalid-key=x'])
    assert not (tmp_path/'bags/another').exists()


def test_verify_cli_exit_codes_and_short_diff(monkeypatch, capsys):
    import shopping_zone_manager.eval_verify as command
    monkeypatch.setattr(command, 'verify', lambda *args: (True, []))
    assert command.main(['run']) == 0
    assert 'matches' in capsys.readouterr().out
    monkeypatch.setattr(command, 'verify', lambda *args: (False, ['metric.mean']))
    assert command.main(['run']) == 1
    assert 'metric.mean' in capsys.readouterr().out
    assert changed_keys({'empty': {}}, {'empty': []}) == ['empty']


def test_high_precision_parameters_reproduce_with_real_evaluator(tmp_path):
    from shopping_zone_manager.eval_report import main as evaluate
    world = repo_root()/'ros2_ws/src/shopping_robot_gazebo/worlds/supermarket.sdf'
    log = tmp_path/'observations.jsonl'
    payload = '{"type":"distance","t":0,"distance":1.8}\n'
    log.write_text(payload)
    assert evaluate([str(log), '--world', str(world), '--output', str(tmp_path/'report'),
                     '--target', '1.123456789123']) == 0
    (tmp_path/'observations.jsonl.meta.json').write_text('{"max_skew": 0.15}')
    def convert(arguments):
        Path(arguments[arguments.index('--output')+1]).write_text(payload)
    assert verify(tmp_path, convert, evaluate) == (True, [])
