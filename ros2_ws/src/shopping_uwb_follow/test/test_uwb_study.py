"""Regression coverage for offline geometry, rejection and export semantics."""
import csv
import io
import math
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from shopping_uwb_follow.uwb_study import (
    ANCHOR_SETS, CSV_FIELDS, Noise, evaluate_static, evaluate_trajectory,
    relative_trajectory, render_csv, render_markdown, run_study, static_points,
)

SRC = Path(__file__).resolve().parents[2]
ROOT = SRC.parents[1]  # repository root; absent when colcon tests the mounted /ws/src


@pytest.fixture
def path():
    return yaml.safe_load((SRC/'shopping_robot_gazebo/config/person_path.yaml').read_text())


def test_seed_reproduces_all_exports(path):
    first = run_study(path, trials=1, seed=83, quick=True)
    second = run_study(path, trials=1, seed=83, quick=True)
    assert first == second
    assert render_csv(first) == render_csv(second)
    assert render_markdown(first, 1, 83, 0) == render_markdown(second, 1, 83, 0)


def test_exact_three_anchor_geometry():
    result = evaluate_static(static_points(True), noise=Noise(noise_std=0), trials=2)
    assert result['aggregate']['error_max'] < 1e-10
    assert result['aggregate']['invalid_rate'] == 0
    assert result['aggregate']['flip_rate'] == 0
    assert len(result['points']) == 18


def test_two_anchor_ambiguity_and_optimistic_prior():
    points = [(-1, .1), (-2, -.4), (1, .2)]
    two = evaluate_static(points, ANCHOR_SETS['two'], Noise(noise_std=0), trials=3)
    seeded = evaluate_static(points, ANCHOR_SETS['two'], Noise(noise_std=0),
                             trials=3, seeded=True)
    three = evaluate_static(points, noise=Noise(noise_std=.001), trials=3)
    assert two['aggregate']['flip_rate'] == pytest.approx(2/3)
    assert two['points'][0]['flips'] == 3
    assert seeded['aggregate']['error_max'] < 1e-10
    assert three['aggregate']['flip_rate'] == 0


def test_dropout_nlos_and_empty_statistics():
    points = [(1, .1), (2, -.3)]
    baseline = evaluate_static(points, trials=100)['aggregate']
    dropout = evaluate_static(points, noise=Noise(dropout=.3), trials=100)['aggregate']
    nlos = evaluate_static(points, noise=Noise(nlos_probability=.2), trials=100)['aggregate']
    assert dropout['invalid_rate'] > baseline['invalid_rate']
    assert nlos['error_rmse'] > baseline['error_rmse']
    empty = evaluate_static(points, noise=Noise(dropout=1), trials=2)['aggregate']
    assert empty['invalid_rate'] == 1
    assert empty['error_rmse'] is None and empty['flip_rate'] is None
    assert empty['attempts'] == 4 and empty['valid'] == 0


def test_quality_rejection_and_flip_denominator():
    rejected = evaluate_static([(1, 0)], threshold=1.1, trials=2)['aggregate']
    assert rejected['valid'] == 0 and rejected['invalid_rate'] == 1
    lateral = evaluate_static([(0, 1)], noise=Noise(noise_std=0), trials=2)['aggregate']
    assert lateral['valid'] == 2 and lateral['flip_eligible'] == 0
    assert lateral['flip_rate'] is None


def test_trajectory_lag_and_unfiltered_truth(path):
    result = evaluate_trajectory(path, rate=10)
    assert math.isfinite(result['lag_s']) and 0 <= result['lag_s'] <= .5
    exact = evaluate_trajectory(path, noise=Noise(noise_std=0), rate=5, alpha=1)
    assert exact['error_max'] < 1e-9
    assert exact['lag_s'] == 0
    missing = evaluate_trajectory(path, noise=Noise(dropout=1), rate=5)
    assert missing['lag_s'] is None and missing['invalid_rate'] == 1
    trajectory = relative_trajectory(path, 10)
    # The first corner is only 0.9 m from the path origin.
    assert trajectory[0] == pytest.approx((.9, .1))
    assert any(abs(y) > .1 for _, y in trajectory)


def test_markdown_sections_and_csv_schema(path):
    rows = run_study(path, trials=1, quick=True)
    markdown = render_markdown(rows, 1, 42, 1.23, quick=True)
    for section in ('方法與重現', '錨點配置與靜態掃描', '軌跡、更新率與濾波延遲',
                    '結論與建議參數', '假設與限制', '2027 年 2 月後實車重測', '待驗證'):
        assert section in markdown
    reader = csv.DictReader(io.StringIO(render_csv(rows)))
    assert tuple(reader.fieldnames) == CSV_FIELDS
    parsed = list(reader)
    assert len(parsed) == len(rows)
    assert any(r['scope'] == 'point' for r in parsed)
    assert {'0.01', '0.02', '0.05', '0.1'} == {
        r['case'] for r in rows if r['group'] == 'noise_std'}
    assert len({r['case'] for r in rows if r['group'] == 'nlos'}) == 8


@pytest.mark.skipif(not (ROOT/'scripts/eval/uwb_study.py').exists(), reason='repository scripts not mounted')
def test_cli_quick(tmp_path):
    md, csv_path = tmp_path/'report.md', tmp_path/'report.csv'
    result = subprocess.run([sys.executable, str(ROOT/'scripts/eval/uwb_study.py'), '--quick',
                             '--out-md', str(md), '--out-csv', str(csv_path)],
                            capture_output=True, text=True, check=True)
    assert 'runtime:' in result.stdout
    assert 'UWB 錨點' in md.read_text()
    assert csv_path.read_text().splitlines()[0] == ','.join(CSV_FIELDS)


@pytest.mark.parametrize('trials', [0, -1])
def test_invalid_trials(trials):
    with pytest.raises(ValueError):
        evaluate_static([(1, 0)], trials=trials)
