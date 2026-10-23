"""Seeded offline sensitivity study using the production ranging/estimation logic."""
import csv
import io
import math
from dataclasses import asdict, dataclass

from .estimator import TargetFilter, trilaterate
from .person_path import path_position
from .uwb_model import Anchor, DEFAULT_ANCHORS, RangeSimulator


ANCHOR_SETS = {
    'two': DEFAULT_ANCHORS[:2],
    'default': DEFAULT_ANCHORS,
    'wide': (Anchor(0, 0.3), Anchor(0, -0.3), Anchor(-0.35, 0)),
    'front': (Anchor(0, 0.22), Anchor(0, -0.22), Anchor(0.25, 0)),
}


@dataclass(frozen=True)
class Noise:
    noise_std: float = 0.02
    bias: float = 0.0
    nlos_probability: float = 0.0
    nlos_bias: float = 0.5
    dropout: float = 0.0


def _summary(values, prefix):
    ordered = sorted(values)
    if not ordered:
        return {prefix + key: None for key in ('mean', 'rmse', 'p95', 'max')}
    return {prefix + 'mean': sum(values)/len(values),
            prefix + 'rmse': math.sqrt(sum(v*v for v in values)/len(values)),
            prefix + 'p95': ordered[math.ceil(.95*len(values))-1],
            prefix + 'max': ordered[-1]}


def _metrics(errors, bearings, attempts, eligible, flips):
    return dict(_summary(errors, 'error_'), **_summary(bearings, 'bearing_'),
                attempts=attempts, valid=len(errors), invalid_rate=1-len(errors)/attempts,
                flip_eligible=eligible, flips=flips,
                flip_rate=flips/eligible if eligible else None)


def _error(value, truth):
    angle = value.bearing-math.atan2(truth[1], truth[0])
    return (math.hypot(value.x-truth[0], value.y-truth[1]),
            abs(math.degrees(math.atan2(math.sin(angle), math.cos(angle)))))


def _estimate(simulator, truth, previous, threshold):
    try:
        value = trilaterate(simulator.anchors, simulator.sample(*truth), previous)
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in vars(value).values()) or value.quality < threshold:
        return None
    return value


def static_points(quick=False):
    return [(r*math.cos(math.radians(b)), r*math.sin(math.radians(b)))
            for r in ((.6, 2, 4) if quick else (.6, 1, 2, 3, 4))
            for b in range(-180, 180, 60 if quick else 15)]


def evaluate_static(points, anchors=DEFAULT_ANCHORS, noise=Noise(), trials=40,
                    seed=42, seeded=False, threshold=.15):
    """Independent trials; seeded=True supplies an optimistic exact prior per trial."""
    if trials < 1 or not points:
        raise ValueError('Require points and positive trials')
    simulator = RangeSimulator(anchors, seed=seed, **asdict(noise))
    all_errors, all_bearings, rows = [], [], []
    total_eligible = total_flips = 0
    for x, y in points:
        errors, bearings = [], []
        eligible = flips = 0
        for _ in range(trials):
            value = _estimate(simulator, (x, y), (x, y) if seeded else None, threshold)
            if value is None:
                continue
            error, bearing = _error(value, (x, y))
            errors.append(error)
            bearings.append(bearing)
            if abs(x) > .2:
                eligible += 1
                flips += int(value.x*x < 0)
        rows.append(dict(x=x, y=y, **_metrics(errors, bearings, trials, eligible, flips)))
        all_errors.extend(errors)
        all_bearings.extend(bearings)
        total_eligible += eligible
        total_flips += flips
    return dict(points=rows, aggregate=_metrics(all_errors, all_bearings,
                len(points)*trials, total_eligible, total_flips))


def relative_trajectory(path, rate):
    """One closed path lap after warmup; robot follows one metre of travel behind."""
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('Require positive update rate')
    speed = float(path['speed'])
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError('Require positive path speed')
    points = path['waypoints']
    duration = sum(math.dist(a, b) for a, b in zip(points, points[1:]+points[:1]))/speed
    delay = 1.0/speed
    result = []
    for i in range(math.ceil(duration*rate)):
        t = i/rate
        robot = path_position(path, t)
        ahead = path_position(path, t+.001)
        heading = math.atan2(ahead[1]-robot[1], ahead[0]-robot[0])
        person = path_position(path, t+delay)
        dx, dy = person[0]-robot[0], person[1]-robot[1]
        result.append((dx*math.cos(heading)+dy*math.sin(heading),
                       -dx*math.sin(heading)+dy*math.cos(heading)))
    return result


def evaluate_trajectory(path, anchors=DEFAULT_ANCHORS, noise=Noise(), rate=10,
                        seed=42, alpha=.6, threshold=.15, repeats=1):
    """Replay node prior, rejection, EMA and timeout; score newly published samples."""
    if repeats < 1:
        raise ValueError('Require positive repeats')
    truth = relative_trajectory(path, rate)
    errors, bearings, observed = [], [], []
    eligible = flips = 0
    for repeat in range(repeats):
        simulator = RangeSimulator(anchors, seed=seed+repeat, **asdict(noise))
        target_filter = TargetFilter(alpha=alpha)
        for i, point in enumerate(truth):
            previous = target_filter.get(i/rate)
            value = _estimate(simulator, point,
                              (previous.x, previous.y) if previous else None, threshold)
            if value is None:
                continue
            value = target_filter.update(value, i/rate)
            error, bearing = _error(value, point)
            errors.append(error)
            bearings.append(bearing)
            observed.append((i, value.x, value.y))
            if abs(point[0]) > .2:
                eligible += 1
                flips += int(value.x*point[0] < 0)
    # Use identical accepted indices at every shift, avoiding missing-data bias.
    max_shift = math.floor(.5*rate)
    common = [(i, x, y) for i, x, y in observed if i >= max_shift]
    lag = None
    if common:
        costs = [sum((x-truth[i-k][0])**2+(y-truth[i-k][1])**2
                     for i, x, y in common)/len(common) for k in range(max_shift+1)]
        lag = min(range(len(costs)), key=costs.__getitem__)/rate
    return dict(_metrics(errors, bearings, len(truth)*repeats, eligible, flips), lag_s=lag)


def run_study(path, trials=40, seed=42, quick=False):
    """One-factor-at-a-time sweeps, not a Cartesian product of noise factors."""
    points = static_points(quick)
    rows = []
    cases = [('anchors', name, anchors, seeded, Noise(), .15)
             for name, anchors, seeded in [
                 ('two', ANCHOR_SETS['two'], False),
                 ('two_seeded', ANCHOR_SETS['two'], True),
                 *[(name, ANCHOR_SETS[name], False) for name in ('default', 'wide', 'front')]]]
    sweeps = [('noise_std', (.01, .02, .05, .1)), ('bias', (0, .05, .1)),
              ('dropout', (0, .1, .3))]
    for field, values in sweeps:
        for value in values:
            cases.append((field, str(value), DEFAULT_ANCHORS, False,
                          Noise(**{field: value}), .15))
    for bias in (.3, .5):
        for probability in (0, .05, .1, .2):
            cases.append(('nlos', f'{probability}/{bias}', DEFAULT_ANCHORS, False,
                          Noise(nlos_probability=probability, nlos_bias=bias), .15))
    for threshold in (.15, .3, .5):
        cases.append(('quality', str(threshold), DEFAULT_ANCHORS, False,
                      Noise(nlos_probability=.1), threshold))
    for group, case, anchors, seeded, noise, threshold in cases:
        result = evaluate_static(points, anchors, noise, trials, seed, seeded, threshold)
        metadata = dict(kind='static', group=group, case=case, seed=seed,
                        anchors=';'.join(f'{a.x}:{a.y}' for a in anchors),
                        seeded=seeded, threshold=threshold, **asdict(noise))
        rows.append(dict(metadata, scope='aggregate', **result['aggregate']))
        rows.extend(dict(metadata, scope='point', **point) for point in result['points'])
    for rate in (5, 10, 20):
        for alpha in (.3, .6, 1.0):
            for dropout in (0, .3):
                noise = Noise(dropout=dropout)
                metrics = evaluate_trajectory(path, noise=noise, rate=rate, seed=seed,
                                              alpha=alpha, repeats=min(trials, 2 if quick else 5))
                rows.append(dict(kind='trajectory', group='trajectory',
                                 case=f'{rate}/{alpha}/{dropout}', scope='aggregate',
                                 seed=seed, anchors=';'.join(f'{a.x}:{a.y}' for a in DEFAULT_ANCHORS),
                                 seeded=False, threshold=.15, rate_hz=rate, alpha=alpha,
                                 **asdict(noise), **metrics))
    return rows


CSV_FIELDS = ('kind', 'group', 'case', 'scope', 'seed', 'anchors', 'seeded',
              'threshold', 'noise_std', 'bias', 'nlos_probability', 'nlos_bias',
              'dropout', 'rate_hz', 'alpha', 'x', 'y', 'attempts', 'valid',
              'error_mean', 'error_rmse', 'error_p95', 'error_max',
              'bearing_mean', 'bearing_rmse', 'bearing_p95', 'bearing_max',
              'flip_eligible', 'flips', 'flip_rate', 'invalid_rate', 'lag_s')


def render_csv(rows):
    output = io.StringIO(newline='')
    writer = csv.DictWriter(output, fieldnames=CSV_FIELDS, lineterminator='\n')
    writer.writeheader()
    writer.writerows(rows)
    return output.getvalue()


def _number(value):
    return '無有效樣本' if value is None else f'{value:.4f}'


def render_markdown(rows, trials, seed, elapsed, quick=False):
    lines = ['# UWB 錨點與噪聲離線研究', '', '## 方法與重現', '',
             f'種子 {seed}；每靜態點 {trials} 次獨立試驗；模式：{"快速" if quick else "完整"}；單次實測執行時間 {elapsed:.2f} 秒（依主機而異）。',
             '以 Python 標準函式庫與 PyYAML 執行，不需 ROS、Docker 或硬體。',
             '`/workspace/scratch/venv-shop/bin/python scripts/uwb_study.py --trials '
             f'{trials} --seed {seed}' + (' --quick' if quick else '') + '`', '',
             '重用 RangeSimulator、trilaterate 與 TargetFilter；品質低於門檻（預設 0.15）、ValueError 或非有限解皆拒收。',
             '靜態點位於 base_link；網格涵蓋 0.6、1、2、3、4 公尺與 -180 至 165 度、每 15 度一點（快速模式為三個半徑、每 60 度）；每點等權。',
             '採單因子掃描，未涵蓋所有因子的交互作用。相同種子重設亂數流；掉訊會改變後續亂數消耗，並非逐筆配對比較。',
             '誤差僅計算通過門檻的估測；角度為最短絕對角差（度），p95 為最近秩百分位數。無效率分母為全部嘗試；翻側率分母為通過門檻且真值 |x| > 0.2 公尺的樣本，x 符號錯誤即翻側。',
             'CSV 保留每點與彙總的平均值、RMSE、p95、最大值、有效數及翻側分母；表格位置誤差單位為公尺，率以 0–1 表示。', '',
             '## 錨點配置與靜態掃描', '',
             'two：(0, ±0.22)；default：再加 (-0.25, 0)；wide：(0, ±0.30)、(-0.35, 0)；front：前兩點相同、第三點為 (0.25, 0)。',
             'two_seeded 每次試驗都給精確真值作為先前位置，是已知正確側的樂觀上界，並非可實作的初始化方法。未種子化的兩錨點固定選前方解。', '',
             '| 因子 | 設定 | 平均 | RMSE | p95 | 最大 | 角度 RMSE | 翻側率 | 無效率 |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for row in rows:
        if row['kind'] == 'static' and row['scope'] == 'aggregate':
            lines.append('| ' + ' | '.join([row['group'], row['case']] +
                         [_number(row[key]) for key in ('error_mean', 'error_rmse', 'error_p95',
                          'error_max', 'bearing_rmse', 'flip_rate', 'invalid_rate')]) + ' |')
    lines += ['', '## 軌跡、更新率與濾波延遲', '',
              '唯一來源為 shopping_robot_gazebo/config/person_path.yaml；使用其速度 0.35 m/s 與完整閉合折線。',
              '合成車體沿相同路徑落後人物 1/速度秒，以車體當下路徑切線為朝向。直線相距 1 公尺，轉角時直線距離會縮短且車體朝向瞬間改變；這不是動力學或避障模擬。',
              f'每組重播 {min(trials, 2 if quick else 5)} 次，種子逐次加一；初始無先前位置，之後以有效濾波值選解，逾時 0.5 秒即失效。拒收不更新濾波，也不計入已發布樣本的誤差。',
              '延遲以相同有效時間索引，搜尋 0 至 0.5 秒內使平方誤差最小的真值回移量；解析度為 1/更新率，零值不表示沒有次取樣延遲。', '',
              '| Hz／alpha／掉訊率 | 平均 | RMSE | p95 | 最大 | 角度 RMSE | 翻側率 | 無效率 | 延遲秒 |',
              '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for row in rows:
        if row['kind'] == 'trajectory':
            lines.append('| ' + ' | '.join([row['case']] + [_number(row[key]) for key in
                         ('error_mean', 'error_rmse', 'error_p95', 'error_max', 'bearing_rmse',
                          'flip_rate', 'invalid_rate', 'lag_s')]) + ' |')
    lines += ['', '## 結論與建議參數', '',
              '以下依本次輸出比較，並非實車規格或安全保證。', '']
    lookup = {(r['group'], r['case']): r for r in rows if r['scope'] == 'aggregate'}
    two, three, wide = [lookup['anchors', name] for name in ('two', 'default', 'wide')]
    q15, q50 = [lookup['quality', name] for name in ('0.15', '0.5')]
    nlos = lookup['nlos', '0.1/0.5']
    dropout = lookup['dropout', '0.3']
    trajectory = [lookup['trajectory', name] for name in ('5/0.6/0', '10/0.6/0', '20/0.6/0')]
    alpha_low = lookup['trajectory', '10/0.3/0']
    lines += [f'- 兩錨點未初始化翻側率為 {two["flip_rate"]:.2%}，三錨點為 {three["flip_rate"]:.2%}；兩圓交點的前後歧義支持採三錨點。三錨點仍可能因噪聲或掉訊翻側。',
              f'- 預設三角形 RMSE {three["error_rmse"]:.4f} 公尺，擴大配置為 {wide["error_rmse"]:.4f} 公尺。應依表格比較；wide 的 ±0.30 與後方 -0.35 公尺超出約 0.6 × 0.5 公尺車體的部分範圍，不能直接視為可安裝方案。',
              f'- NLOS 機率 10%、正偏差 0.5 公尺時 RMSE 為 {nlos["error_rmse"]:.4f} 公尺；掉訊 30% 時為 {dropout["error_rmse"]:.4f} 公尺、無效率 {dropout["invalid_rate"]:.2%}。三錨點掉一筆仍可能回退兩錨點解，不能只看拒收率。',
              f'- 暫保留品質門檻 0.15：在品質掃描的 NLOS 條件下，提高至 0.5 僅將 RMSE 從 {q15["error_rmse"]:.4f} 降至 {q50["error_rmse"]:.4f} 公尺，無效率由 {q15["invalid_rate"]:.2%} 升至 {q50["invalid_rate"]:.2%}。兩錨點回退解可能殘差很低；品質門檻不能消除歧義或保證排除 NLOS。',
              f'- 暫保留 alpha=0.6：無掉訊的 5/10/20 Hz 軌跡 RMSE 依序為 {trajectory[0]["error_rmse"]:.4f}／{trajectory[1]["error_rmse"]:.4f}／{trajectory[2]["error_rmse"]:.4f} 公尺；10 Hz、alpha=0.3 為 {alpha_low["error_rmse"]:.4f} 公尺、延遲 {alpha_low["lag_s"]:.2f} 秒。建議至少 10 Hz 作為後續整合起點，20 Hz 若資源允許可優先測試；這不是最低安全頻率的驗證。',
              '- 本次不修改執行期 YAML、xacro、品質門檻或濾波預設值。', '',
              '## 假設與限制', '',
              '僅二維、未建模標籤高度；獨立高斯測距噪聲、各錨點共同固定偏差，NLOS 為獨立事件的固定正偏差，掉訊亦獨立。沒有人體遮蔽、貨架多重路徑、相關噪聲、天線方向性或封包時間差模型。',
              '非線性估測可能產生極大但有限的誤差；最大值與無效率均需一起解讀。合成相對軌跡不含輪速、里程計、TF 誤差、控制器、通訊排程或 ROS 延遲，也不評估過期目標保持期間的控制安全。',
              '樣本有限，未提供信賴區間；單次主機計時不是效能保證，不能以靜態研究取代 Gazebo 或硬體驗收。', '',
              '## 2027 年 2 月後實車重測', '',
              '確認三錨點可安裝座標與高度、天線方向及標籤配戴方式；逐距離／角度量測偏差、標準差、相關性、掉訊連續長度與有效更新率。',
              '量測人體遮蔽、貨架 NLOS 與多重路徑的偏差分布及持續時間，再重跑掃描；核對品質與真實誤差的關係、初始化／掉訊恢復、濾波延遲及速度下的安全停車距離。', '',
              '## 審查驗證', '', '| 項目 | 結果 |', '| --- | --- |',
              '| 離線方法、資料與結論審查 | 待驗證 |', '| Gazebo 整合與實車／貨架 NLOS | 待驗證 |', '']
    return '\n'.join(lines)
