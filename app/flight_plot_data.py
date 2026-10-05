"""飞行曲线的后台计算与显示抽样；原始快照和 CSV 数据不在此修改。"""

from threading import Lock

import numpy as np

from simulation.flight_data import ALIGNMENT_S

DISPLAY_POINTS = 1600
EVENT_LIMIT = 256
SMOOTH_WINDOWS = (1, 5, 10, 20)


class LatestSnapshot:
    """单槽交接：新快照覆盖未消费版本，不向 Qt 事件队列逐条发送数据。"""

    def __init__(self):
        self._lock = Lock()
        self._value = None

    def publish(self, value):
        with self._lock:
            self._value = value

    def take(self):
        with self._lock:
            value, self._value = self._value, None
        return value


def smooth(values, window):
    """按原始采样点滑动平均；全缺失窗口保持 NaN，缺失点不计入均值。"""
    if window < 2 or len(values) == 0:
        return values
    padded = np.concatenate([np.full(window - 1, np.nan), values])
    stacked = np.lib.stride_tricks.sliding_window_view(padded, window)
    present = ~np.isnan(stacked)
    count = present.sum(axis=1)
    total = np.where(present, stacked, 0.0).sum(axis=1)
    return np.where(count > 0, total / np.maximum(count, 1), np.nan)


def aligned_errors(estimate, truth):
    """与 aligned_samples 相同的最近邻/插值规则，批量计算全量 XY 误差。"""
    errors = np.full(len(estimate), np.nan)
    if not len(estimate) or not len(truth):
        return errors
    times = estimate[:, 0]
    index = np.searchsorted(truth[:, 0], times, side="left")
    before = np.clip(index - 1, 0, len(truth) - 1)
    after = np.clip(index, 0, len(truth) - 1)
    left_delta = np.abs(times - truth[before, 0])
    right_delta = np.abs(truth[after, 0] - times)
    nearest = np.where(left_delta <= right_delta, before, after)
    valid = np.minimum(left_delta, right_delta) <= ALIGNMENT_S
    xy = truth[nearest, 1:3].copy()
    bracketed = (index > 0) & (index < len(truth))
    interpolate = bracketed & (left_delta <= ALIGNMENT_S) & (right_delta <= ALIGNMENT_S)
    rows = np.flatnonzero(interpolate)
    weight = (times[rows] - truth[before[rows], 0]) / (
        truth[after[rows], 0] - truth[before[rows], 0]
    )
    xy[rows] = truth[before[rows], 1:3] + weight[:, None] * (
        truth[after[rows], 1:3] - truth[before[rows], 1:3]
    )
    errors[valid] = np.linalg.norm(estimate[valid, 1:3] - xy[valid], axis=1)
    return errors


def envelope(x, values, limit):
    """按桶保留各分量极值与缺失标记；输出点数有界，首尾点始终保留。

    桶内存在 NaN 时加入断线点，避免跨缺失段连接；抽样只作用于显示副本。
    """
    values = np.asarray(values)
    if len(x) <= limit:
        return x, values
    matrix = values[:, None] if values.ndim == 1 else values
    per_bucket = 3 * matrix.shape[1]
    buckets = max(1, (limit - 2) // per_bucket)
    boundaries = np.linspace(0, len(x), buckets + 1, dtype=int)
    indices = {0, len(x) - 1}
    for first, last in zip(boundaries[:-1], boundaries[1:]):
        block = matrix[first:last]
        for column in range(block.shape[1]):
            data = block[:, column]
            finite = np.flatnonzero(np.isfinite(data))
            if len(finite):
                indices.add(first + finite[np.argmin(data[finite])])
                indices.add(first + finite[np.argmax(data[finite])])
            missing = np.flatnonzero(~np.isfinite(data))
            if len(missing):
                indices.add(first + missing[0])
    chosen = np.asarray(sorted(indices))
    return x[chosen], values[chosen]


def yaw_curve(yaw):
    """航向跨 ±180° 时断线，避免将角度环绕画成大幅旋转。"""
    result = np.rad2deg(yaw).copy()
    result[np.r_[False, np.abs(np.diff(result)) > 180]] = np.nan
    return result


def prepare_plot(snapshot, cancelled=lambda: False):
    """在读取线程计算完整数组、准确统计及有界绘图副本。

    full 提供准确游标读数，stats 使用所有有效误差；curves 只供绘图。
    快照本身仍由调用方保存，export_csv 继续直接读取原始 estimates/truths。
    """
    full, curves, stats = {}, {}, {}
    limit = max(128, DISPLAY_POINTS // max(1, len(snapshot["ids"])))
    for vehicle in snapshot["ids"]:
        if cancelled():
            return None
        estimate = np.asarray(snapshot["estimates"].get(vehicle, []), dtype=float)
        truth = np.asarray(snapshot.get("truths", {}).get(vehicle, []), dtype=float)
        if not len(estimate):
            continue
        errors = aligned_errors(estimate, truth)
        speed = np.linalg.norm(estimate[:, 4:7], axis=1)
        full[vehicle] = (estimate, truth, errors, speed)
        finite = errors[np.isfinite(errors)]
        stats[vehicle] = (
            (float(np.sqrt(np.mean(finite ** 2))), float(np.percentile(finite, 95)), float(finite.max()))
            if len(finite) else None
        )
        times = estimate[:, 0]
        draw = {
            "xy": envelope(times, estimate[:, 1:3], limit)[1],
            "height": envelope(times, estimate[:, 3], limit),
            "yaw": envelope(times, yaw_curve(estimate[:, 7]), limit),
            "confidence": envelope(times, estimate[:, 8], limit),
        }
        for window in SMOOTH_WINDOWS:
            if cancelled():
                return None
            draw[("speed", window)] = envelope(times, smooth(speed, window), limit)
            draw[("errors", window)] = envelope(times, smooth(errors, window), limit)
            for index in (4, 5, 6):
                draw[(index, window)] = envelope(times, smooth(estimate[:, index], window), limit)
        # 有效性用桶内有效比例表达；短历史仍逐点展示。两个泳道集合的顶点数受限。
        boundaries = np.unique(np.linspace(0, len(times), min(limit, len(times)) + 1, dtype=int))
        counts = np.diff(boundaries)
        validity = np.add.reduceat((estimate[:, 9:11] > 0.5).astype(float), boundaries[:-1])
        draw["validity"] = (times[boundaries[:-1]], validity / counts[:, None])
        if len(truth):
            draw["truth_xy"] = envelope(truth[:, 0], truth[:, 1:3], limit)[1]
            draw["truth_height"] = envelope(truth[:, 0], truth[:, 3], limit)
            draw["truth_yaw"] = envelope(truth[:, 0], yaw_curve(truth[:, 4]), limit)
        curves[vehicle] = draw
    # 事件原件保留；显示时间线限制色块数，并保持每个色块的原始起止时间。
    phases = snapshot.get("phases", [])
    phase_indices = np.unique(np.linspace(0, len(phases) - 1, min(EVENT_LIMIT, len(phases)), dtype=int))
    end = max((data[0][-1, 0] for data in full.values()), default=0.0)
    display_phases = [
        (*phases[i], phases[i + 1][0] if i + 1 < len(phases) else end) for i in phase_indices
    ]
    impacts = snapshot.get("impacts", [])
    impact_indices = np.unique(np.linspace(0, len(impacts) - 1, min(EVENT_LIMIT, len(impacts)), dtype=int))
    return {
        "full": full, "curves": curves, "stats": stats,
        "phases": display_phases, "impacts": [impacts[i] for i in impact_indices],
        "phase_times": np.asarray([row[0] for row in phases]),
    }
