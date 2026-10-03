# @file flight_data.py
# @brief 增量读取飞行历史，为图表与 CSV 提供有界缓存、时间对齐和水平误差。

"""Incremental flight history and timestamp-aligned evaluation samples."""

import csv
import math
from bisect import bisect_left
from collections import Counter, deque
from pathlib import Path

from simulation.localization_metrics import JsonlTail, read_status

FIELDS = (
    "time_s",
    "x_m",
    "y_m",
    "z_m",
    "vx_mps",
    "vy_mps",
    "vz_mps",
    "yaw_rad",
    "confidence",
    "flow_valid",
    "xy_valid",
)
# 对齐容差和估计降采样间隔，单位 s；数据使用本次运行的仿真时间轴。
ALIGNMENT_S = 0.1
SAMPLE_PERIOD_S = 0.05


# 读取有限数值并转成 float；字段缺失或不是有限数值时返回 NaN，供绘图保留数据空缺。
def number(row, key):
    value = row.get(key)
    return (
        float(value)
        if isinstance(value, (int, float)) and math.isfinite(value)
        else math.nan
    )


# @brief 将估计采样时刻映射到真值时间轴，供轨迹比较和 CSV 导出使用。
# @param estimates 按时间排列的估计元组，首项为秒，后续位置分量为米。
# @param truths 按时间升序排列的真值元组。
# @return 逐条产生（估计、对齐真值或 None、水平误差米）。
# @note 两侧均不超过 100 ms 时插值 xyz，否则选容差内最近真值；不跨大间隔外推。
def aligned_samples(estimates, truths):
    """Interpolate bracketed truth within 100 ms on either side; gaps stay NaN."""
    times = [row[0] for row in truths]
    for estimate in estimates:
        stamp = estimate[0]
        # 二分定位左右真值；先保留容差内最近帧作为无法插值时的后备。
        index = bisect_left(times, stamp)
        choices = [i for i in (index - 1, index) if 0 <= i < len(times)]
        nearest = min(choices, key=lambda i: abs(times[i] - stamp)) if choices else None
        truth = (
            truths[nearest]
            if nearest is not None and abs(times[nearest] - stamp) <= ALIGNMENT_S
            else None
        )
        # 两侧都足够近时按时间比例插值 xyz；大间隔不外推。
        if 0 < index < len(times):
            before, after = truths[index - 1], truths[index]
            if stamp - before[0] <= ALIGNMENT_S and after[0] - stamp <= ALIGNMENT_S:
                weight = (stamp - before[0]) / (after[0] - before[0])
                truth = (
                    stamp,
                    *(before[j] + weight * (after[j] - before[j]) for j in range(1, 4)),
                )
        # 水平误差只比较世界 XY；缺少对齐真值用 NaN 表达，导出时留空。
        error = (
            math.hypot(estimate[1] - truth[1], estimate[2] - truth[2])
            if truth is not None
            else math.nan
        )
        yield estimate, truth, error


# 保存每架飞机的有界时序和任务事件；由调用方串行 poll 后取快照供界面使用。
# 估计元组字段顺序遵循 FIELDS；真值为时间秒、xyz 米和偏航弧度。
class FlightHistory:
    # @param folder 已存在且包含飞行日志的运行目录。
    # @param max_points 每机每类位置历史的容量；满时丢弃最旧点并累计 dropped。
    def __init__(self, folder, max_points=100000):
        self.folder = Path(folder)
        if not self.folder.is_dir():
            raise ValueError("运行目录不存在")
        self.config = read_status(self.folder / "map.json", 4 * 1024 * 1024)
        self.max_points = max_points
        self.tails = {}
        self.estimates = {}
        self.truths = {}
        self.last_seen = {}
        self.phases = []
        self.impacts = []
        self.fusion = Counter()
        self.ended = set()
        self.dropped = 0
        self.invalid = 0
        self.revision = 0
        self.discover()
        if not self.tails:
            raise ValueError("目录内没有飞行日志")

    # 发现运行过程中新增的日志文件，保留已有文件游标；新文件从头读取。
    def discover(self):
        files = [
            ("truth", self.folder / "truth_evaluation_only.jsonl"),
            ("team", self.folder / "team_mission.jsonl"),
        ]
        for path in self.folder.glob("companion_*.jsonl"):
            suffix = path.stem.removeprefix("companion_")
            if suffix.isdigit():
                files.append((int(suffix), path))
        for key, path in files:
            if key not in self.tails and path.is_file():
                tail = JsonlTail(path)
                tail.full = True
                self.tails[key] = tail

    # 拒绝重复或倒序时间戳，估计轨迹按 50 ms 最小间隔降采样；真值不做该降采样。
    # @return 是否真正追加数据点；last_seen 同时记录被降采样跳过的最新时间。
    def _append(self, kind, vehicle, sample):
        key = (kind, vehicle)
        if sample[0] <= self.last_seen.get(key, -1):
            return False
        self.last_seen[key] = sample[0]
        series = self.estimates if kind == "estimate" else self.truths
        history = series.setdefault(vehicle, deque(maxlen=self.max_points))
        if (
            kind == "estimate"
            and history
            and sample[0] - history[-1][0] < SAMPLE_PERIOD_S - 1e-6
        ):
            return False
        if len(history) == self.max_points:
            self.dropped += 1
        history.append(sample)
        return True

    # 按来源和车辆编号校验一批日志，将仿真微秒转成秒，分类累计位置、阶段、融合和停止事件。
    # @return 是否发生位置或事件更新；仅融合计数变化不设置该返回标志。
    def consume(self, key, rows):
        changed = False
        for row in rows:
            stamp = number(row, "sim_time_us") / 1e6
            if not math.isfinite(stamp) or stamp < 0:
                continue
            kind, vehicle = row.get("kind"), row.get("vehicle")
            if key == "team":
                phase = row.get("mission_phase", row.get("phase"))
                if not isinstance(phase, str):
                    continue
                event = (stamp, phase, row.get("active_vehicle", -1))
                if self.phases and stamp < self.phases[-1][0]:
                    continue
                if not self.phases or event[1:] != self.phases[-1][1:]:
                    self.phases.append(event)
                    changed = True
                continue
            if (
                type(vehicle) is not int
                or vehicle < 0
                or (type(key) is int and key != vehicle)
            ):
                self.invalid += 1
                continue
            if key == "truth":
                if kind != "truth_evaluation_only":
                    continue
                sample = (
                    stamp,
                    *(number(row, field) for field in ("x_m", "y_m", "z_m", "yaw_rad")),
                )
                if not all(math.isfinite(v) for v in sample[:4]):
                    self.invalid += 1
                    continue
                changed |= self._append("truth", vehicle, sample)
                continue
            if row.get("source") != "ESTIMATE":
                continue
            if kind == "estimate":
                sample = (
                    stamp,
                    *(number(row, field) for field in FIELDS[1:8]),
                    number(row, "position_confidence"),
                    float(row.get("flow_valid") is True),
                    float(row.get("xy_valid") is True),
                )
                if not all(math.isfinite(v) for v in sample[:4]):
                    self.invalid += 1
                    continue
                changed |= self._append("estimate", vehicle, sample)
            elif kind == "impact":
                event = (stamp, vehicle)
                if event not in self.impacts:
                    self.impacts.append(event)
                    changed = True
            elif kind == "fusion":
                self.fusion[
                    (
                        vehicle,
                        row.get("sensor_kind", "unknown"),
                        row.get("accepted") is True,
                    )
                ] += 1
            elif kind == "stop":
                self.ended.add(vehicle)
                changed = True
        return changed

    # @param rounds 一次调用最多读取的轮数，用于限制界面刷新时的工作量。
    # @return 是否更新历史；文件替换或截短时重建缓存并增加 revision，避免混入旧运行。
    def poll(self, rounds=4):
        self.discover()
        changed = False
        for _ in range(rounds):
            for key, tail in list(self.tails.items()):
                batch = tail.read()
                if tail.reset:
                    folder, maximum = self.folder, self.max_points
                    revision = self.revision
                    self.__init__(folder, maximum)
                    self.revision = revision + 1
                    return True
                changed |= self.consume(key, batch)
            if not self.backlog:
                break
        if changed:
            self.revision += 1
        return changed

    # 是否仍有任一日志的已观测字节尚未读完。
    @property
    def backlog(self):
        return any(tail.backlog for tail in self.tails.values())

    # 返回图表使用的数据副本及 revision；估计和真值仍分开保存，匹配在消费快照时进行。
    def snapshot(self):
        ids = sorted(set(self.estimates) | set(self.truths))
        return {
            "folder": str(self.folder),
            "config": self.config,
            "ids": ids,
            "estimates": {i: list(self.estimates.get(i, [])) for i in ids},
            "truths": {i: list(self.truths.get(i, [])) for i in ids},
            "phases": list(self.phases),
            "impacts": list(self.impacts),
            "fusion": dict(self.fusion),
            "ended": sorted(self.ended),
            "backlog": self.backlog,
            "dropped": self.dropped,
            "invalid": self.invalid,
            "revision": self.revision,
        }


# @brief 将缓存中的估计、对齐真值和水平误差导出为逐估计采样的 CSV。
# @param snapshot FlightHistory.snapshot 返回的数据，导出范围受历史缓存容量限制。
# @param path CSV 输出路径，以带 BOM 的 UTF-8 覆盖写入。
# @param vehicles 要导出的车辆编号；None 选择快照中的全部车辆。
# @note 每条估计对应一行；无法匹配的真值和非有限数值留空，水平误差单位为米。
def export_csv(snapshot, path, vehicles=None):
    selected = snapshot["ids"] if vehicles is None else vehicles
    with Path(path).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "vehicle",
                *FIELDS,
                "truth_time_s",
                "truth_x_m",
                "truth_y_m",
                "truth_z_m",
                "xy_error_m",
            ]
        )
        for vehicle in selected:
            for estimate, truth, error in aligned_samples(
                snapshot["estimates"].get(vehicle, []),
                snapshot["truths"].get(vehicle, []),
            ):
                values = [
                    vehicle,
                    *estimate,
                    *(truth[:4] if truth else [math.nan] * 4),
                    error,
                ]
                writer.writerow(
                    [
                        "" if isinstance(v, float) and not math.isfinite(v) else v
                        for v in values
                    ]
                )
