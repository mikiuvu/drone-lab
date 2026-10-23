"""从 A、B、C 工作包的原始 JSON 生成预研报告的图表、Markdown 与 PDF。

数据来源固定为三处，缺失即报错，不生成占位数字：

* A：simulation/research/scenarios.json，场景噪声配置与未知机器人占位。
* B：logs/research-20261023/<运行目录>/，板端测量 JSON 与 summary.json。
* C：build/research-20261023/localization-*.json，同输入配对定位实验。

用法：

    python scripts/build_research_20261023_report.py check
    python scripts/build_research_20261023_report.py figures
    python scripts/build_research_20261023_report.py markdown
    python scripts/build_research_20261023_report.py pdf
    python scripts/build_research_20261023_report.py all

图表阶段使用仓库 .venv 的 matplotlib，PDF 阶段使用 workspace 依赖运行时中的 reportlab，
两个阶段相互独立，可用不同解释器分别执行。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

SCENARIO_ORDER = ["ideal", "noise", "occluders", "combined", "adverse", "blackout"]
SCENARIO_LABELS = {
    "ideal": "理想静态",
    "noise": "仅测距噪声",
    "occluders": "仅未知占位",
    "combined": "噪声与占位",
    "adverse": "恶劣测距",
    "blackout": "局部 ToF 暂停",
}
SCENARIO_ALIASES = {
    "ideal": "ideal",
    "baseline": "ideal",
    "clean": "ideal",
    "noise": "noise",
    "nominal": "noise",
    "range_noise": "noise",
    "occluder": "occluders",
    "occluders": "occluders",
    "occlusion": "occluders",
    "robots": "occluders",
    "unknown_robots": "occluders",
    "combined": "combined",
    "noise_occluders": "combined",
    "adverse": "adverse",
    "severe": "adverse",
    "blackout": "blackout",
    "dropout": "blackout",
    "tof_blackout": "blackout",
}

MODE_ORDER = ["tof_imu_height", "tof_flow_imu_height"]
MODE_LABELS = {
    "tof_imu_height": "ToF+IMU+高度辅助",
    "tof_flow_imu_height": "ToF+光流+IMU+高度辅助",
}
MODE_ALIASES = {
    "tof_imu_height": "tof_imu_height",
    "imu_height": "tof_imu_height",
    "height": "tof_imu_height",
    "tof_height": "tof_imu_height",
    "tof_flow_imu_height": "tof_flow_imu_height",
    "flow_imu_height": "tof_flow_imu_height",
    "tof_flow_height": "tof_flow_imu_height",
    "flow_height": "tof_flow_imu_height",
}

COMM_ORDER = ["full", "off", "intermittent"]
COMM_LABELS = {
    "full": "完整共享",
    "off": "完全关闭",
    "intermittent": "间歇中断",
}
COMM_ALIASES = {
    "full": "full",
    "on": "full",
    "enabled": "full",
    "share": "full",
    "shared": "full",
    "full_share": "full",
    "cooperative": "full",
    "off": "off",
    "none": "off",
    "disabled": "off",
    "no_comm": "off",
    "independent": "off",
    "intermittent": "intermittent",
    "dropout": "intermittent",
    "outage": "intermittent",
    "lossy": "intermittent",
    "partial": "intermittent",
}

PROFILE_FIELDS = [
    ("noise_std_m", "白噪声标准差 m", 3),
    ("range_sigma_per_m", "距离尺度噪声 1/m", 3),
    ("bias_bound_m", "固定偏置上界 m", 3),
    ("scale_bound", "尺度偏差上界", 3),
    ("frame_bias_std_m", "逐帧公共误差 m", 3),
    ("drop_probability", "丢弃概率", 2),
]

STAGE_LABELS = {
    "imu_predict": "IMU 预测",
    "flow_height": "光流与高度融合",
    "tof_frame": "ToF 单帧融合",
    "uwb_peers": "UWB 融合与来源门控",
    "uwb_fallback": "UWB 退化分支",
    "vision_pixel": "灯条像素检测",
    "vision_detect_pnp": "OpenCV 检测与 PnP",
    "green_guide": "绿灯检测与平面求交",
    "target_tracking": "目标观测与共享合入",
    "periodic_learning": "周期学习与预测",
    "navigation": "导航意图",
    "intercept_guidance": "截获与视觉制导",
    "team_impact_schedule": "四机接触任务调度",
    "vision_submit": "视觉任务提交",
    "vision_poll": "视觉结果轮询",
    "periodic_service": "周期模型服务",
}


class MissingData(RuntimeError):
    """输入数据缺失或不满足报告要求。"""


def read_json(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def pick(mapping, *names):
    """按候选键名取值，返回 (值, 使用的键名)。"""
    for name in names:
        if isinstance(mapping, dict) and name in mapping and mapping[name] is not None:
            return mapping[name], name
    return None, None


def as_float(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)) and math.isfinite(value):
        return float(value)
    return None


def fmt(value, digits=3):
    number = as_float(value)
    if number is None:
        return "—"
    if digits == 0:
        return f"{number:.0f}"
    if abs(number) < 0.5 * 10 ** (-digits):
        number = 0.0
    return f"{number:.{digits}f}"


def fmt_ratio(value, digits=1):
    number = as_float(value)
    if number is None:
        return "—"
    return f"{100 * number:.{digits}f}%"


def _text_units(text):
    """估算排版宽度：中日韩字符按两个单位计，其余按一个单位。"""
    return sum(2 if ord(char) > 0x2E80 else 1 for char in str(text))


def _pdf_text(text):
    """转义 reportlab 段落标记，避免尖括号或 & 被当作标签解析。"""
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


def canonical(name, aliases):
    if isinstance(name, str):
        key = name.strip().lower().replace("-", "_").replace(" ", "_")
        if key in aliases:
            return aliases[key]
    return None


def median(values):
    numbers = [value for value in values if value is not None]
    return statistics.median(numbers) if numbers else None


def percentile(values, fraction):
    numbers = sorted(value for value in values if value is not None)
    if not numbers:
        return None
    if fraction > 1:
        fraction = fraction / 100
    fraction = min(max(fraction, 0.0), 1.0)
    position = fraction * (len(numbers) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return numbers[lower]
    weight = position - lower
    return numbers[lower] * (1 - weight) + numbers[upper] * weight


def mean(values):
    numbers = [value for value in values if value is not None]
    return statistics.fmean(numbers) if numbers else None


# ---------------------------------------------------------------- A：场景


@dataclass
class Scenarios:
    path: Path
    profiles: dict = field(default_factory=dict)
    robots: list = field(default_factory=list)
    research: dict = field(default_factory=dict)
    context: str | None = None


def load_scenarios(path: Path) -> Scenarios:
    if not path.is_file():
        raise MissingData(f"缺少场景文件：{path}")
    raw = read_json(path)
    profiles = raw.get("profiles")
    robots = raw.get("robots")
    if not isinstance(profiles, dict) or not profiles:
        raise MissingData(f"{path} 缺少 profiles 字段")
    if not isinstance(robots, list):
        raise MissingData(f"{path} 缺少 robots 字段")
    for robot in robots:
        if not isinstance(robot, dict):
            raise MissingData(f"{path} 的 robots 条目不是对象")
        for key in ("center_m", "size_m"):
            values = robot.get(key)
            if not isinstance(values, list) or len(values) != 3:
                raise MissingData(f"{path} 的 robots 条目缺少三元 {key}")
    return Scenarios(
        path=path,
        profiles=profiles,
        robots=robots,
        research=raw.get("research") if isinstance(raw.get("research"), dict) else {},
        context=raw.get("context") or raw.get("notes"),
    )


# ---------------------------------------------------------------- B：板端


@dataclass
class BoardProfile:
    name: str
    path: Path
    data: dict
    entry: dict | None = None
    monitor: dict | None = None
    failed: bool = False

    @property
    def completed(self):
        if self.failed or not self.data.get("stages"):
            return False
        verification = self.verification
        if verification.get("ok") is False:
            return False
        return self.exit_status in (0, None)

    def kernel_log_text(self):
        path = self.path.parent / f"{self.name}.kernel.log"
        if path.is_file():
            return path.read_text(encoding="utf-8", errors="replace")
        return ""

    def oom_line(self):
        for line in self.kernel_log_text().splitlines():
            if "Out of memory" in line:
                return line.split("] ", 1)[-1].strip()
        return None

    def failure_reason(self):
        entry = self.entry or {}
        reasons = []
        if entry.get("timed_out"):
            reasons.append("运行超时")
        status = entry.get("exit_status")
        if status not in (0, None):
            reasons.append(f"退出状态 {status}")
        verification = self.verification
        if verification.get("ok") is False:
            failures = verification.get("failures")
            if isinstance(failures, list) and failures:
                reasons.append("校验未通过：" + "；".join(str(item) for item in failures))
            else:
                reasons.append("校验未通过")
        if self.oom_line():
            reasons.append("内核内存不足终止")
        if not reasons:
            reasons.append("无完整测量结果")
        return "，".join(reasons)

    def short_failure_reason(self):
        entry = self.entry or {}
        reasons = []
        if entry.get("timed_out"):
            reasons.append("运行超时")
        status = entry.get("exit_status")
        if status not in (0, None):
            reasons.append(f"退出状态 {status}")
        if self.oom_line():
            reasons.append("内核内存不足终止")
        if self.verification.get("ok") is False and not reasons:
            reasons.append("校验未通过")
        if not reasons:
            reasons.append("无完整测量结果")
        return "，".join(reasons)

    @property
    def spec(self):
        return (self.entry or {}).get("spec") or {}

    @property
    def verification(self):
        return (self.entry or {}).get("verification") or {}

    @property
    def exit_status(self):
        return (self.entry or {}).get("exit_status")

    @property
    def timed_out(self):
        return (self.entry or {}).get("timed_out")

    @property
    def elapsed_s(self):
        return as_float(self.data.get("elapsed_s"))

    @property
    def stages(self):
        stages = self.data.get("stages")
        return stages if isinstance(stages, dict) else {}

    def achieved_hz(self, stage: str):
        entry = self.stages.get(stage)
        elapsed = self.elapsed_s
        if not isinstance(entry, dict) or not elapsed:
            return None
        samples = as_float(entry.get("samples"))
        return samples / elapsed if samples else None

    @property
    def skipped_fraction(self):
        skipped = as_float(self.data.get("skipped_ticks"))
        completed = as_float(self.data.get("completed_ticks"))
        if skipped is None or completed is None or skipped + completed <= 0:
            return None
        return skipped / (skipped + completed)

    @property
    def process_cpu_percent(self):
        value = as_float(self.data.get("process_cpu_percent"))
        if value is not None:
            return value
        seconds = as_float(self.data.get("process_cpu_seconds"))
        elapsed = self.elapsed_s
        return 100 * seconds / elapsed if seconds is not None and elapsed else None

    @property
    def resources(self):
        rows = self.data.get("resources")
        return [row for row in rows if isinstance(row, dict)] if isinstance(rows, list) else []

    def max_temperature(self):
        values = []
        for row in self.resources:
            temperatures = row.get("temperatures_c")
            if isinstance(temperatures, dict):
                values.extend(as_float(value) for value in temperatures.values())
        values = [value for value in values if value is not None]
        return max(values) if values else None

    def min_available_memory_kib(self):
        values = [as_float(row.get("available_memory_kib")) for row in self.resources]
        values = [value for value in values if value is not None]
        return min(values) if values else None

    def system_cpu_series(self):
        """整机 CPU 占用，由独立监控通道的 /proc/stat 计数器逐间隔重算。"""
        samples = self.monitor_samples()
        series = []
        previous = None
        origin = None
        for sample in samples:
            stamp = as_float(sample.get("epoch_s"))
            if origin is None and stamp is not None:
                origin = stamp
            counters = sample.get("cpu") if isinstance(sample, dict) else None
            if not isinstance(counters, dict):
                previous = None
                continue
            busy = as_float(counters.get("busy"))
            total = as_float(counters.get("total"))
            if busy is None or total is None:
                previous = None
                continue
            if previous is not None and total > previous[1]:
                when = stamp - origin if stamp is not None and origin is not None else None
                series.append((when, 100 * (busy - previous[0]) / (total - previous[1])))
            previous = (busy, total)
        return series if series else None

    def monitor_samples(self):
        samples = (self.monitor or {}).get("samples")
        return [sample for sample in samples if isinstance(sample, dict)] if isinstance(samples, list) else []

    def device_cpu_percent(self):
        return as_float((self.monitor or {}).get("device_cpu_percent"))

    def device_cpu_interval_max(self):
        return as_float((self.monitor or {}).get("device_cpu_percent_interval_max"))

    def loadavg_1min_max(self):
        return as_float((self.monitor or {}).get("loadavg_1min_max"))

    def monitor_time_series(self):
        samples = self.monitor_samples()
        stamps = [as_float(sample.get("epoch_s")) for sample in samples]
        stamps = [stamp for stamp in stamps if stamp is not None]
        if not stamps:
            return []
        origin = stamps[0]
        return [stamp - origin for stamp in stamps]

    def cpu_series(self):
        return [as_float(row.get("process_cpu_percent")) for row in self.resources]

    def memory_series(self):
        return [as_float(row.get("available_memory_kib")) for row in self.resources]

    def temperature_series(self):
        series = []
        for row in self.resources:
            temperatures = row.get("temperatures_c")
            values = []
            if isinstance(temperatures, dict):
                values = [as_float(value) for value in temperatures.values()]
            values = [value for value in values if value is not None]
            series.append(max(values) if values else None)
        return series

    def time_series(self):
        return [as_float(row.get("elapsed_s")) for row in self.resources]


@dataclass
class BoardSet:
    directory: Path
    profiles: list
    manifest: Path | None = None
    expected: list = field(default_factory=list)
    missing: list = field(default_factory=list)
    sources: list = field(default_factory=list)
    meta: dict = field(default_factory=dict)

    def completed(self):
        return [profile for profile in self.profiles if profile.completed]

    def incomplete(self):
        return [profile for profile in self.profiles if not profile.completed]

    def by_name(self, name: str):
        for profile in self.profiles:
            if profile.name == name:
                return profile
        return None

    def reference(self):
        """阶段与资源明细使用的参考配置：整幅 RMUC2027 地形优先，其次持续负载。"""
        for name in ("rmuc2027_15", "rmuc2027-15", "sustained15", "sustained-15hz"):
            profile = self.by_name(name)
            if profile is not None and profile.completed:
                return profile
        completed = self.completed()
        return completed[0] if completed else self.profiles[0]


def load_board(root: Path) -> BoardSet:
    base = root / "logs" / "research-20261023"
    if not base.is_dir():
        raise MissingData(f"缺少板端结果目录：{base}")
    directories = sorted(path for path in base.iterdir() if path.is_dir())
    if not directories:
        raise MissingData(f"{base} 下没有运行目录")
    directory = directories[-1]
    summary_path = directory / "summary.json"
    if not summary_path.is_file():
        raise MissingData(f"缺少板端汇总文件：{summary_path}")
    summary = read_json(summary_path)
    entries = summary.get("profiles")
    if not isinstance(entries, list) or not entries:
        raise MissingData(f"{summary_path} 的 profiles 为空")
    manifest_path = directory / "manifest.json"
    expected = []
    if manifest_path.is_file():
        names = read_json(manifest_path).get("profiles")
        if isinstance(names, list):
            expected = [str(name) for name in names]
    profiles = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        name = entry.get("profile")
        if not isinstance(name, str):
            continue
        measurement_path = directory / f"{name}.json"
        measurement = {}
        if measurement_path.is_file() and measurement_path.stat().st_size > 0:
            loaded = read_json(measurement_path)
            if isinstance(loaded, dict) and loaded.get("stages"):
                measurement = loaded
        profiles.append(
            BoardProfile(
                name=name,
                path=measurement_path,
                data=measurement,
                entry=entry,
                monitor=entry.get("device"),
                failed=not measurement,
            )
        )
    if not any(profile.completed for profile in profiles):
        raise MissingData(f"{directory} 下没有完成校验的板端测量结果")
    present = {profile.name for profile in profiles}
    sources = [summary_path, *[profile.path for profile in profiles if profile.completed]]
    if manifest_path.is_file():
        sources.append(manifest_path)
    return BoardSet(
        directory=directory,
        profiles=profiles,
        manifest=manifest_path if manifest_path.is_file() else None,
        expected=expected,
        missing=[name for name in expected if name not in present],
        sources=sources,
        meta={key: value for key, value in summary.items() if key != "profiles"},
    )


# ---------------------------------------------------------------- C：定位


@dataclass
class LocRecord:
    window: str
    scenario: str
    mode: str
    communication: str
    seed: int | None
    rmse_xy_m: float | None
    p50_xy_m: float | None
    p95_xy_m: float | None
    max_xy_m: float | None
    xy_valid_fraction: float | None
    structural_valid_fraction: float | None
    false_valid_fraction: float | None
    longest_invalid_s: float | None
    uwb_accept_rate: float | None
    occluded_rays_per_frame: float | None
    input_fingerprint: str | None
    raw: dict


@dataclass
class Localization:
    paths: list
    records: list
    key_map: dict = field(default_factory=dict)
    meta: dict = field(default_factory=dict)
    sensor_model: list = field(default_factory=list)
    paired: list = field(default_factory=list)

    def windows(self):
        return sorted({record.window for record in self.records})

    def primary_window(self):
        windows = self.windows()
        if "transit" in windows:
            return "transit"
        return windows[0] if windows else "transit"

    def seeds(self):
        return sorted({record.seed for record in self.records if record.seed is not None})

    def select(self, scenario=None, mode=None, communication=None, window=None):
        selected_window = self.primary_window() if window is None else window
        return [
            record
            for record in self.records
            if (scenario is None or record.scenario == scenario)
            and (mode is None or record.mode == mode)
            and (communication is None or record.communication == communication)
            and (selected_window == "*" or record.window == selected_window)
        ]


METRIC_KEYS = {
    "rmse_xy_m": ("xy_rmse_m", "rmse_xy_m", "rmse_m", "rmse_xy", "rmse"),
    "p50_xy_m": ("xy_p50_m", "p50_xy_m", "p50_m", "median_xy_m", "median_m", "p50"),
    "p95_xy_m": ("xy_p95_m", "p95_xy_m", "p95_m", "p95"),
    "max_xy_m": ("max_xy_error_m", "max_xy_m", "max_m", "max_error_m", "max"),
    "xy_valid_fraction": (
        "xy_valid_fraction",
        "fraction_xy_valid",
        "xy_valid_ratio",
        "valid_fraction",
        "xy_valid",
    ),
    "structural_valid_fraction": (
        "xy_structural_fraction",
        "structural_valid_fraction",
        "fraction_structural_valid",
        "structural_valid_ratio",
        "structural_valid",
    ),
    "false_valid_fraction": (
        "false_valid_fraction",
        "fraction_false_valid",
        "false_valid_ratio",
        "false_valid",
    ),
    "longest_invalid_s": (
        "longest_invalid_s",
        "longest_invalid_duration_s",
        "max_invalid_duration_s",
    ),
    "uwb_accept_rate": (
        "uwb_accept_fraction",
        "uwb_accept_rate",
        "peer_accept_rate",
        "uwb_accept_ratio",
    ),
    "occluded_rays_per_frame": (
        "robot_occluded_rays_per_frame",
        "occluded_rays_per_frame",
        "robot_occluded_rays_mean",
        "robot_occluded_rays",
    ),
    "input_fingerprint": (
        "input_checksum",
        "input_fingerprint",
        "fingerprint",
        "checksum",
    ),
}


def _run_records(raw):
    if isinstance(raw, list):
        return raw
    if isinstance(raw, dict):
        for key in ("runs", "results", "experiments", "cases", "records"):
            value = raw.get(key)
            if isinstance(value, list) and value:
                return value
    return []


def _vehicle_metrics(record):
    for key in ("vehicles", "per_vehicle", "agents", "drones"):
        value = record.get(key)
        if isinstance(value, list) and value:
            entries = [entry for entry in value if isinstance(entry, dict)]
            if entries:
                return entries
    return []


def normalize_localization(paths: list) -> Localization:
    records = []
    key_map = {}
    meta = {}
    sensor_model = []
    paired = []
    for path in paths:
        raw = read_json(path)
        config = raw.get("config") if isinstance(raw, dict) else None
        config = config if isinstance(config, dict) else {}
        if isinstance(raw, dict):
            for key in (
                "rates_hz",
                "sensor_noise",
                "initialization",
                "limitations",
                "tof_geometry",
                "terrain",
                "trajectory",
                "communication_modes",
                "fusion_modes",
                "profiles",
                "scenario_groups_excluded",
                "tof_error_model",
                "generator",
                "scenarios",
            ):
                if key in raw and key not in meta:
                    meta[key] = raw[key]
            rows = raw.get("sensor_model")
            if isinstance(rows, list):
                sensor_model.extend(row for row in rows if isinstance(row, dict))
            rows = raw.get("paired_by_seed")
            if isinstance(rows, list):
                paired.extend(row for row in rows if isinstance(row, dict))
        for entry in _run_records(raw):
            if not isinstance(entry, dict):
                continue
            scenario = canonical(
                pick(entry, "scenario", "scene", "condition")[0] or config.get("scenario"),
                SCENARIO_ALIASES,
            )
            mode = canonical(
                pick(entry, "mode", "sensor_mode", "fusion", "variant")[0] or config.get("mode"),
                MODE_ALIASES,
            )
            communication = canonical(
                pick(entry, "communication", "comm", "comms", "link", "network")[0]
                or config.get("communication"),
                COMM_ALIASES,
            )
            if scenario is None or mode is None or communication is None:
                continue
            window_value, _ = pick(entry, "window", "phase", "segment")
            window = window_value.strip() if isinstance(window_value, str) else "transit"
            seed_value, _ = pick(entry, "seed", "random_seed", "rng_seed")
            seed = int(seed_value) if isinstance(seed_value, (int, float)) else None
            values = {}
            for field_name, candidates in METRIC_KEYS.items():
                value, used = pick(entry, *candidates)
                if used is not None:
                    key_map.setdefault(field_name, used)
                values[field_name] = value
            vehicles = _vehicle_metrics(entry)
            if vehicles:
                for field_name in ("rmse_xy_m", "p50_xy_m", "p95_xy_m"):
                    if as_float(values[field_name]) is None:
                        values[field_name] = mean(
                            [
                                as_float(pick(vehicle, *METRIC_KEYS[field_name])[0])
                                for vehicle in vehicles
                            ]
                        )
                if as_float(values["max_xy_m"]) is None:
                    maxima = [
                        as_float(pick(vehicle, *METRIC_KEYS["max_xy_m"])[0]) for vehicle in vehicles
                    ]
                    maxima = [value for value in maxima if value is not None]
                    values["max_xy_m"] = max(maxima) if maxima else None
                if as_float(values["longest_invalid_s"]) is None:
                    durations = [
                        as_float(pick(vehicle, *METRIC_KEYS["longest_invalid_s"])[0])
                        for vehicle in vehicles
                    ]
                    durations = [value for value in durations if value is not None]
                    values["longest_invalid_s"] = max(durations) if durations else None
            records.append(
                LocRecord(
                    window=window,
                    scenario=scenario,
                    mode=mode,
                    communication=communication,
                    seed=seed,
                    rmse_xy_m=as_float(values["rmse_xy_m"]),
                    p50_xy_m=as_float(values["p50_xy_m"]),
                    p95_xy_m=as_float(values["p95_xy_m"]),
                    max_xy_m=as_float(values["max_xy_m"]),
                    xy_valid_fraction=as_float(values["xy_valid_fraction"]),
                    structural_valid_fraction=as_float(values["structural_valid_fraction"]),
                    false_valid_fraction=as_float(values["false_valid_fraction"]),
                    longest_invalid_s=as_float(values["longest_invalid_s"]),
                    uwb_accept_rate=as_float(values["uwb_accept_rate"]),
                    occluded_rays_per_frame=as_float(values["occluded_rays_per_frame"]),
                    input_fingerprint=(
                        str(values["input_fingerprint"])
                        if values["input_fingerprint"] is not None
                        else None
                    ),
                    raw=entry,
                )
            )
    if not records:
        raise MissingData(
            "定位结果中未找到带 scenario、mode、communication 的运行记录："
            + ", ".join(str(path) for path in paths)
        )
    return Localization(
        paths=paths,
        records=records,
        key_map=key_map,
        meta=meta,
        sensor_model=sensor_model,
        paired=paired,
    )


def localization_paths(root: Path) -> list:
    base = root / "build" / "research-20261023"
    if not base.is_dir():
        raise MissingData(f"缺少定位结果目录：{base}")
    paths = sorted(
        path
        for path in base.glob("localization-*.json")
        if not path.name.endswith(".manifest.json")
    )
    if not paths:
        raise MissingData(f"{base} 下没有 localization-*.json")
    return paths


# ---------------------------------------------------------------- 数据集合


@dataclass
class ReportData:
    root: Path
    scenarios: Scenarios
    board: BoardSet
    localization: Localization
    sources: list = field(default_factory=list)


def collect(root: Path) -> ReportData:
    scenarios = load_scenarios(root / "simulation" / "research" / "scenarios.json")
    board = load_board(root)
    if not board.completed():
        raise MissingData("板端没有完成的测量配置")
    paths = localization_paths(root)
    localization = normalize_localization(paths)
    sources = [scenarios.path, *board.sources, *paths]
    for extra in (
        root / "docs" / "research-20261023" / "tof-sources.md",
        root / "docs" / "research-20261023" / "nano-findings.md",
        root / "docs" / "research-20261023" / "localization-findings.md",
    ):
        if extra.is_file():
            sources.append(extra)
    if board.manifest is not None:
        sources.append(board.manifest)
    return ReportData(
        root=root,
        scenarios=scenarios,
        board=board,
        localization=localization,
        sources=sources,
    )


# ---------------------------------------------------------------- 图表


def _mpl():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DengXian"]
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["figure.dpi"] = 200
    plt.rcParams["savefig.bbox"] = "tight"
    return plt


def _scenario_axis(axis, scenarios):
    labels = [SCENARIO_LABELS.get(name, name) for name in scenarios]
    axis.set_xticks(range(len(scenarios)))
    axis.set_xticklabels(labels, rotation=40, ha="right", fontsize=7)
    axis.grid(axis="y", color="#D9D9D9", linewidth=0.6)
    axis.set_axisbelow(True)


def figure_scene(data: ReportData, path: Path):
    plt = _mpl()
    robots = data.scenarios.robots
    figure, axis = plt.subplots(figsize=(7.0, 3.3))
    colors = ["#4C78A8", "#F58518", "#54A24B", "#B279A2", "#E45756"]
    left = right = bottom = top = None
    for index, robot in enumerate(robots):
        center = robot["center_m"]
        size = robot["size_m"]
        left = min(left, center[0] - size[0] / 2) if left is not None else center[0] - size[0] / 2
        right = max(right, center[0] + size[0] / 2) if right is not None else center[0] + size[0] / 2
        bottom = min(bottom, center[1] - size[1] / 2) if bottom is not None else center[1] - size[1] / 2
        top = max(top, center[1] + size[1] / 2) if top is not None else center[1] + size[1] / 2
        color = colors[index % len(colors)]
        axis.add_patch(
            plt.Rectangle(
                (center[0] - size[0] / 2, center[1] - size[1] / 2),
                size[0],
                size[1],
                facecolor=color,
                edgecolor="#333333",
                alpha=0.45,
                linewidth=0.8,
            )
        )
        axis.annotate(
            str(robot.get("role", "")),
            (center[0], center[1]),
            textcoords="offset points",
            xytext=(0, -6),
            ha="center",
            va="top",
            rotation=90,
            fontsize=6,
        )
    route_y = as_float((data.scenarios.research.get("route") or {}).get("y_m"))
    if route_y is not None:
        axis.axhline(route_y, color="#666666", linestyle="--", linewidth=0.8)
        axis.annotate(
            f"编队航路 y={route_y:.1f} m",
            (right if right is not None else 0.0, route_y),
            textcoords="offset points",
            xytext=(-4, 4),
            ha="right",
            fontsize=6,
            color="#666666",
        )
    axis.set_xlabel("地图 X m")
    axis.set_ylabel("地图 Y m")
    axis.grid(color="#D9D9D9", linewidth=0.6)
    axis.set_aspect("equal", adjustable="box")
    if left is not None:
        padding_x = max((right - left) * 0.08, 0.4)
        padding_y = max((top - bottom) * 0.12, 0.4)
        axis.set_xlim(left - padding_x, right + padding_x)
        axis.set_ylim(bottom - padding_y, top + padding_y)
    axis.set_title("未知机器人占位俯视投影", fontsize=10)
    figure.savefig(path)
    plt.close(figure)


def figure_localization_rmse(data: ReportData, path: Path):
    plt = _mpl()
    communications = [name for name in COMM_ORDER if data.localization.select(communication=name)]
    modes = [name for name in MODE_ORDER if data.localization.select(mode=name)]
    figure, axes = plt.subplots(
        1, max(len(communications), 1), figsize=(7.2, 2.9), sharey=True, squeeze=False
    )
    offsets = {mode: (index - (len(modes) - 1) / 2) * 0.28 for index, mode in enumerate(modes)}
    mode_colors = dict(zip(modes, ["#4C78A8", "#F58518"]))
    for axis, communication in zip(axes[0], communications):
        for mode in modes:
            for position, scenario in enumerate(SCENARIO_ORDER):
                records = data.localization.select(
                    scenario=scenario, mode=mode, communication=communication
                )
                values = [record.rmse_xy_m for record in records if record.rmse_xy_m is not None]
                if not values:
                    continue
                center = position + offsets[mode]
                axis.bar(
                    center,
                    median(values),
                    width=0.26,
                    color=mode_colors[mode],
                    alpha=0.35,
                    edgecolor=mode_colors[mode],
                    linewidth=0.8,
                )
                axis.scatter([center] * len(values), values, s=12, color=mode_colors[mode], zorder=3)
        _scenario_axis(axis, SCENARIO_ORDER)
        axis.set_title(COMM_LABELS.get(communication, communication), fontsize=9)
        axis.set_yscale("log")
    axes[0][0].set_ylabel("水平 RMSE m")
    handles = [
        plt.Line2D(
            [], [], color=mode_colors[mode], marker="o", linestyle="", label=MODE_LABELS.get(mode, mode)
        )
        for mode in modes
    ]
    figure.legend(handles=handles, loc="upper center", ncol=max(len(modes), 1), fontsize=7, frameon=False)
    figure.tight_layout(rect=(0, 0, 1, 0.93))
    figure.savefig(path)
    plt.close(figure)


def figure_localization_valid(data: ReportData, path: Path):
    plt = _mpl()
    communications = [name for name in COMM_ORDER if data.localization.select(communication=name)]
    figure, axes = plt.subplots(1, 2, figsize=(7.2, 2.7))
    width = 0.8 / max(len(communications), 1)
    colors = dict(zip(communications, ["#4C78A8", "#E45756", "#54A24B"]))
    for index, communication in enumerate(communications):
        positions = []
        valid = []
        false_valid = []
        for position, scenario in enumerate(SCENARIO_ORDER):
            records = data.localization.select(scenario=scenario, communication=communication)
            valid.append(mean([record.xy_valid_fraction for record in records]) or 0)
            false_valid.append(mean([record.false_valid_fraction for record in records]) or 0)
            positions.append(position + (index - (len(communications) - 1) / 2) * width)
        axes[0].bar(
            positions,
            valid,
            width=width,
            color=colors[communication],
            label=COMM_LABELS.get(communication, communication),
        )
        axes[1].bar(positions, false_valid, width=width, color=colors[communication])
    for axis, title in zip(axes, ["水平有效率", "大误差仍标为有效比例"]):
        _scenario_axis(axis, SCENARIO_ORDER)
        axis.set_title(title, fontsize=9)
        axis.set_ylabel("比例")
        axis.yaxis.set_major_formatter(lambda value, _position: f"{100 * value:.0f}%")
    handles = [
        plt.Line2D([], [], color=colors[name], linewidth=6, label=COMM_LABELS.get(name, name))
        for name in communications
    ]
    figure.legend(
        handles=handles,
        loc="upper center",
        ncol=max(len(communications), 1),
        fontsize=7,
        frameon=False,
    )
    figure.tight_layout(rect=(0, 0, 1, 0.93))
    figure.savefig(path)
    plt.close(figure)


def figure_communication_delta(data: ReportData, path: Path):
    plt = _mpl()
    modes = [name for name in MODE_ORDER if data.localization.select(mode=name)]
    comparisons = ["off", "intermittent"]
    figure, axes = plt.subplots(1, max(len(modes), 1), figsize=(7.2, 2.8), sharey=True, squeeze=False)
    colors = dict(zip(comparisons, ["#E45756", "#F58518"]))
    for axis, mode in zip(axes[0], modes):
        for index, communication in enumerate(comparisons):
            for position, scenario in enumerate(SCENARIO_ORDER):
                base = {
                    record.seed: record.rmse_xy_m
                    for record in data.localization.select(
                        scenario=scenario, mode=mode, communication="full"
                    )
                }
                deltas = []
                for record in data.localization.select(
                    scenario=scenario, mode=mode, communication=communication
                ):
                    reference = base.get(record.seed)
                    if reference is not None and record.rmse_xy_m is not None:
                        deltas.append(record.rmse_xy_m - reference)
                if not deltas:
                    continue
                center = position + (index - (len(comparisons) - 1) / 2) * 0.28
                axis.scatter([center] * len(deltas), deltas, s=14, color=colors[communication], zorder=3)
                axis.hlines(
                    median(deltas), center - 0.12, center + 0.12, color=colors[communication], linewidth=2
                )
        _scenario_axis(axis, SCENARIO_ORDER)
        axis.axhline(0, color="#666666", linewidth=0.8)
        axis.set_title(MODE_LABELS.get(mode, mode), fontsize=9)
        axis.set_yscale("symlog", linthresh=0.05)
    axes[0][0].set_ylabel("相对完整共享的 RMSE 变化 m")
    handles = [
        plt.Line2D(
            [], [], color=colors[name], marker="o", linestyle="", label=f"相对{COMM_LABELS[name]}"
        )
        for name in comparisons
    ]
    figure.legend(handles=handles, loc="upper center", ncol=len(comparisons), fontsize=7, frameon=False)
    figure.tight_layout(rect=(0, 0, 1, 0.92))
    figure.savefig(path)
    plt.close(figure)


def figure_board_rates(data: ReportData, path: Path):
    plt = _mpl()
    profiles = data.board.completed()
    names = [profile.name for profile in profiles]
    figure, axes = plt.subplots(1, 2, figsize=(7.2, 2.7))
    positions = list(range(len(profiles)))
    vision = [profile.achieved_hz("vision_detect_pnp") or 0 for profile in profiles]
    navigation = [profile.achieved_hz("navigation") or 0 for profile in profiles]
    requested = [as_float(profile.data.get("camera_hz")) or 0 for profile in profiles]
    axes[0].bar(
        [position - 0.18 for position in positions], vision, width=0.34, color="#4C78A8", label="实测视觉 Hz"
    )
    axes[0].bar(
        [position + 0.18 for position in positions],
        navigation,
        width=0.34,
        color="#54A24B",
        label="实测导航 Hz",
    )
    axes[0].scatter(positions, requested, marker="_", s=200, color="#333333", label="请求视觉 Hz")
    axes[0].set_ylabel("频率 Hz")
    top = max([*vision, *navigation, *requested, 1.0]) * 1.45
    axes[0].set_ylim(0, top)
    axes[0].legend(fontsize=6, frameon=False, loc="upper center")
    axes[0].set_title("实际调用频率", fontsize=9)
    axes[1].bar(positions, [100 * (profile.skipped_fraction or 0) for profile in profiles], color="#E45756")
    axes[1].set_ylabel("跳过槽位比例 %")
    axes[1].set_title("100 Hz 槽位跳过比例", fontsize=9)
    for axis in axes:
        axis.set_xticks(positions)
        axis.set_xticklabels(names, rotation=20, ha="right", fontsize=8)
        axis.grid(axis="y", color="#D9D9D9", linewidth=0.6)
        axis.set_axisbelow(True)
    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)


def figure_board_stages(data: ReportData, path: Path):
    plt = _mpl()
    profile = data.board.reference()
    entries = [
        (stage, entry)
        for stage, entry in profile.stages.items()
        if as_float(entry.get("samples")) and stage in STAGE_LABELS
    ]
    entries.sort(key=lambda item: as_float(item[1].get("p99_ms")) or 0)
    labels = [STAGE_LABELS[stage] for stage, _ in entries]
    means = [as_float(entry.get("mean_ms")) or 0 for _, entry in entries]
    p99 = [as_float(entry.get("p99_ms")) or 0 for _, entry in entries]
    maxima = [as_float(entry.get("max_ms")) or 0 for _, entry in entries]
    figure, axis = plt.subplots(figsize=(7.2, 3.0))
    positions = list(range(len(entries)))
    axis.barh([position + 0.24 for position in positions], means, height=0.24, color="#4C78A8", label="平均")
    axis.barh(positions, p99, height=0.24, color="#F58518", label="P99")
    axis.barh([position - 0.24 for position in positions], maxima, height=0.24, color="#E45756", label="最长")
    axis.set_yticks(positions)
    axis.set_yticklabels(labels, fontsize=8)
    axis.set_xscale("log")
    axis.set_xlabel("单次调用耗时 ms（对数轴）")
    axis.grid(axis="x", color="#D9D9D9", linewidth=0.6)
    axis.set_axisbelow(True)
    axis.legend(fontsize=7, frameon=False)
    axis.set_title(f"阶段耗时 {profile.name}", fontsize=9)
    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)


def figure_board_resources(data: ReportData, path: Path):
    plt = _mpl()
    profile = data.board.reference()
    times = profile.time_series()
    figure, axes = plt.subplots(1, 2, figsize=(7.2, 2.6))
    axes[0].plot(times, profile.cpu_series(), color="#4C78A8", label="进程 CPU %（100% 为一个核）")
    system_cpu = profile.system_cpu_series() or []
    if system_cpu:
        axes[0].plot(
            [when for when, _value in system_cpu],
            [value for _when, value in system_cpu],
            color="#B279A2",
            label="整机 CPU %（独立监控通道）",
        )
    axes[0].set_xlabel("运行时间 s")
    axes[0].set_ylabel("CPU 占用 %")
    axes[0].legend(fontsize=7, frameon=False)
    axes[0].set_title("CPU", fontsize=9)
    temperature_axis = axes[1]
    temperature_axis.plot(times, profile.temperature_series(), color="#E45756", label="最高采样温度 °C")
    temperature_axis.set_xlabel("运行时间 s")
    temperature_axis.set_ylabel("温度 °C")
    memory_axis = temperature_axis.twinx()
    memory_axis.plot(
        times,
        [value / 1024 if value is not None else None for value in profile.memory_series()],
        color="#54A24B",
        label="系统可用内存 MiB",
    )
    memory_axis.set_ylabel("可用内存 MiB")
    lines = temperature_axis.get_lines() + memory_axis.get_lines()
    temperature_axis.legend(lines, [line.get_label() for line in lines], fontsize=7, frameon=False)
    temperature_axis.set_title("温度与内存", fontsize=9)
    for axis in axes:
        axis.grid(color="#D9D9D9", linewidth=0.6)
        axis.set_axisbelow(True)
    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)


FIGURE_SPECS = [
    ("fig-scene-occluders.png", figure_scene, "图 1 未知机器人占位俯视投影"),
    ("fig-loc-rmse.png", figure_localization_rmse, "图 2 各场景水平 RMSE"),
    ("fig-loc-valid.png", figure_localization_valid, "图 3 水平有效率与大误差仍标为有效的比例"),
    ("fig-comm-delta.png", figure_communication_delta, "图 4 通信受限相对完整共享的 RMSE 变化"),
    ("fig-board-rates.png", figure_board_rates, "图 5 板端实际调用频率与跳过槽位比例"),
    ("fig-board-stages.png", figure_board_stages, "图 6 板端阶段耗时"),
    ("fig-board-resources.png", figure_board_resources, "图 7 板端 CPU、温度与可用内存"),
]


def build_figures(data: ReportData, figure_dir: Path) -> list:
    figure_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for filename, builder, _caption in FIGURE_SPECS:
        path = figure_dir / filename
        builder(data, path)
        if not path.is_file() or path.stat().st_size == 0:
            raise RuntimeError(f"图表未生成：{path}")
        written.append(path)
    return written


# ---------------------------------------------------------------- 文档模型


def h1(text):
    return ("h1", text)


def h2(text):
    return ("h2", text)


def paragraph(text):
    return ("p", text)


def table(caption, headers, rows, aligns):
    if len(aligns) != len(headers):
        raise ValueError(f"{caption} 的列对齐数量与表头不一致")
    for row in rows:
        if len(row) != len(headers):
            raise ValueError(f"{caption} 的行列数与表头不一致：{row}")
    return ("table", caption, headers, rows, aligns)


def figure(caption, path):
    return ("figure", caption, path)


def code(text):
    return ("code", text)


def _table_rows(data: ReportData):
    """各场景、模式、通信条件的误差与有效率汇总行。"""
    rows = []
    for scenario in SCENARIO_ORDER:
        for mode in MODE_ORDER:
            for communication in COMM_ORDER:
                records = data.localization.select(
                    scenario=scenario, mode=mode, communication=communication
                )
                if not records:
                    continue
                rmse = [record.rmse_xy_m for record in records if record.rmse_xy_m is not None]
                seeds = sorted({record.seed for record in records if record.seed is not None})
                valid_mean = mean([record.xy_valid_fraction for record in records])
                false_mean = mean([record.false_valid_fraction for record in records])
                rows.append(
                    [
                        SCENARIO_LABELS[scenario],
                        MODE_LABELS[mode],
                        COMM_LABELS[communication],
                        str(len(seeds)) if seeds else str(len(records)),
                        fmt(median(rmse)),
                        f"{fmt(min(rmse) if rmse else None)} ~ {fmt(max(rmse) if rmse else None)}",
                        fmt(median([record.p95_xy_m for record in records])),
                        fmt(median([record.longest_invalid_s for record in records])),
                        fmt(100 * valid_mean if valid_mean is not None else None, 1),
                        fmt(100 * false_mean if false_mean is not None else None, 1),
                    ]
                )
    return rows


def _board_rows(data: ReportData):
    rows = []
    for profile in data.board.completed():
        rows.append(
            [
                profile.name,
                fmt(profile.elapsed_s, 1),
                fmt(profile.achieved_hz("vision_detect_pnp"), 2),
                fmt(profile.achieved_hz("navigation"), 2),
                fmt(profile.process_cpu_percent, 1),
                fmt(profile.device_cpu_percent(), 1),
                fmt(100 * profile.skipped_fraction if profile.skipped_fraction is not None else None, 1),
                fmt(as_float(profile.data.get("deadline_misses")), 0),
                fmt(as_float(profile.data.get("tick_work", {}).get("p99_ms")), 2),
                fmt(as_float(profile.data.get("tick_work", {}).get("max_ms")), 2),
                fmt(profile.max_temperature(), 1),
            ]
        )
    return rows


def _incomplete_rows(data: ReportData):
    rows = []
    for profile in data.board.incomplete():
        entry = profile.entry or {}
        rows.append(
            [
                profile.name,
                fmt(as_float(entry.get("wall_seconds")), 1),
                str(entry.get("exit_status")),
                profile.short_failure_reason(),
            ]
        )
    return rows


def _stage_rows(profile):
    rows = []
    for stage, entry in profile.stages.items():
        if stage not in STAGE_LABELS:
            continue
        if not as_float(entry.get("samples")):
            continue
        p99 = as_float(entry.get("p99_ms")) or 0
        rows.append(
            (
                p99,
                [
                    STAGE_LABELS[stage],
                    fmt(as_float(entry.get("samples")), 0),
                    fmt(as_float(entry.get("mean_ms"))),
                    fmt(as_float(entry.get("p95_ms"))),
                    fmt(as_float(entry.get("p99_ms"))),
                    fmt(as_float(entry.get("max_ms"))),
                    fmt(as_float(entry.get("accepted")), 0),
                    fmt(as_float(entry.get("rejected")), 0),
                ],
            )
        )
    rows.sort(key=lambda item: item[0], reverse=True)
    return [row for _key, row in rows]


def _profile_rows(data: ReportData):
    rows = []
    for name, profile in data.scenarios.profiles.items():
        if not isinstance(profile, dict):
            continue
        rows.append(
            [name, *[fmt(as_float(profile.get(key)), digits) for key, _label, digits in PROFILE_FIELDS]]
        )
    return rows


def _robot_rows(data: ReportData):
    rows = []
    for robot in data.scenarios.robots:
        center = robot.get("center_m", [None, None, None])
        size = robot.get("size_m", [None, None, None])
        velocity = robot.get("velocity_mps") or [0, 0, 0]
        rows.append(
            [
                str(robot.get("id", "—")),
                str(robot.get("role", "—")),
                ", ".join(fmt(value, 2) for value in center),
                " × ".join(fmt(value, 2) for value in size),
                ", ".join(fmt(value, 2) for value in velocity),
            ]
        )
    return rows


def _bottleneck_rows(data: ReportData, limit=8):
    """按阶段取全部完成配置中最差的一次调用，避免同一阶段占满整表。"""
    worst = {}
    for profile in data.board.completed():
        for stage, entry in profile.stages.items():
            if stage not in STAGE_LABELS:
                continue
            if not as_float(entry.get("samples")):
                continue
            maximum = as_float(entry.get("max_ms")) or 0
            if stage not in worst or maximum > worst[stage][0]:
                worst[stage] = (
                    maximum,
                    [
                        STAGE_LABELS[stage],
                        profile.name,
                        fmt(as_float(entry.get("samples")), 0),
                        fmt(as_float(entry.get("mean_ms"))),
                        fmt(as_float(entry.get("p99_ms"))),
                        fmt(as_float(entry.get("max_ms"))),
                    ],
                )
    ordered = sorted(worst.values(), key=lambda item: item[0], reverse=True)
    return [row for _key, row in ordered[:limit]]


def _empty_stage_rows(data: ReportData, reference):
    """参考配置中没有调用、但在其他配置中有调用的阶段。"""
    empty = [
        stage
        for stage, entry in reference.stages.items()
        if stage in STAGE_LABELS and not as_float(entry.get("samples"))
    ]
    rows = []
    for stage in empty:
        covering = [
            profile.name
            for profile in data.board.completed()
            if as_float((profile.stages.get(stage) or {}).get("samples"))
        ]
        rows.append([STAGE_LABELS[stage], "、".join(covering) if covering else "无"])
    return rows


def _fingerprint_rows(data: ReportData, limit=24):
    rows = []
    seen = set()
    for record in data.localization.records:
        if record.input_fingerprint is None:
            continue
        key = (record.scenario, record.mode, record.seed)
        if key in seen:
            continue
        seen.add(key)
        rows.append(
            [
                SCENARIO_LABELS.get(record.scenario, record.scenario),
                MODE_LABELS.get(record.mode, record.mode),
                str(record.seed) if record.seed is not None else "—",
                record.input_fingerprint,
            ]
        )
    rows.sort(key=lambda row: (row[0], row[1], row[2]))
    return rows[:limit]


def _source_rows(data: ReportData):
    rows = []
    seen = set()
    for path in data.sources:
        key = str(path.resolve())
        if key in seen:
            continue
        seen.add(key)
        try:
            relative = path.relative_to(data.root).as_posix()
        except ValueError:
            relative = path.as_posix()
        rows.append([relative, str(path.stat().st_size), sha256_file(path)[:16]])
    return rows


def verify_pairing(data: ReportData):
    """用 runs 与 paired_by_seed 相互核对同种子配对差值。"""
    lookup = {}
    for record in data.localization.select(window="*"):
        lookup[(record.window, record.scenario, record.mode, record.seed, record.communication)] = record
    checked = 0
    worst = 0.0
    for row in data.localization.paired:
        scenario = canonical(row.get("scenario"), SCENARIO_ALIASES)
        mode = canonical(row.get("fusion") or row.get("mode"), MODE_ALIASES)
        seed_value = row.get("seed")
        seed = int(seed_value) if isinstance(seed_value, (int, float)) else None
        if scenario is None or mode is None or seed is None:
            continue
        window = (
            row.get("window")
            if isinstance(row.get("window"), str)
            else data.localization.primary_window()
        )
        for key, left_key, right_key in (
            ("delta_rmse_on_minus_off_m", "xy_rmse_on_m", "xy_rmse_off_m"),
            ("delta_rmse_intermittent_minus_off_m", "xy_rmse_intermittent_m", "xy_rmse_off_m"),
        ):
            left = as_float(row.get(left_key))
            right = as_float(row.get(right_key))
            delta = as_float(row.get(key))
            if left is not None and right is not None and delta is not None:
                worst = max(worst, abs((left - right) - delta))
                checked += 1
        for communication, key in (
            ("full", "xy_rmse_on_m"),
            ("off", "xy_rmse_off_m"),
            ("intermittent", "xy_rmse_intermittent_m"),
        ):
            record = lookup.get((window, scenario, mode, seed, communication))
            value = as_float(row.get(key))
            if record is not None and record.rmse_xy_m is not None and value is not None:
                worst = max(worst, abs(record.rmse_xy_m - value))
                checked += 1
    return checked, worst


def occlusion_stats(data: ReportData):
    """按场景汇总传感器侧的射线结果，取自 sensor_model 数组。"""
    window = data.localization.primary_window()
    grouped = {}
    for row in data.localization.sensor_model:
        row_window = row.get("window") if isinstance(row.get("window"), str) else "transit"
        if row_window != window:
            continue
        scenario = canonical(row.get("scenario"), SCENARIO_ALIASES)
        if scenario is None:
            continue
        grouped.setdefault(scenario, []).append(row)
    stats = {}
    for scenario, rows in grouped.items():
        stats[scenario] = {
            "rays": mean([as_float(row.get("rays")) for row in rows]),
            "valid_returns": mean([as_float(row.get("valid_returns")) for row in rows]),
            "no_return": mean([as_float(row.get("no_return")) for row in rows]),
            "out_of_range": mean([as_float(row.get("out_of_range")) for row in rows]),
            "dropped": mean([as_float(row.get("dropped")) for row in rows]),
            "occluded_rays": mean([as_float(row.get("robot_occluded_rays")) for row in rows]),
            "occluded_fraction": mean([as_float(row.get("robot_occluded_fraction")) for row in rows]),
            "robot_count": mean([as_float(row.get("robot_count")) for row in rows]),
        }
    return stats


def _occlusion_rows(data: ReportData):
    stats = occlusion_stats(data)
    rows = []
    for scenario in SCENARIO_ORDER:
        values = stats.get(scenario)
        if not values:
            continue
        rows.append(
            [
                SCENARIO_LABELS[scenario],
                fmt(values["rays"], 0),
                fmt(values["valid_returns"], 0),
                fmt(values["no_return"], 0),
                fmt(values["out_of_range"], 0),
                fmt(values["dropped"], 0),
                fmt(values["occluded_rays"], 0),
                fmt(
                    100 * values["occluded_fraction"]
                    if values["occluded_fraction"] is not None
                    else None,
                    2,
                ),
            ]
        )
    return rows


def _divergence_rows(data: ReportData, threshold=1.0):
    rows = []
    for record in divergence_records(data, threshold):
        rows.append(
            [
                SCENARIO_LABELS.get(record.scenario, record.scenario),
                MODE_LABELS.get(record.mode, record.mode),
                COMM_LABELS.get(record.communication, record.communication),
                str(record.seed) if record.seed is not None else "—",
                fmt(record.rmse_xy_m),
                fmt(record.max_xy_m),
                fmt(100 * record.xy_valid_fraction if record.xy_valid_fraction is not None else None, 1),
                fmt(
                    100 * record.false_valid_fraction
                    if record.false_valid_fraction is not None
                    else None,
                    1,
                ),
            ]
        )
    return rows


def _sensor_parameter_rows(data: ReportData):
    """融合模式与传感器参数，优先取自 C 的 rates_hz 与 sensor_noise。"""
    rates = data.localization.meta.get("rates_hz") or {}
    noise = data.localization.meta.get("sensor_noise") or {}
    tof_hz = as_float(rates.get("tof"))
    sensors = as_float(rates.get("tof_sensors"))
    imu_hz = as_float(rates.get("imu"))
    height_hz = as_float(rates.get("height_assist"))
    flow_hz = as_float(rates.get("optical_flow"))
    uwb_hz = as_float(rates.get("uwb"))
    rows = [
        [
            "三路 8×8 ToF",
            f"{fmt(tof_hz, 0)} Hz × {fmt(sensors, 0)} 路",
            "逐像素白噪声、距离尺度噪声、固定偏置与尺度偏差、逐帧公共误差、丢弃",
        ],
        [
            "IMU 预测",
            f"{fmt(imu_hz, 0)} Hz",
            f"加速度偏置 {fmt(as_float(noise.get('imu_accel_bias_mps2')))} m/s²、"
            f"白噪声 {fmt(as_float(noise.get('imu_accel_white_mps2')))} m/s²；"
            f"偏航偏置 {fmt(as_float(noise.get('imu_yaw_bias_rad')))} rad、"
            f"白噪声 {fmt(as_float(noise.get('imu_yaw_white_rad')))} rad",
        ],
        [
            "高度辅助",
            f"{fmt(height_hz, 0)} Hz",
            f"白噪声 {fmt(as_float(noise.get('height_white_m')))} m，使用带噪真值高度",
        ],
        [
            "光流",
            f"{fmt(flow_hz, 0)} Hz",
            f"尺度 {fmt(as_float(noise.get('flow_scale')))}、"
            f"白噪声 {fmt(as_float(noise.get('flow_white_mps')))} m/s",
        ],
        [
            "UWB",
            f"{fmt(uwb_hz, 0)} Hz",
            f"距离 σ {fmt(as_float(noise.get('uwb_range_sigma_m')))} m、"
            f"方位 σ {fmt(as_float(noise.get('uwb_bearing_sigma_rad')))} rad",
        ],
    ]
    return rows


def _initialization_rows(data: ReportData):
    init = data.localization.meta.get("initialization") or {}
    offset = init.get("offset_m") or []
    rows = [
        ["初始水平偏移 m", ", ".join(fmt(value) for value in offset) if offset else "—"],
        ["初始偏航偏移 rad", fmt(as_float(init.get("yaw_offset_rad")))],
        ["初始协方差对角", fmt(as_float(init.get("covariance_diagonal")))],
        ["逐帧真值重置", "否" if init.get("per_frame_truth_reset") is False else "是"],
        ["通信延迟 ms", fmt(as_float(init.get("communication_delay_ms")), 0)],
    ]
    return rows


class Numbering:
    """按出现顺序编号图表，避免增删内容后编号错位。"""

    def __init__(self):
        self.tables = 0
        self.figures = 0
        self.figure_numbers = {}

    def table(self, caption):
        self.tables += 1
        return f"表 {self.tables} {caption}"

    def figure(self, caption):
        self.figures += 1
        self.figure_numbers[caption] = self.figures
        return f"图 {self.figures} {caption}"

    def figure_number(self, caption):
        return self.figure_numbers.get(caption, "—")


def _figure_source_rows(numbering: Numbering):
    specs = [
        ("未知机器人占位俯视投影", "simulation/research/scenarios.json", "占位体 center_m 与 size_m 投影到地图平面"),
        ("各场景水平 RMSE（对数纵轴）", "localization-*.json", "每种子一个点，横条为组内 RMSE 中位数"),
        ("水平有效率与大误差仍标为有效的比例", "localization-*.json", "xy_valid 与 false_valid 样本比例均值"),
        ("通信受限相对完整共享的 RMSE 变化（纵轴对称对数）", "localization-*.json", "同种子配对差值：受限减完整共享"),
        ("板端实际调用频率与跳过槽位比例", "logs/research-20261023/*/", "samples 除以 elapsed_s，跳过比例取 skipped 与 completed"),
        ("板端阶段耗时", "logs/research-20261023/*/", "参考配置阶段 mean、p99 与 max，对数横轴"),
        ("板端 CPU、温度与可用内存", "logs/research-20261023/*/", "逐秒资源采样与监控通道整机 CPU"),
    ]
    rows = [
        [numbering.figure_number(caption), source, formula]
        for caption, source, formula in specs
    ]
    rows.sort(key=lambda row: row[0] if isinstance(row[0], int) else 0)
    return [[f"图 {row[0]}", row[1], row[2]] for row in rows]


def build_blocks(data: ReportData, figure_dir: Path) -> list:
    reference = data.board.reference()
    seeds = data.localization.seeds()
    numbering = Numbering()
    blocks = []
    blocks.append(h1("无人机定位与板端算力预研报告"))
    blocks.append(
        paragraph(
            "对象为 RMUC2027 地图条件下的多机定位与 LicheeRV Nano 板端算力。"
            "数据来自仿真敏感性实验、当前实机板端测量与同输入配对通信对照，"
            "全部数字由原始 JSON 重新计算。"
        )
    )

    blocks.append(h1("1 摘要与结论"))
    blocks.append(paragraph(_summary_text(data, reference, seeds)))
    blocks.append(paragraph(_conclusion_text(data, reference, seeds)))

    blocks.append(h1("2 研究范围与方案边界"))
    blocks.append(h2("2.1 定位链的组成"))
    blocks.append(
        paragraph(
            "本研究的定位结果由三路 8×8 ToF 测距与惯性、高度信息共同给出，不是纯 ToF 解算。"
            "ToF 每路 15 Hz 提供视场内的距离观测；IMU 以 100 Hz 提供姿态与加速度预测；"
            "高度辅助以 50 Hz 提供垂直方向约束；光流组在同一实验内以 50 Hz 提供机体速度约束。"
            "凡涉及精度、有效率与误判比例的结果，均随模式标注，不把光流与高度辅助的贡献并入 ToF。"
        )
    )
    blocks.append(h2("2.2 回放与闭环的差别"))
    blocks.append(
        paragraph(
            "实验输入是历史仿真运行记录的轨迹与按模型生成的测量，估计器在固定时间步上回放。"
            "回放不等同真实传感器采集与飞控闭环：相机曝光与取帧、ToF 硬件时序、传感器总线、"
            "MAVLink 传输、飞控执行与碰撞物理都不在数据链路内。"
            "因此本报告给出的是算法层在给定输入条件下的误差、有效率与负载，不是飞行试验的落点或命中结论。"
        )
    )
    blocks.append(h2("2.3 传感器与占位的假设性质"))
    blocks.append(
        paragraph(
            "测距误差按随机部分与设备固定偏差分开建模：逐帧白噪声、随距离增长的尺度噪声、"
            "逐设备固定偏置与尺度偏差、逐帧公共误差以及丢弃概率。"
            "这些取值是研究假设，用于给出误差随条件变化的敏感性，不是厂家保证值，也不替代实机标定。"
            "未知机器人占位是带碰撞与可视体的工程包络长方体，只用于遮挡 ToF 视线，"
            "尺寸不取自官方 CAD，位置按量程重叠的压力条件放置，不构成合法站位声明。"
        )
    )
    blocks.append(h2("2.4 地图同源与认证边界"))
    blocks.append(
        paragraph(
            "传感器观测与估计器先验使用同一静态地图网格，理想场景因此是自洽条件下的对照。"
            "理想地图与未知障碍物的对比属于敏感性分析，用于判断遮挡与噪声对误差和有效率的相对影响，"
            "不构成实物定位认证。结论的重复单位是独立随机种子，不使用同一条件下的更多帧提高置信度。"
        )
    )
    limitations = data.localization.meta.get("limitations") or []
    excluded = data.localization.meta.get("scenario_groups_excluded") or []
    blocks.append(h2("2.5 实验条件限制"))
    blocks.append(
        paragraph(
            "以下限制随全部精度结果同时成立："
            + "；".join(str(item) for item in limitations)
            + "。"
            + (
                "共享 schema 中未纳入实验的占位分组：" + "、".join(str(name) for name in excluded) + "。"
                if excluded
                else ""
            )
        )
    )

    blocks.append(h1("3 研究设计"))
    blocks.append(h2("3.1 场景与噪声配置"))
    blocks.append(
        paragraph(
            "场景按理想静态、仅测距噪声、仅未知占位、噪声与占位叠加、恶劣测距、局部 ToF 暂停分组，"
            "每组噪声参数在全部种子间固定，随机性只来自种子。"
        )
    )
    blocks.append(
        table(
            numbering.table("测距噪声配置"),
            ["配置", *[label for _key, label, _digits in PROFILE_FIELDS]],
            _profile_rows(data),
            ["l", *["r"] * len(PROFILE_FIELDS)],
        )
    )
    blocks.append(h2("3.2 未知障碍物占位"))
    geometry = data.scenarios.research.get("tof_geometry") or {}
    pressure = data.scenarios.research.get("pressure_robots") or []
    standing_note = data.scenarios.research.get("standing_note")
    geometry_text = ""
    if geometry:
        resolution = geometry.get("resolution") or []
        hfov = as_float(geometry.get("hfov_rad"))
        pitch = as_float(geometry.get("pitch_down_rad"))
        minimum = as_float(geometry.get("min_range_m"))
        maximum = as_float(geometry.get("max_range_m"))
        cruise = as_float(geometry.get("cruise_z_m"))
        geometry_text = (
            "ToF 视场为 "
            + "×".join(str(int(value)) for value in resolution if isinstance(value, (int, float)))
            + f" 分辨率，水平与垂直视场 {fmt(math.degrees(hfov) if hfov else None, 1)}°，"
            f"安装俯角 {fmt(math.degrees(pitch) if pitch else None, 1)}°，"
            f"量程 {fmt(minimum, 2)} 到 {fmt(maximum, 1)} m，"
            f"巡航高度 {fmt(cruise, 1)} m。"
        )
    pressure_text = (
        f"另有 {len(pressure)} 个托盘平台占位体用于制造贴身遮挡压力，属非比赛站位工况。"
        if pressure
        else ""
    )
    blocks.append(
        paragraph(
            "占位体只进入传感器视线与物理遮挡，不进入估计器先验；"
            "表中坐标与尺寸为工程包络，落地高度取自地形高度网格。"
            + geometry_text
            + pressure_text
            + (standing_note or "")
        )
    )
    blocks.append(
        table(
            numbering.table("未知机器人占位"),
            ["编号", "角色", "中心 m", "尺寸 m", "速度 m/s"],
            _robot_rows(data),
            ["l", "l", "r", "r", "r"],
        )
    )
    blocks.append(
        figure(numbering.figure("未知机器人占位俯视投影"), figure_dir / "fig-scene-occluders.png")
    )
    blocks.append(h2("3.3 传感器与融合模式"))
    blocks.append(
        paragraph(
            "两种融合模式共用同一批测量，区别只在光流支路是否启用。"
            "高度辅助使用带噪真值高度，属于仿真条件；两种模式都不是纯 ToF 解算。"
        )
    )
    blocks.append(
        table(
            numbering.table("传感器与频率"),
            ["环节", "频率", "误差模型"],
            _sensor_parameter_rows(data),
            ["l", "c", "l"],
        )
    )
    blocks.append(
        table(
            numbering.table("初始化与辅助条件"),
            ["项目", "取值"],
            _initialization_rows(data),
            ["l", "l"],
        )
    )
    blocks.append(h2("3.4 通信对照的公平性"))
    blocks.append(
        paragraph(
            "通信条件分完整共享、完全关闭与间歇中断三种，同一场景与种子使用同一批测量输入。"
            "共享时队友状态来自各机估计器，真值只用于生成传感器观测与离线评分；"
            "关闭时估计器不调用队友融合。间歇条件按固定丢包比例与定时中断施加，不改变测量本身。"
            "输入指纹用于确认同一场景与种子在各通信条件下使用一致输入。"
        )
    )
    blocks.append(h2("3.5 统计口径与指标定义"))
    blocks.append(
        paragraph(
            "重复单位是独立随机种子，同一条件下的连续帧不构成独立重复。"
            "误差为世界坐标系水平 RMSE，取全部评分样本；有效率分结构定位有效与总有效两类；"
            "大误差仍标为有效指误差超过 0.1 m 且仍被标为有效的样本占全部样本的比例；"
            "失连续时间为最长不有效区间；队友更新接受率为被接受的队友约束占全部队友更新的比例。"
            "板端负载同时给出平均 CPU、P99 与最长卡顿以及导航实际频率。"
        )
    )

    blocks.append(h1("4 板端算力与调度"))
    completed_profiles = data.board.completed()
    profile_names = "、".join(profile.name for profile in completed_profiles)
    cpufreq_available = (reference.monitor or {}).get("cpufreq_available")
    cpufreq_note = (
        "设备没有可用的 cpufreq 节点，CPU 频率不可用。"
        if cpufreq_available is False
        else ""
    )
    blocks.append(
        paragraph(
            f"测量在 {data.board.directory.name} 目录下的运行中进行，本批包含 "
            f"{len(completed_profiles)} 组完成校验的配置：{profile_names}。"
            f"阶段耗时与资源曲线取参考配置 {reference.name}。"
            "进程 CPU 为进程 CPU 时间除以实际墙钟时间，100% 对应占满一个核心；"
            "整机 CPU、1 分钟负载、可用内存与温度由独立监控通道每秒采样，与该进程的进程级读数分开列出。"
            "频率为实测调用次数除以运行时长，不用理论帧率替代；"
            "单轮 P99 与最长耗时统计计算调用，不含等待下一周期的睡眠。"
            "计时范围不含相机取帧、ToF 硬件时序、MAVLink 传输、执行器与 UWB 射频。"
            + cpufreq_note
            + "本测量范围不含加速器，报告中不推断加速器使用情况。"
        )
    )
    blocks.append(
        table(
            numbering.table("定频回放结果"),
            [
                "配置",
                "时长 s",
                "实测视觉 Hz",
                "实测导航 Hz",
                "进程 CPU %",
                "整机 CPU %",
                "跳过槽位 %",
                "截止期错过",
                "单轮 P99 ms",
                "最长单轮 ms",
                "最高温度 °C",
            ],
            _board_rows(data),
            ["l", *["r"] * 10],
        )
    )
    blocks.append(
        figure(
            numbering.figure("板端实际调用频率与跳过槽位比例"),
            figure_dir / "fig-board-rates.png",
        )
    )
    incomplete = data.board.incomplete()
    if incomplete:
        blocks.append(paragraph(_incomplete_text(data, reference)))
        blocks.append(
            table(
                numbering.table("未完成配置"),
                ["配置", "墙钟 s", "退出状态", "判定"],
                _incomplete_rows(data),
                ["l", "r", "r", "l"],
            )
        )
    blocks.append(
        table(
            numbering.table("阶段耗时"),
            ["阶段", "调用数", "平均 ms", "P95 ms", "P99 ms", "最长 ms", "接受", "拒绝"],
            _stage_rows(reference),
            ["l", *["r"] * 7],
        )
    )
    empty_rows = _empty_stage_rows(data, reference)
    if empty_rows:
        blocks.append(
            paragraph(
                "参考配置中没有调用的阶段在表中省略："
                + "；".join(f"{row[0]}（在 {row[1]} 中有调用）" for row in empty_rows)
                + "。"
            )
        )
    blocks.append(figure(numbering.figure("板端阶段耗时"), figure_dir / "fig-board-stages.png"))
    blocks.append(
        figure(numbering.figure("板端 CPU、温度与可用内存"), figure_dir / "fig-board-resources.png")
    )

    blocks.append(h1("5 ToF 定位误差与有效率"))
    seed_text = f"（{', '.join(str(seed) for seed in seeds)}）" if seeds else ""
    blocks.append(
        paragraph(
            "误差与有效率按场景、融合模式与通信条件分组，每个种子独立运行。"
            f"本批数据含 {len(seeds) if seeds else 0} 个独立种子{seed_text}，结论的重复单位是种子。"
        )
    )
    blocks.append(
        figure(numbering.figure("各场景水平 RMSE（对数纵轴）"), figure_dir / "fig-loc-rmse.png")
    )
    blocks.append(
        table(
            numbering.table("误差与有效率汇总"),
            [
                "场景",
                "模式",
                "通信",
                "种子数",
                "RMSE 中位数 m",
                "种子范围 m",
                "P95 中位数 m",
                "最长失连续 s",
                "总有效率 %",
                "大误差仍有效 %",
            ],
            _table_rows(data),
            ["l", "l", "l", "r", "r", "r", "r", "r", "r", "r"],
        )
    )
    blocks.append(paragraph(_divergence_text(data)))
    divergence_rows = _divergence_rows(data)
    if divergence_rows:
        blocks.append(
            table(
                numbering.table("发散样本"),
                ["场景", "模式", "通信", "种子", "RMSE m", "最大误差 m", "总有效率 %", "大误差仍有效 %"],
                divergence_rows,
                ["l", "l", "l", "r", "r", "r", "r", "r"],
            )
        )
    blocks.append(
        figure(
            numbering.figure("水平有效率与大误差仍标为有效的比例"),
            figure_dir / "fig-loc-valid.png",
        )
    )
    occlusion_rows = _occlusion_rows(data)
    if occlusion_rows:
        blocks.append(
            table(
                numbering.table("传感器遮挡与射线结果"),
                ["场景", "射线总数", "有效返回", "无返回", "超量程", "丢弃", "被占位遮挡", "遮挡比例 %"],
                occlusion_rows,
                ["l", *["r"] * 7],
            )
        )
    blocks.append(paragraph(_occlusion_text(data)))

    blocks.append(h1("6 多机通信对照"))
    blocks.append(
        paragraph(
            "通信对照在同一场景、同一模式与同一种子下配对比较，"
            "受限条件的误差变化定义为该条件 RMSE 减去完整共享 RMSE，正值为误差增大。"
        )
    )
    blocks.append(
        figure(
            numbering.figure("通信受限相对完整共享的 RMSE 变化（纵轴对称对数）"),
            figure_dir / "fig-comm-delta.png",
        )
    )
    group_count, consistent = fingerprint_consistency(data)
    if group_count:
        blocks.append(
            paragraph(
                f"输入指纹按窗口、场景、模式与种子分组共 {group_count} 组，"
                f"其中 {consistent} 组在三种通信条件下完全一致，"
                "用于确认配对对照使用同一批测量输入。"
            )
        )
    blocks.append(paragraph(_communication_text(data)))
    blocks.append(paragraph(_pairing_text(data)))

    blocks.append(h1("7 瓶颈与决策"))
    blocks.append(paragraph(_bottleneck_text(data, reference)))
    blocks.append(
        table(
            numbering.table("最长单次调用排序"),
            ["阶段", "最差配置", "调用数", "平均 ms", "P99 ms", "最长 ms"],
            _bottleneck_rows(data),
            ["l", "l", "r", "r", "r", "r"],
        )
    )
    blocks.append(paragraph(_decision_text(data, reference)))

    blocks.append(h1("8 后续验证"))
    blocks.append(paragraph(_followup_text(data)))

    blocks.append(h1("9 复现附录"))
    blocks.append(h2("9.1 生成命令"))
    blocks.append(
        code(
            "$ErrorActionPreference = 'Stop'\n"
            "$reportPython = '<workspace 依赖运行时>/python.exe'\n"
            "$poppler = '<workspace 依赖运行时>/native/poppler/Library/bin'\n"
            "& '.venv/Scripts/python.exe' -X utf8 'scripts/build_research_20261023_report.py' figures\n"
            "if ($LASTEXITCODE -ne 0) { throw '图表生成失败' }\n"
            "& '.venv/Scripts/python.exe' -X utf8 'scripts/build_research_20261023_report.py' markdown\n"
            "if ($LASTEXITCODE -ne 0) { throw 'Markdown 生成失败' }\n"
            "& $reportPython -X utf8 'scripts/build_research_20261023_report.py' pdf\n"
            "if ($LASTEXITCODE -ne 0) { throw 'PDF 生成失败' }\n"
            "& \"$poppler\\pdftoppm.exe\" -png -r 110 'docs/research-20261023/report.pdf' 'tmp/report-pages/page'\n"
            "if ($LASTEXITCODE -ne 0) { throw 'PDF 页面渲染失败' }"
        )
    )
    blocks.append(
        paragraph(
            "图表阶段使用仓库 .venv 的 matplotlib；Markdown 与 PDF 阶段由 workspace 依赖运行时执行，"
            "该运行时提供 reportlab；逐页 PNG 检查使用同一运行时中的 Poppler。"
            "$reportPython 与 $poppler 按该运行时在本机的实际路径填写。"
        )
    )
    blocks.append(h2("9.2 输入数据清单"))
    blocks.append(
        table(
            numbering.table("输入文件与摘要"),
            ["文件", "字节", "SHA-256 前 16 位"],
            _source_rows(data),
            ["l", "r", "l"],
        )
    )
    blocks.append(h2("9.3 图表口径"))
    blocks.append(
        table(
            numbering.table("图表输入与计算式"),
            ["图", "输入", "计算式"],
            _figure_source_rows(numbering),
            ["l", "l", "l"],
        )
    )
    blocks.append(h2("9.4 数值核对"))
    blocks.append(paragraph(_validation_text(data)))
    extra_windows = [window for window in data.localization.windows() if window != data.localization.primary_window()]
    if extra_windows:
        blocks.append(
            paragraph(
                "定位实验同时覆盖了 "
                + "、".join(extra_windows)
                + " 窗口，主表与主图只使用 "
                + data.localization.primary_window()
                + " 窗口；其余窗口按下表单独给出，差值为相对同场景同模式的 "
                + data.localization.primary_window()
                + " 窗口中位数。"
            )
        )
        blocks.append(
            table(
                numbering.table("其余窗口对照"),
                ["场景", "模式", "种子数", "RMSE 中位数 m", "总有效率 %", "与主窗口差值 m"],
                [
                    row
                    for window in extra_windows
                    for row in _window_detail_rows(data, window)
                ],
                ["l", "l", "r", "r", "r", "r"],
            )
        )
    blocks.append(h2("9.5 来源"))
    blocks.append(
        paragraph(
            "场景噪声与占位参数、板端测量与定位消融实验分别来自场地与测距干扰、板端负载、"
            "通信消融三项实验，对应说明文档为 docs/research-20261023/tof-sources.md、"
            "docs/research-20261023/nano-findings.md 与 docs/research-20261023/localization-findings.md。"
            "本报告的数字全部由上述原始 JSON 重新计算，未使用二手汇总值。"
        )
    )
    return blocks


# ---------------------------------------------------------------- 正文


def _summary_text(data: ReportData, reference, seeds) -> str:
    localization = data.localization
    ideal = localization.select(scenario="ideal")
    combined = localization.select(scenario="combined")
    adverse = localization.select(scenario="adverse")
    parts = [
        f"本报告汇总 {len(localization.records)} 条定位运行记录、"
        f"{len(data.board.completed())} 组完成校验的板端测量"
        + (f"与 {len(data.board.incomplete())} 组未完成配置。" if data.board.incomplete() else "。")
    ]
    if ideal:
        parts.append(
            f"理想静态场景的水平 RMSE 中位数为 {fmt(median([record.rmse_xy_m for record in ideal]))} m，"
            f"总有效率中位数 {fmt_ratio(median([record.xy_valid_fraction for record in ideal]))}。"
        )
    if combined:
        parts.append(
            f"噪声与占位叠加后 RMSE 中位数 {fmt(median([record.rmse_xy_m for record in combined]))} m，"
            f"总有效率中位数 {fmt_ratio(median([record.xy_valid_fraction for record in combined]))}，"
            f"大误差仍标为有效的比例中位数 "
            f"{fmt_ratio(median([record.false_valid_fraction for record in combined]))}。"
        )
    if adverse:
        parts.append(
            f"恶劣测距条件下 RMSE 中位数 {fmt(median([record.rmse_xy_m for record in adverse]))} m，"
            f"最长失连续时间中位数 {fmt(median([record.longest_invalid_s for record in adverse]))} s。"
        )
    diverged = divergence_records(data)
    if diverged:
        parts.append(
            f"其中 {len(diverged)} 条运行的水平 RMSE 超过 1.0 m，"
            f"最大 {fmt(diverged[0].rmse_xy_m)} m。"
        )
    skipped = reference.skipped_fraction
    parts.append(
        f"板端参考配置 {reference.name} 的实测视觉频率 "
        f"{fmt(reference.achieved_hz('vision_detect_pnp'), 2)} Hz、"
        f"实测导航频率 {fmt(reference.achieved_hz('navigation'), 2)} Hz、"
        f"进程 CPU {fmt(reference.process_cpu_percent, 1)}%、"
        f"最长单轮 {fmt(as_float(reference.data.get('tick_work', {}).get('max_ms')), 1)} ms、"
        f"跳过槽位比例 {fmt(100 * skipped if skipped is not None else None, 1)}%。"
    )
    if seeds:
        parts.append(f"定位结论的独立重复为 {len(seeds)} 个种子。")
    return "".join(parts)


def _conclusion_text(data: ReportData, reference, seeds) -> str:
    parts = []
    for communication, label in (("off", "完全关闭"), ("intermittent", "间歇中断")):
        sentence = _paired_sentence(data, communication, label)
        if sentence:
            parts.append(sentence)
    if parts:
        parts.append(
            "配对差值只覆盖本批场景与种子，不构成协同融合普遍提升或普遍退化的结论。"
            + _divergence_text(data)
        )
    bottleneck = _bottleneck_rows(data, limit=3)
    if bottleneck:
        parts.append(
            "最长单次调用依次为："
            + "、".join(f"{row[0]}（{row[1]}，{row[5]} ms）" for row in bottleneck)
            + "，调度决策按该排序给出。"
        )
    if not parts:
        parts.append("本批数据不足以给出跨条件结论。")
    return "".join(parts)


def _occlusion_text(data: ReportData) -> str:
    occluders = data.localization.select(scenario="occluders")
    ideal = data.localization.select(scenario="ideal")
    stats = occlusion_stats(data)
    parts = []
    occluded = stats.get("occluders") or {}
    if occluded.get("occluded_rays") is not None:
        parts.append(
            "仅未知占位场景下，被占位体挡住的射线每帧平均 "
            f"{fmt(occluded.get('occluded_rays'), 0)} 条，"
            f"占全部射线 {fmt_ratio(occluded.get('occluded_fraction'))}。"
        )
    ideal_stats = stats.get("ideal") or {}
    if ideal_stats.get("no_return") is not None:
        parts.append(
            f"理想场景每帧射线 {fmt(ideal_stats.get('rays'), 0)} 条，"
            f"有效返回 {fmt(ideal_stats.get('valid_returns'), 0)} 条，"
            f"无返回 {fmt(ideal_stats.get('no_return'), 0)} 条，"
            f"超量程 {fmt(ideal_stats.get('out_of_range'), 0)} 条。"
        )
    if occluders and ideal:
        parts.append(
            "把每个场景的模式与通信条件合并后，仅未知占位的 RMSE 中位数为 "
            f"{fmt(median([record.rmse_xy_m for record in occluders]))} m，"
            f"理想静态为 {fmt(median([record.rmse_xy_m for record in ideal]))} m；"
            "两者之差是占位遮挡在本仿真条件下的敏感度量。"
        )
    blackout = data.localization.select(scenario="blackout")
    if blackout:
        parts.append(
            "局部 ToF 暂停场景的最长失连续时间中位数 "
            f"{fmt(median([record.longest_invalid_s for record in blackout]))} s，"
            "用于判断暂停段是否被高度与惯性预测覆盖。"
        )
    parts.append(
        "初始位置与偏航按固定偏差施加且过程中不重置，因此表中的误差包含出生误差的收敛过程。"
    )
    return "".join(parts)


def _incomplete_text(data: ReportData, reference) -> str:
    parts = []
    for profile in data.board.incomplete():
        entry = profile.entry or {}
        verification = profile.verification
        failures = verification.get("failures")
        failures = failures if isinstance(failures, list) else []
        missing_stages = [
            str(item).split(":", 1)[1]
            for item in failures
            if isinstance(item, str) and item.startswith("missing_required_stages:")
        ]
        detail = profile.oom_line()
        parts.append(
            f"{profile.name} 在 {fmt(as_float(entry.get('wall_seconds')), 1)} s 后结束，"
            f"退出状态 {entry.get('exit_status')}"
            + (f"，内核记录显示 {detail}" if detail else "")
            + "。"
            + (
                f"该次运行没有产生任何阶段样本，缺失阶段共 {len(missing_stages[0].split(','))} 个。"
                if missing_stages
                else ""
            )
        )
    if data.board.missing:
        parts.append("未启动的配置：" + "、".join(data.board.missing) + "。")
    parts.append(
        "该配置不计入定频回放表，也不作为算力结论依据；"
        "失败原因是当前板端在整幅地形输入下的可用内存上限，"
        "需要在降低地形规模或提高可用内存后重新测量。"
    )
    return "".join(parts)


def _pairing_text(data: ReportData) -> str:
    checked, worst = verify_pairing(data)
    if not checked:
        return "配对差值由运行记录直接重算，输入侧没有独立的逐种子配对表可供比对。"
    return (
        "配对差值同时由运行记录重算，并与输入侧的逐种子配对表逐项比对："
        f"{checked} 项一致，最大偏差 {worst:.2e}。"
    )


def _validation_text(data: ReportData) -> str:
    checked, worst = verify_pairing(data)
    windows = data.localization.windows()
    parts = [
        f"报告中的表格与图表均由输入 JSON 重新计算，未引用输入文件中的汇总值。"
        f"定位记录 {len(data.localization.records)} 条，"
        f"独立种子 {len(data.localization.seeds())} 个，"
        f"窗口 {'、'.join(windows) if windows else '—'}。"
    ]
    if data.localization.sensor_model:
        parts.append(f"传感器侧记录 {len(data.localization.sensor_model)} 条，用于遮挡与射线统计。")
    if checked:
        parts.append(f"配对差值重算与逐种子配对表比对 {checked} 项，最大偏差 {worst:.2e}。")
    group_count, consistent = fingerprint_consistency(data)
    if group_count:
        parts.append(
            f"输入指纹分组 {group_count} 组，其中 {consistent} 组在三种通信条件下一致。"
        )
    parts.append(
        "板端表格取自测量 JSON 的 stages、tick_work 与 resources 字段；"
        "整机 CPU 由独立监控通道的 /proc/stat 计数逐间隔重算。"
    )
    return "".join(parts)


def _window_rows(data: ReportData, windows):
    rows = []
    for window in windows:
        records = data.localization.select(window=window)
        if not records:
            continue
        seeds = {record.seed for record in records if record.seed is not None}
        rows.append(
            [
                window,
                str(len(records)),
                str(len(seeds)),
                fmt(median([record.rmse_xy_m for record in records])),
                fmt_ratio(mean([record.xy_valid_fraction for record in records])),
            ]
        )
    return rows


def paired_deltas(data: ReportData, communication: str, mode=None):
    """受限条件相对完整共享的逐种子配对差值，仅在同场景、同模式、同种子内比较。"""
    base = {
        (record.mode, record.scenario, record.seed): record.rmse_xy_m
        for record in data.localization.select(communication="full")
    }
    deltas = []
    for record in data.localization.select(communication=communication, mode=mode):
        reference = base.get((record.mode, record.scenario, record.seed))
        if reference is not None and record.rmse_xy_m is not None:
            deltas.append(record.rmse_xy_m - reference)
    return deltas


def divergence_records(data: ReportData, threshold=1.0):
    """水平 RMSE 超过阈值的发散样本，按误差从大到小排列。"""
    records = [
        record
        for record in data.localization.select()
        if record.rmse_xy_m is not None and record.rmse_xy_m > threshold
    ]
    records.sort(key=lambda record: record.rmse_xy_m, reverse=True)
    return records


def fingerprint_consistency(data: ReportData):
    """按窗口、场景、模式、种子分组，检查三种通信条件是否使用同一输入指纹。"""
    groups = {}
    for record in data.localization.select(window="*"):
        if record.input_fingerprint is None:
            continue
        key = (record.window, record.scenario, record.mode, record.seed)
        groups.setdefault(key, set()).add(record.input_fingerprint)
    consistent = sum(1 for values in groups.values() if len(values) == 1)
    return len(groups), consistent


def _divergence_text(data: ReportData, threshold=1.0) -> str:
    total = len(data.localization.select())
    diverged = divergence_records(data, threshold)
    if not diverged:
        return f"本批 {total} 条运行中没有水平 RMSE 超过 {threshold:.1f} m 的发散样本。"
    worst = diverged[0]
    scenarios = "、".join(
        sorted({SCENARIO_LABELS.get(record.scenario, record.scenario) for record in diverged})
    )
    modes = "、".join(sorted({MODE_LABELS.get(record.mode, record.mode) for record in diverged}))
    communications = "、".join(
        sorted({COMM_LABELS.get(record.communication, record.communication) for record in diverged})
    )
    return (
        f"本批 {total} 条运行中有 {len(diverged)} 条水平 RMSE 超过 {threshold:.1f} m，"
        f"出现在{scenarios}场景、{modes}模式与{communications}通信条件下。"
        f"最差一条为{SCENARIO_LABELS.get(worst.scenario)}、{MODE_LABELS.get(worst.mode)}、"
        f"{COMM_LABELS.get(worst.communication)}、种子 {worst.seed}，"
        f"RMSE {fmt(worst.rmse_xy_m)} m，总有效率 {fmt_ratio(worst.xy_valid_fraction)}。"
        "这些样本属于估计器发散，与分位数一起报告，中位数不能代表它们。"
    )


def _paired_sentence(data: ReportData, communication: str, label: str) -> str:
    deltas = paired_deltas(data, communication)
    if not deltas:
        return ""
    large = [value for value in deltas if abs(value) > 0.5]
    increased = sum(1 for value in deltas if value > 0)
    return (
        f"相对完整共享，{label}的逐种子配对 RMSE 差值中位数为 {fmt(median(deltas))} m，"
        f"四分位区间 {fmt(percentile(deltas, 0.25))} 到 {fmt(percentile(deltas, 0.75))} m，"
        f"{increased} / {len(deltas)} 组误差增大，其中 {len(large)} 组绝对变化超过 0.5 m。"
    )


def _window_detail_rows(data: ReportData, window: str):
    primary = data.localization.primary_window()
    rows = []
    for scenario in SCENARIO_ORDER:
        for mode in MODE_ORDER:
            records = data.localization.select(scenario=scenario, mode=mode, window=window)
            if not records:
                continue
            current = median([record.rmse_xy_m for record in records])
            base = median(
                [
                    record.rmse_xy_m
                    for record in data.localization.select(scenario=scenario, mode=mode, window=primary)
                ]
            )
            delta = current - base if current is not None and base is not None else None
            valid_mean = mean([record.xy_valid_fraction for record in records])
            rows.append(
                [
                    SCENARIO_LABELS[scenario],
                    MODE_LABELS[mode],
                    str(len({record.seed for record in records if record.seed is not None})),
                    fmt(current),
                    fmt(100 * valid_mean if valid_mean is not None else None, 1),
                    fmt(delta),
                ]
            )
    return rows


def _communication_text(data: ReportData) -> str:
    parts = []
    for mode in MODE_ORDER:
        for communication in ("off", "intermittent"):
            deltas = paired_deltas(data, communication, mode=mode)
            if not deltas:
                continue
            large = [value for value in deltas if abs(value) > 0.5]
            increased = sum(1 for value in deltas if value > 0)
            parts.append(
                f"{MODE_LABELS[mode]}、{COMM_LABELS[communication]} 的配对差值中位数 "
                f"{fmt(median(deltas))} m，四分位区间 {fmt(percentile(deltas, 0.25))} 到 "
                f"{fmt(percentile(deltas, 0.75))} m，{increased} / {len(deltas)} 组误差增大，"
                f"其中 {len(large)} 组绝对变化超过 0.5 m。"
            )
    rates = [
        record.uwb_accept_rate
        for record in data.localization.records
        if record.uwb_accept_rate is not None
    ]
    if rates:
        parts.append(
            f"队友更新接受率平均 {fmt_ratio(mean(rates))}，"
            "因此共享与关闭的差别只来自少量被接受的队友约束。"
        )
    parts.append(
        "配对比较只在同场景、同模式、同种子内进行；关闭条件不调用队友融合，"
        "共享条件的队友状态来自估计器而非真值。"
    )
    return "".join(parts)


def _bottleneck_text(data: ReportData, reference) -> str:
    skipped = reference.skipped_fraction
    parts = [
        f"参考配置 {reference.name} 的跳过槽位比例为 "
        f"{fmt(100 * skipped if skipped is not None else None, 1)}%，"
        f"单轮 P99 {fmt(as_float(reference.data.get('tick_work', {}).get('p99_ms')), 1)} ms、"
        f"最长 {fmt(as_float(reference.data.get('tick_work', {}).get('max_ms')), 1)} ms，"
        f"截止期错过 {fmt(as_float(reference.data.get('deadline_misses')), 0)} 次。"
    ]
    heavy = [
        (stage, as_float(entry.get("max_ms")) or 0)
        for stage, entry in reference.stages.items()
        if stage in STAGE_LABELS
    ]
    heavy.sort(key=lambda item: item[1], reverse=True)
    if heavy:
        parts.append(
            "单次调用最长的阶段为："
            + "、".join(f"{STAGE_LABELS[stage]}（{fmt(value, 1)} ms）" for stage, value in heavy[:3])
            + "；调度预算应优先覆盖这些阶段，并保留最新输入策略，不补执行积压槽位。"
        )
    return "".join(parts)


def _decision_text(data: ReportData, reference) -> str:
    return (
        "决策顺序为：第一，把单次调用最长的阶段移出控制循环或限制每轮计算预算，"
        "控制循环只读取已完成结果；第二，为 ToF、视觉与控制分别设定调度预算，保留时间戳与最新帧策略；"
        "第三，在真实相机帧下复测视觉分支，再决定视觉频率配置；"
        "第四，定位精度结论按模式与通信条件分别引用，不把理想地图同源自洽结果当作实物定位指标。"
        f"当前 {reference.name} 的实测导航频率与请求值的差距是是否需要降载的直接判据。"
    )


def _followup_text(data: ReportData) -> str:
    return (
        "后续验证按以下顺序安排：第一，用实机采集的 ToF 与图像数据重新标定噪声参数，"
        "替换本报告的假设取值；第二，在真实多机链路上重复通信对照，覆盖丢包、延迟与断链恢复；"
        "第三，把回放链路接入飞控闭环，验证定位误差对轨迹与接触结果的传递；"
        "第四，延长板端持续运行时间，观察温度与内存的长期趋势；"
        "第五，在更多独立种子上重复遮挡与恶劣测距场景，确认误差分位数的稳定性。"
    )


# ---------------------------------------------------------------- Markdown


def to_markdown(blocks) -> str:
    lines = []
    for block in blocks:
        kind = block[0]
        if kind == "h1":
            lines += ["", f"## {block[1]}", ""]
        elif kind == "h2":
            lines += ["", f"### {block[1]}", ""]
        elif kind == "p":
            lines += [block[1], ""]
        elif kind == "table":
            _kind, caption, headers, rows, aligns = block
            lines += [f"**{caption}**", ""]
            lines.append("| " + " | ".join(str(header) for header in headers) + " |")
            lines.append("|" + "|".join(":--" if align == "l" else "--:" for align in aligns) + "|")
            for row in rows:
                lines.append("| " + " | ".join(str(cell) for cell in row) + " |")
            lines.append("")
        elif kind == "figure":
            _kind, caption, path = block
            lines += [f"![{caption}](figures/{Path(path).name})", "", f"*{caption}*", ""]
        elif kind == "code":
            lines += ["```powershell", block[1], "```", ""]
    return "\n".join(lines).strip() + "\n"


# ---------------------------------------------------------------- PDF


PDF_FONTS = [
    ("ReportBody", r"C:\Windows\Fonts\msyh.ttc", 0),
    ("ReportBold", r"C:\Windows\Fonts\msyhbd.ttc", 0),
]


def to_pdf(blocks, output: Path):
    from PIL import Image as PILImage
    from reportlab.lib import colors
    from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT, TA_RIGHT
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import ParagraphStyle
    from reportlab.lib.units import cm
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from reportlab.platypus import (
        BaseDocTemplate,
        Frame,
        Image,
        KeepTogether,
        PageTemplate,
        Paragraph,
        Table,
        TableStyle,
    )

    for name, path, index in PDF_FONTS:
        pdfmetrics.registerFont(TTFont(name, path, subfontIndex=index))

    body = ParagraphStyle(
        "body",
        fontName="ReportBody",
        fontSize=10,
        leading=15,
        alignment=TA_LEFT,
        spaceAfter=6,
        textColor=colors.black,
        wordWrap="CJK",
    )
    title_style = ParagraphStyle(
        "title",
        parent=body,
        fontName="ReportBold",
        fontSize=19,
        leading=26,
        alignment=TA_LEFT,
        spaceAfter=8,
    )
    heading1 = ParagraphStyle(
        "h1",
        parent=body,
        fontName="ReportBold",
        fontSize=14,
        leading=20,
        spaceBefore=14,
        spaceAfter=6,
        keepWithNext=1,
    )
    heading2 = ParagraphStyle(
        "h2",
        parent=body,
        fontName="ReportBold",
        fontSize=11.5,
        leading=17,
        spaceBefore=10,
        spaceAfter=4,
        keepWithNext=1,
    )
    heading2_plain = ParagraphStyle("h2plain", parent=heading2, keepWithNext=0)
    figure_caption_style = ParagraphStyle(
        "caption",
        parent=body,
        fontSize=8.5,
        leading=12,
        alignment=TA_CENTER,
        spaceBefore=3,
        spaceAfter=10,
    )
    table_caption_style = ParagraphStyle(
        "tablecaption",
        parent=body,
        fontName="ReportBold",
        fontSize=9,
        leading=13,
        alignment=TA_LEFT,
        spaceBefore=6,
        spaceAfter=4,
        keepWithNext=1,
    )
    table_caption_plain = ParagraphStyle(
        "tablecaptionplain",
        parent=table_caption_style,
        keepWithNext=0,
    )
    code_style = ParagraphStyle(
        "code",
        parent=body,
        fontSize=8.5,
        leading=13,
        alignment=TA_LEFT,
        leftIndent=10,
        spaceAfter=6,
    )
    align_map = {"l": TA_LEFT, "c": TA_CENTER, "r": TA_RIGHT}
    cell_styles = {}
    for align, value in align_map.items():
        cell_styles[align] = ParagraphStyle(
            f"cell-{align}", parent=body, fontSize=8, leading=11, alignment=value, spaceAfter=0
        )
    header_styles = {}
    for align, value in align_map.items():
        header_styles[align] = ParagraphStyle(
            f"head-{align}",
            parent=body,
            fontName="ReportBold",
            fontSize=8,
            leading=11,
            alignment=value,
            spaceAfter=0,
            textColor=colors.white,
        )

    page_width, page_height = A4
    margin_x, margin_top, margin_bottom = 2.1 * cm, 1.9 * cm, 1.7 * cm
    frame = Frame(
        margin_x,
        margin_bottom,
        page_width - 2 * margin_x,
        page_height - margin_top - margin_bottom,
        id="body",
        leftPadding=0,
        rightPadding=0,
        topPadding=0,
        bottomPadding=0,
    )

    def decorate(canvas, document):
        canvas.saveState()
        canvas.setFont("ReportBody", 8)
        canvas.setFillColor(colors.HexColor("#666666"))
        canvas.drawString(margin_x, margin_bottom / 2, "无人机定位与板端算力预研报告")
        canvas.drawRightString(page_width - margin_x, margin_bottom / 2, f"第 {document.page} 页")
        canvas.restoreState()

    document = BaseDocTemplate(
        str(output),
        pagesize=A4,
        leftMargin=margin_x,
        rightMargin=margin_x,
        topMargin=margin_top,
        bottomMargin=margin_bottom,
        title="无人机定位与板端算力预研报告",
    )
    document.addPageTemplates([PageTemplate(id="body", frames=[frame], onPage=decorate)])

    available = page_width - 2 * margin_x

    def make_table(headers, rows, aligns):
        size = 7.2 if len(headers) >= 9 else 8
        leading = size + 3
        local_cells = {}
        local_headers = {}
        for align, value in align_map.items():
            local_cells[align] = ParagraphStyle(
                f"cell-{align}-{size}",
                parent=body,
                fontSize=size,
                leading=leading,
                alignment=value,
                spaceAfter=0,
            )
            local_headers[align] = ParagraphStyle(
                f"head-{align}-{size}",
                parent=body,
                fontName="ReportBold",
                fontSize=size,
                leading=leading,
                alignment=value,
                spaceAfter=0,
                textColor=colors.white,
            )
        data = [
            [
                Paragraph(_pdf_text(header), local_headers[aligns[index]])
                for index, header in enumerate(headers)
            ]
        ]
        for row in rows:
            data.append(
                [
                    Paragraph(_pdf_text(cell), local_cells[aligns[index]])
                    for index, cell in enumerate(row)
                ]
            )
        weights = []
        for index, header in enumerate(headers):
            units = sorted(
                [_text_units(header)] + [_text_units(row[index]) for row in rows]
            )
            floor = 6.5 if aligns[index] == "r" else 4.0
            weights.append(max(floor, min(units[int(len(units) * 0.9)], 40)))
        if len(headers) >= 9 and weights[0] >= 10:
            weights[0] *= 1.3
        total = sum(weights)
        widths = [available * weight / total for weight in weights]
        flow = Table(data, colWidths=widths, repeatRows=1, hAlign="LEFT")
        flow.setStyle(
            TableStyle(
                [
                    ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#404040")),
                    ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#D9D9D9")),
                    ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                    ("TOPPADDING", (0, 0), (-1, -1), 4),
                    ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
                    ("LEFTPADDING", (0, 0), (-1, -1), 4),
                    ("RIGHTPADDING", (0, 0), (-1, -1), 4),
                    (
                        "ROWBACKGROUNDS",
                        (0, 1),
                        (-1, -1),
                        [colors.white, colors.HexColor("#F2F2F2")],
                    ),
                ]
            )
        )
        return flow

    story = []
    first_heading = True
    index = 0
    while index < len(blocks):
        block = blocks[index]
        following = blocks[index + 1] if index + 1 < len(blocks) else None
        kind = block[0]
        if (
            kind == "h2"
            and following is not None
            and following[0] == "table"
        ):
            _kind, caption_text, headers, rows, aligns = following
            if len(rows) <= 14:
                story.append(
                    KeepTogether(
                        [
                            Paragraph(_pdf_text(block[1]), heading2),
                            Paragraph(_pdf_text(caption_text), table_caption_style),
                            make_table(headers, rows, aligns),
                        ]
                    )
                )
            else:
                story.append(Paragraph(_pdf_text(block[1]), heading2_plain))
                story.append(Paragraph(_pdf_text(caption_text), table_caption_plain))
                story.append(make_table(headers, rows, aligns))
            index += 2
            continue
        if kind == "h1":
            style = title_style if first_heading else heading1
            first_heading = False
            story.append(Paragraph(_pdf_text(block[1]), style))
        elif kind == "h2":
            story.append(Paragraph(_pdf_text(block[1]), heading2))
        elif kind == "p":
            story.append(Paragraph(_pdf_text(block[1]), body))
        elif kind == "code":
            for line in block[1].splitlines() or [""]:
                story.append(
                    Paragraph(_pdf_text(line).replace(" ", "&nbsp;") or "&nbsp;", code_style)
                )
        elif kind == "figure":
            _kind, caption_text, path = block
            image = PILImage.open(path)
            width = available
            height = width * image.height / image.width
            if height > 16.5 * cm:
                height = 16.5 * cm
                width = height * image.width / image.height
            story.append(
                KeepTogether(
                    [
                        Image(str(path), width=width, height=height),
                        Paragraph(_pdf_text(caption_text), figure_caption_style),
                    ]
                )
            )
        elif kind == "table":
            _kind, caption_text, headers, rows, aligns = block
            if not rows:
                index += 1
                continue
            caption_flow = Paragraph(_pdf_text(caption_text), table_caption_style)
            table_flow = make_table(headers, rows, aligns)
            if len(rows) <= 14:
                story.append(KeepTogether([caption_flow, table_flow]))
            else:
                story.append(Paragraph(_pdf_text(caption_text), table_caption_plain))
                story.append(table_flow)
        index += 1
    document.build(story)
    return output


# ---------------------------------------------------------------- 命令


def default_root() -> Path:
    return Path(__file__).resolve().parents[1]


def run_check(root: Path) -> int:
    problems = []
    try:
        scenarios = load_scenarios(root / "simulation" / "research" / "scenarios.json")
        print(f"A 场景：{len(scenarios.profiles)} 个配置，{len(scenarios.robots)} 个占位体")
    except MissingData as error:
        problems.append(str(error))
    try:
        board = load_board(root)
        print(
            f"B 板端：{board.directory.name}，{len(board.profiles)} 组测量，"
            f"配置 {'、'.join(profile.name for profile in board.profiles)}"
        )
        if board.missing:
            problems.append("板端配置尚未完成：" + ", ".join(board.missing))
    except MissingData as error:
        problems.append(str(error))
    try:
        paths = localization_paths(root)
        localization = normalize_localization(paths)
        print(f"C 定位：{len(paths)} 个文件，{len(localization.records)} 条记录")
        if localization.key_map:
            print("字段映射：" + json.dumps(localization.key_map, ensure_ascii=False))
    except MissingData as error:
        problems.append(str(error))
    if problems:
        print("缺少数据：")
        for problem in problems:
            print("  - " + problem)
        return 2
    print("数据齐备")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["check", "figures", "markdown", "pdf", "all"])
    parser.add_argument("--repo-root", type=Path, default=default_root())
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args()
    root = args.repo_root.resolve()
    out_dir = (args.out_dir or (root / "docs" / "research-20261023")).resolve()
    figure_dir = out_dir / "figures"
    if args.command == "check":
        return run_check(root)
    data = collect(root)
    if args.command in ("figures", "all"):
        for path in build_figures(data, figure_dir):
            print(f"图表：{path}")
    blocks = build_blocks(data, figure_dir)
    if args.command in ("markdown", "all"):
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "report.md").write_text(to_markdown(blocks), encoding="utf-8")
        print(f"Markdown：{out_dir / 'report.md'}")
    if args.command in ("pdf", "all"):
        out_dir.mkdir(parents=True, exist_ok=True)
        to_pdf(blocks, out_dir / "report.pdf")
        print(f"PDF：{out_dir / 'report.pdf'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
