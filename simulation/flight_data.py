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
ALIGNMENT_S = 0.1
SAMPLE_PERIOD_S = 0.05


def number(row, key):
    value = row.get(key)
    return (
        float(value)
        if isinstance(value, (int, float)) and math.isfinite(value)
        else math.nan
    )


def aligned_samples(estimates, truths):
    """Interpolate bracketed truth within 100 ms on either side; gaps stay NaN."""
    times = [row[0] for row in truths]
    for estimate in estimates:
        stamp = estimate[0]
        index = bisect_left(times, stamp)
        choices = [i for i in (index - 1, index) if 0 <= i < len(times)]
        nearest = min(choices, key=lambda i: abs(times[i] - stamp)) if choices else None
        truth = (
            truths[nearest]
            if nearest is not None and abs(times[nearest] - stamp) <= ALIGNMENT_S
            else None
        )
        if 0 < index < len(times):
            before, after = truths[index - 1], truths[index]
            if stamp - before[0] <= ALIGNMENT_S and after[0] - stamp <= ALIGNMENT_S:
                weight = (stamp - before[0]) / (after[0] - before[0])
                truth = (
                    stamp,
                    *(before[j] + weight * (after[j] - before[j]) for j in range(1, 4)),
                )
        error = (
            math.hypot(estimate[1] - truth[1], estimate[2] - truth[2])
            if truth is not None
            else math.nan
        )
        yield estimate, truth, error


class FlightHistory:
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

    @property
    def backlog(self):
        return any(tail.backlog for tail in self.tails.values())

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
