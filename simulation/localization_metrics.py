# @file localization_metrics.py
# @brief 对齐估计与真值日志，统计定位误差、新鲜度、机间距离误差和落点偏移。

"""Read-only localization evaluation. Never imported by flight/control code.

Statistics cover the observed tail since attaching, not the full mission.
Matching uses nearest simulation timestamps within 100 ms, without frame fitting.
"""

import argparse
import heapq
import json
import math
import os
import subprocess
import time
from collections import deque
from pathlib import Path


# 按字节预算增量读取 JSONL 文件，保存未完成行；文件替换或截短时通过 reset 通知上层重置统计。
class JsonlTail:
    # @param path JSONL 日志路径，文件尚不存在时也可构造读取器。
    # @param budget 单次最大读取字节数，同时限制未完成行的缓冲大小。
    def __init__(self, path, budget=262144):
        self.path = Path(path)
        self.budget = budget
        self.offset = None
        self.pending = b""
        self.identity = None
        self.skipped_bytes = 0
        self.reset = False
        self.backlog = False
        self.full = False

    # 每次最多读取 budget 字节，返回完整且可解析的 JSON 对象列表。
    # 首次默认从文件尾预算窗口接入，full=True 从头接入；半行跨调用保留，损坏记录跳过。
    def read(self):
        self.reset = False
        try:
            stat = self.path.stat()
            identity = (stat.st_dev, stat.st_ino)
            if (
                self.offset is None
                or identity != self.identity
                or stat.st_size < self.offset
            ):
                self.reset = self.offset is not None
                self.offset = 0 if self.full else max(0, stat.st_size - self.budget)
                self.skipped_bytes += self.offset
                self.pending = b""
                self.identity = identity
                discard = self.offset > 0
            else:
                discard = False
            with self.path.open("rb") as stream:
                stream.seek(self.offset)
                chunk = stream.read(self.budget)
                self.offset += len(chunk)
            self.backlog = self.offset < stat.st_size
            data = self.pending + chunk
            lines = data.split(b"\n")
            self.pending = lines.pop()
            if discard and lines:
                lines.pop(0)
            # 超长且无换行的损坏记录超过预算时丢弃，防止 pending 无限增长。
            if len(self.pending) > self.budget:
                self.pending = b""
            result = []
            for line in lines:
                try:
                    row = json.loads(line.decode("utf-8"))
                    if isinstance(row, dict):
                        result.append(row)
                except (ValueError, UnicodeError):
                    continue
            return result
        except OSError:
            self.backlog = False
            return []


# 校验车辆编号、非负微秒时间戳和有限 XY 米制坐标；成功返回三元组，非法记录返回 None。
def pose(row, vehicle_count=4):
    try:
        vehicle = row["vehicle"]
        stamp = float(row["sim_time_us"])
        xy = (float(row["x_m"]), float(row["y_m"]))
        if (
            type(vehicle) is int
            and 0 <= vehicle < vehicle_count
            and stamp >= 0
            and all(math.isfinite(v) for v in (stamp, *xy))
        ):
            return vehicle, stamp, xy
    except (KeyError, ValueError, TypeError, OverflowError):
        pass
    return None


# 优先返回有效的显式边界，否则从墙端点求边界；结果为 (xmin,xmax,ymin,ymax)，无法推导返回 None。
def map_bounds(config):
    try:
        bounds = config["bounds"]
        result = tuple(float(bounds[k]) for k in ("x_min", "x_max", "y_min", "y_max"))
        if (
            all(math.isfinite(v) for v in result)
            and result[0] < result[1]
            and result[2] < result[3]
        ):
            return result
    except (KeyError, TypeError, ValueError):
        pass
    walls = config.get("walls", [])
    try:
        xs = [float(w[k]) for w in walls for k in ("x1", "x2")]
        ys = [float(w[k]) for w in walls for k in ("y1", "y2")]
        if (
            xs
            and ys
            and all(math.isfinite(v) for v in xs + ys)
            and min(xs) < max(xs)
            and min(ys) < max(ys)
        ):
            return min(xs), max(xs), min(ys), max(ys)
    except (KeyError, TypeError, ValueError):
        pass
    return None


# @param path JSON 状态文件。
# @param budget 最多接受的字节数，超限、读取失败或顶层非对象均返回空字典。
def read_status(path, budget=65536):
    """Read one bounded JSON status snapshot."""
    try:
        with Path(path).open("rb") as stream:
            data = stream.read(budget + 1)
        if len(data) > budget:
            return {}
        value = json.loads(data.decode("utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError, UnicodeError):
        return {}


# 从出生点提取非负整数编号及有限 xyz 米制坐标；跳过无效条目，返回编号到坐标的映射。
def home_positions(spawns):
    homes = {}
    if not isinstance(spawns, (list, tuple)):
        return homes
    for spawn in spawns:
        try:
            i = spawn["id"]
            point = tuple(float(spawn[k]) for k in ("x", "y", "z"))
            if type(i) is int and i >= 0 and all(math.isfinite(v) for v in point):
                homes[i] = point
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
    return homes


# 维护每机定位统计和有界匹配历史；估计与真值按仿真微秒对齐，新鲜度使用本机时间。
# 误差直接在世界 XY 坐标相减；有限且可匹配的无效估计也计入误差，有效率另行统计。
class LocalizationMetrics:
    tolerance_us = 100000
    stale_seconds = 3.0

    # @param folder 真值和每机伴随日志所在的运行目录。
    # @param full True 从头接入日志，False 只接入有界尾部；完整报告由 read_full 归并生成。
    # @param manager 运行管理元数据，用于出生点、数量与进程状态。
    # @param homes 可选出生点列表，覆盖 manager 中的出生点。
    # @param team_status_path 本目录任务状态缺失时使用的后备路径。
    # @param vehicle_count 正整数机队数量；未指定时从出生点或 manager 推导。
    def __init__(
        self,
        folder,
        full=False,
        manager=None,
        homes=None,
        team_status_path=None,
        vehicle_count=None,
    ):
        self.folder = Path(folder)
        self.full = full
        self.manager = manager or {}
        configured_spawns = self.manager.get("map", {}).get("spawns", [])
        if vehicle_count is None and isinstance(homes, (list, tuple)) and homes:
            vehicle_count = len(homes)
        if (
            vehicle_count is None
            and isinstance(configured_spawns, (list, tuple))
            and configured_spawns
        ):
            vehicle_count = len(configured_spawns)
        if vehicle_count is None:
            vehicle_count = self.manager.get("vehicle_count") or 4
        if type(vehicle_count) is not int or vehicle_count < 1:
            raise ValueError("vehicle_count must be a positive integer")
        self.vehicle_count = vehicle_count
        self.homes = home_positions(
            homes
            if homes is not None
            else self.manager.get("map", {}).get("spawns", [])
        )
        self.team_status_path = (
            Path(team_status_path)
            if team_status_path
            else self.folder / "team_status.json"
        )
        self.tails = [JsonlTail(self.folder / "truth_evaluation_only.jsonl")] + [
            JsonlTail(self.folder / f"companion_{i}.jsonl")
            for i in range(self.vehicle_count)
        ]
        if self.full:
            for tail in self.tails:
                tail.full = True
        self.clear()

    # 清空匹配队列、轨迹、累计误差和落点缓存；文件读取游标由 JsonlTail 单独管理。
    def clear(self):
        self.truth = [deque(maxlen=2048) for _ in range(self.vehicle_count)]
        self.estimates = [deque(maxlen=2048) for _ in range(self.vehicle_count)]
        self.trails = {
            kind: [deque(maxlen=600) for _ in range(self.vehicle_count)]
            for kind in ("truth", "estimate")
        }
        self.received = {
            kind: [None] * self.vehicle_count for kind in ("truth", "estimate")
        }
        self.counts = [
            {
                "samples": 0,
                "valid": 0,
                "structural": 0,
                "peer": 0,
                "flow_valid": 0,
                "position_confidence": 0,
                "position_confidence_count": 0,
                "latest": {},
                "localization_mode": None,
            }
            for _ in range(self.vehicle_count)
        ]
        self.fusion = [
            {
                kind + suffix: 0
                for kind in ("tof", "uwb", "optical_flow", "uwb_anchor")
                for suffix in ("_accepted", "_rejected")
            }
            for _ in range(self.vehicle_count)
        ]
        self.distance = [0.0] * self.vehicle_count
        self.origin = [None] * self.vehicle_count
        self.excursion = [0.0] * self.vehicle_count
        self.matched = [None] * self.vehicle_count
        self.match_history = [deque(maxlen=2048) for _ in range(self.vehicle_count)]
        self.error_stats = [
            {"n": 0, "sum_sq": 0.0, "maximum": 0.0} for _ in range(self.vehicle_count)
        ]
        self.last_stamp = {
            kind: [None] * self.vehicle_count for kind in ("truth", "estimate")
        }
        self.ended = [False] * self.vehicle_count
        self.landing_events = [None] * self.vehicle_count
        self.landings = [None] * self.vehicle_count
        self.started = time.time()

    # 更新出生点参考；坐标变化时清除已缓存落点，避免沿用旧参考下的误差。
    def set_homes(self, spawns):
        homes = home_positions(spawns)
        if homes != self.homes:
            self.homes = homes
            self.landings = [None] * self.vehicle_count

    # 读取各文件的一批增量并匹配，返回快照；任一文件发生重置时清空统计。
    # @param now 本机 monotonic 秒数，可由调用方注入；不能传入仿真时间。
    def poll(self, now=None, include_trails=True):
        now = time.monotonic() if now is None else now
        batches = [tail.read() for tail in self.tails]
        if any(tail.reset for tail in self.tails):
            self.clear()
        self._consume(batches, now)
        self._match()
        return self.snapshot(now, include_trails=include_trails)

    # 校验文件对应的车辆与记录来源，按严格递增仿真时间入队并累计质量、融合和轨迹统计。
    def _consume(self, batches, now):
        for index, rows in enumerate(batches):
            for row in rows:
                kind = row.get("kind")
                if (
                    index
                    and kind == "contact_final"
                    and row.get("source") == "ESTIMATE"
                    and row.get("vehicle") == index - 1
                ):
                    if row.get("landed") is True or row.get("phase") == "LANDED":
                        try:
                            stamp = float(row["sim_time_us"])
                            if math.isfinite(stamp) and stamp >= 0:
                                self.landing_events[index - 1] = stamp
                        except (KeyError, TypeError, ValueError, OverflowError):
                            pass
                    continue
                if (
                    index
                    and kind == "stop"
                    and row.get("vehicle") == index - 1
                    and row.get("source") == "ESTIMATE"
                ):
                    self.ended[index - 1] = True
                    continue
                if index and kind == "fusion" and row.get("source") == "ESTIMATE":
                    sensor = row.get("sensor_kind")
                    if (
                        sensor in ("tof", "uwb", "optical_flow", "uwb_anchor")
                        and row.get("vehicle") == index - 1
                    ):
                        suffix = (
                            "accepted" if row.get("accepted") is True else "rejected"
                        )
                        self.fusion[index - 1][sensor + "_" + suffix] += 1
                    continue
                expected = "truth_evaluation_only" if index == 0 else "estimate"
                if kind != expected or (index and row.get("source") != "ESTIMATE"):
                    continue
                parsed = pose(row, self.vehicle_count)
                if parsed is None:
                    continue
                vehicle, stamp, xy = parsed
                if index and vehicle != index - 1:
                    continue
                name = "truth" if index == 0 else "estimate"
                history = self.truth[vehicle] if index == 0 else self.estimates[vehicle]
                previous = self.last_stamp[name][vehicle]
                if previous is not None and stamp <= previous:
                    continue
                self.last_stamp[name][vehicle] = stamp
                history.append((stamp, xy, row))
                if self.full and index == 0:
                    while (
                        len(history) > 2
                        and stamp - history[1][0] > 2 * self.tolerance_us
                    ):
                        history.popleft()
                self.received[name][vehicle] = now
                trail = self.trails[name][vehicle]
                if index == 0:
                    if trail:
                        self.distance[vehicle] += math.dist(trail[-1], xy)
                    if self.origin[vehicle] is None:
                        self.origin[vehicle] = xy
                    self.excursion[vehicle] = max(
                        self.excursion[vehicle], math.dist(self.origin[vehicle], xy)
                    )
                else:
                    counts = self.counts[vehicle]
                    counts["samples"] += 1
                    for key, field in (
                        ("valid", "xy_valid"),
                        ("structural", "xy_structural_valid"),
                        ("peer", "xy_peer_valid"),
                    ):
                        counts[key] += row.get(field) is True
                    counts["flow_valid"] += row.get("flow_valid") is True
                    if isinstance(
                        row.get("position_confidence"), (int, float)
                    ) and math.isfinite(row["position_confidence"]):
                        counts["position_confidence"] += float(
                            row["position_confidence"]
                        )
                        counts["position_confidence_count"] += 1
                    uncertainty = row.get("xy_uncertainty_m")
                    if not (isinstance(uncertainty, (int, float)) and math.isfinite(uncertainty)):
                        covariance = row.get("pose_covariance")
                        if isinstance(covariance, list) and len(covariance) == 9:
                            try:
                                uncertainty = math.sqrt(max(0.0, float(covariance[0]) + float(covariance[4])))
                            except (TypeError, ValueError, OverflowError):
                                uncertainty = None
                    counts["latest"] = {
                        "xy_valid": row.get("xy_valid") if isinstance(row.get("xy_valid"), bool) else None,
                        "flow_valid": row.get("flow_valid") if isinstance(row.get("flow_valid"), bool) else None,
                        "position_confidence": row.get("position_confidence") if isinstance(row.get("position_confidence"), (int, float)) and math.isfinite(row["position_confidence"]) else None,
                        "xy_uncertainty_m": uncertainty if isinstance(uncertainty, (int, float)) and math.isfinite(uncertainty) else None,
                        "localization_mode": row.get("localization_mode") if isinstance(row.get("localization_mode"), str) else None,
                    }
                    if isinstance(row.get("localization_mode"), str):
                        counts["localization_mode"] = row["localization_mode"]
                trail.append(xy)

    # 按仿真时间归并真值与所有车辆日志；每个输入流以开始迭代时取得的大小为读取终点。
    # @return 完整读取范围的统计，轨迹和匹配历史仍有容量上限。
    def read_full(self, include_trails=False):
        """Merge five finite log snapshots in timestamp order with bounded memory."""
        self.full = True
        self.clear()
        streams = [_stream_rows(t.path, index) for index, t in enumerate(self.tails)]
        for stamp, index, sequence, row in heapq.merge(
            *streams, key=lambda item: item[:3]
        ):
            batches = [[] for _ in self.tails]
            batches[index] = [row]
            self._consume(batches, time.monotonic())
            self._match()
        self._match(final=True)
        return self.snapshot(include_trails=include_trails)

    # 对每条待匹配估计选择容差 100 ms 内最近真值，不插值、不拟合坐标系。
    # 真值与估计分别按日志到达顺序入队，只有真值时间覆盖估计后才消费，避免过早匹配旧帧。
    # @param final True 时处理末尾尚未被真值时间覆盖的估计；正常轮询保留它们等待评估日志追上。
    def _match(self, final=False):
        for i in range(self.vehicle_count):
            truths = self.truth[i]
            if not truths:
                continue
            while self.estimates[i]:
                estimate = self.estimates[i][0]
                stamp = estimate[0]
                if stamp > truths[-1][0] and not final:
                    break  # Wait for the evaluator, do not prematurely choose an older pose.
                self.estimates[i].popleft()
                truth = min(truths, key=lambda item: abs(item[0] - stamp))
                if abs(truth[0] - stamp) > self.tolerance_us:
                    continue
                delta = tuple(a - b for a, b in zip(estimate[1], truth[1]))
                error = math.hypot(*delta)
                self.matched[i] = {
                    "stamp": stamp,
                    "truth_stamp": truth[0],
                    "estimate": estimate[1],
                    "truth": truth[1],
                    "truth_z_m": truth[2].get("z_m"),
                    "truth_landed": truth[2].get("landed") is True,
                    "delta": delta,
                    "error": error,
                    "valid": estimate[2].get("xy_valid") is True,
                }
                self.match_history[i].append(self.matched[i])
                stats = self.error_stats[i]
                stats["n"] += 1
                stats["sum_sq"] += error * error
                stats["maximum"] = max(stats["maximum"], error)

    # 计算当前状态及报告；机间距离误差和共同平移要求参与记录的时间跨度不超过匹配容差。
    # @param include_trails 是否附上有界 XY 轨迹；返回字典中的误差为米、age 为秒。
    def snapshot(self, now=None, include_trails=True):
        now = time.monotonic() if now is None else now
        team = read_status(self.folder / "team_status.json")
        if not team:
            team = read_status(self.team_status_path)
        if (
            team.get("run_dir")
            and _windows_path(team["run_dir"]).resolve() != self.folder.resolve()
        ):
            team = {}
        vehicles = []
        for i in range(self.vehicle_count):
            ages = {
                k: None
                if self.received[k][i] is None
                else max(
                    self._file_age(i + 1 if k == "estimate" else 0, now) or 0,
                    now - self.received[k][i],
                )
                for k in self.received
            }
            match = self.matched[i]
            status = (
                "stopped"
                if self.manager.get("stopped") or self.ended[i]
                else "pending"
                if any(v is None for v in ages.values())
                else "stale"
                if max(ages.values()) > self.stale_seconds
                else "unmatched"
                if match is None
                else "live"
            )
            process_states = self.manager.get("pid_status", {})
            if status != "stopped" and any(
                process_states.get(name) is False
                for name in ("gazebo", "evaluator", f"companion_{i}")
            ):
                status = "stale"
            # New unmatched estimates must not leave old matched errors labelled live.
            if status == "live" and self.trails["estimate"][i]:
                latest_truth = self.truth[i][-1][0]
                if (
                    latest_truth - match["truth_stamp"] > self.tolerance_us
                    or self.last_stamp["estimate"][i] - match["stamp"]
                    > self.tolerance_us
                ):
                    status = "unmatched"
            counts = self.counts[i]
            n = counts["samples"]
            stats = self.error_stats[i]
            vehicle = {
                "vehicle": i,
                "status": status,
                "ages_s": ages,
                "match": match,
                "samples": n,
                "validity": {
                    k: counts[k] / n if n else None
                    for k in ("valid", "structural", "peer")
                },
                "flow_validity": counts["flow_valid"] / n if n else None,
                "position_confidence": (
                    counts["position_confidence"] / counts["position_confidence_count"]
                    if counts["position_confidence_count"]
                    else None
                ),
                "localization_mode": counts["localization_mode"],
                "latest": dict(counts["latest"]),
                "fusion": dict(self.fusion[i]),
                "matched_samples": stats["n"],
                "rmse_m": math.sqrt(stats["sum_sq"] / stats["n"])
                if stats["n"]
                else None,
                "max_error_m": stats["maximum"] if stats["n"] else None,
                "truth_path_m": self.distance[i],
                "truth_excursion_m": self.excursion[i],
                "trajectory": "traveled"
                if self.excursion[i] >= 0.5
                else "stationary_or_short",
            }
            vehicle["home_error"] = self._home_error(i, team)
            vehicles.append(vehicle)
            vehicle["unmatched_samples"] = n - stats["n"]
        aligned_matches = {i: m for i, m in enumerate(self.matched) if m is not None}
        pairs = []
        for i, a in aligned_matches.items():
            for j, b in aligned_matches.items():
                if (
                    j > i
                    and max(a["stamp"], b["stamp"], a["truth_stamp"], b["truth_stamp"])
                    - min(a["stamp"], b["stamp"], a["truth_stamp"], b["truth_stamp"])
                    <= self.tolerance_us
                ):
                    pairs.append(
                        {
                            "vehicles": [i, j],
                            "distance_error_m": abs(
                                math.dist(a["estimate"], b["estimate"])
                                - math.dist(a["truth"], b["truth"])
                            ),
                            "valid": bool(a["valid"] and b["valid"]),
                            "estimate_validity": [a["valid"], b["valid"]],
                            "sim_time_us": [a["stamp"], b["stamp"]],
                            "status": [vehicles[i]["status"], vehicles[j]["status"]],
                        }
                    )
        common = None
        if len(aligned_matches) == self.vehicle_count:
            stamps = [
                s
                for m in aligned_matches.values()
                for s in (m["stamp"], m["truth_stamp"])
            ]
            if max(stamps) - min(stamps) <= self.tolerance_us:
                delta = [
                    sum(m["delta"][axis] for m in aligned_matches.values())
                    / self.vehicle_count
                    for axis in (0, 1)
                ]
                common = {
                    "x_m": delta[0],
                    "y_m": delta[1],
                    "magnitude_m": math.hypot(*delta),
                    "valid": all(m["valid"] for m in aligned_matches.values()),
                    "estimate_validity": [m["valid"] for m in aligned_matches.values()],
                    "sim_time_us": [m["stamp"] for m in aligned_matches.values()],
                    "status": [v["status"] for v in vehicles],
                }
        report = {
            "scope": "full streamed log (file sizes captured on opening)"
            if self.full
            else "observed bounded tail since attachment",
            "folder": str(self.folder),
            "started_unix": self.started,
            "alignment_tolerance_us": self.tolerance_us,
            "vehicles": vehicles,
            "team_status": team,
            "homes": [
                {"id": i, "x": p[0], "y": p[1], "z": p[2]}
                for i, p in sorted(self.homes.items())
            ],
            "home_error_reference": "configured_spawn_world_xy",
            "home_marker": self.manager.get("map", {}).get("home_marker", {}),
            "pairs": pairs,
            "common_translation": common,
            "skipped_bytes": sum(t.skipped_bytes for t in self.tails),
            "backlog": any(t.backlog for t in self.tails),
            "mission_success": "not assessed",
            "manager_stopped": bool(self.manager.get("stopped")),
            "pid_status": self.manager.get("pid_status", {}),
            "error_population": "all aligned finite estimates, including invalid; no frame fitting",
        }
        if include_trails:
            report["trails"] = {
                kind: [list(trail) for trail in self.trails[kind]]
                for kind in ("truth", "estimate")
            }
        return report

    # 计算相对出生点的实时 XY 偏移，并在落地事件附近寻找容差内真值与估计，缓存落点结果。
    def _home_error(self, i, team):
        home = self.homes.get(i)
        if home is None:
            return None

        def offset(point):
            dx, dy = point[0] - home[0], point[1] - home[1]
            return {"dx_m": dx, "dy_m": dy, "xy_m": math.hypot(dx, dy)}

        truth = self.truth[i][-1] if self.truth[i] else None
        estimate = self.trails["estimate"][i]
        result = {
            "home": home,
            "label": "realtime_home_offset",
            "estimate": offset(estimate[-1]) if estimate else None,
            "estimate_stamp_us": self.last_stamp["estimate"][i],
            "truth": offset(truth[1]) if truth else None,
            "truth_stamp_us": truth[0] if truth else None,
            "truth_z_m": truth[2].get("z_m") if truth else None,
            "landing": self.landings[i],
        }
        match = self.matched[i]
        complete = team.get("phase") == "COMPLETE"
        event = self.landing_events[i]
        if (
            event is None
            and truth
            and truth[2].get("landed") is True
            and team.get("mission_phase", team.get("phase"))
            in {"LAND_ALL", "COMPLETE", "ABORTED"}
        ):
            event = truth[0]
        if self.landings[i] is None and event is not None and self.truth[i]:
            landed_truth = min(self.truth[i], key=lambda row: abs(row[0] - event))
            if abs(landed_truth[0] - event) <= self.tolerance_us:
                landing_match = min(
                    self.match_history[i],
                    key=lambda row: abs(row["stamp"] - event),
                    default=None,
                )
                self.landings[i] = {
                    "sim_time_us": event,
                    "truth_stamp_us": landed_truth[0],
                    "truth": offset(landed_truth[1]),
                    "truth_z_m": landed_truth[2].get("z_m"),
                    "estimate": offset(landing_match["estimate"])
                    if landing_match
                    and abs(landing_match["stamp"] - event) <= self.tolerance_us
                    else None,
                }
        if self.landings[i] and self.landings[i]["estimate"] is None:
            landing = self.landings[i]
            landing_match = min(
                self.match_history[i],
                key=lambda row: abs(row["stamp"] - landing["sim_time_us"]),
                default=None,
            )
            if (
                landing_match
                and abs(landing_match["stamp"] - landing["sim_time_us"])
                <= self.tolerance_us
            ):
                landing["estimate"] = offset(landing_match["estimate"])
        result["landing"] = self.landings[i]
        if result["landing"] is not None:
            result["label"] = "final_landing_error"
        elif (
            complete
            and match
            and truth
            and abs(match["truth_stamp"] - truth[0]) <= self.tolerance_us
            and abs(match["stamp"] - self.last_stamp["estimate"][i])
            <= self.tolerance_us
        ):
            result["label"] = "final_matched_home_error"
            result["final"] = {
                "sim_time_us": match["stamp"],
                "truth_stamp_us": match["truth_stamp"],
                "estimate": offset(match["estimate"]),
                "truth": offset(match["truth"]),
            }
        return result

    # 返回日志修改距当前墙钟的秒数；文件不存在时返回 None，未来时间截为零。
    def _file_age(self, vehicle_index, now):
        path = self.tails[vehicle_index].path
        try:
            return max(0.0, time.time() - path.stat().st_mtime)
        except OSError:
            return None


# 在 Windows 将 /mnt/<盘符>/ 路径转换为盘符路径，其他输入直接构造 Path。
def _windows_path(value):
    value = str(value)
    if os.name == "nt" and value.startswith("/mnt/") and len(value) > 6:
        return Path(value[5].upper() + ":" + value[6:])
    return Path(value)


# 以 256 KiB 分块产生（微秒时间、流编号、序号、记录），读取范围固定到开始时的文件大小。
# 归并排序键对倒退时间取前值，原始记录时间保持不变，随后仍由消费逻辑校验。
def _stream_rows(path, index):
    """At most one 256 KiB chunk and one record ahead per input file."""
    tail = JsonlTail(path)
    tail.full = True
    try:
        limit = path.stat().st_size
    except OSError:
        return
    sequence, previous = 0, 0.0
    while tail.offset is None or tail.offset < limit:
        tail.budget = min(262144, limit - (tail.offset or 0))
        rows = tail.read()
        for row in rows:
            try:
                stamp = float(row.get("sim_time_us", previous))
                if not math.isfinite(stamp):
                    continue
            except (TypeError, ValueError):
                continue
            previous = max(previous, stamp)
            yield previous, index, sequence, row
            sequence += 1
        if not tail.backlog or tail.offset >= limit:
            break


# 读取指定运行对应的管理状态并检查 Linux 进程身份；Windows 通过 WSL 执行一次 /proc 查询。
# 不属于当前目录的状态返回空字典，进程检查不可用时保留元数据并标记检查不可用。
def manager_context(root, folder, manager_path=None):
    """Only use manager metadata for the requested run; Linux PIDs are never Windows PIDs."""
    try:
        value = json.loads(
            Path(manager_path or root / "simulation/runtime/manager.json").read_text(
                encoding="utf-8"
            )
        )
        if _windows_path(value["run_dir"]).resolve() != folder.resolve():
            return {}
    except (OSError, ValueError, KeyError, TypeError):
        return {}
    processes = value.get("processes", [])
    if not processes or value.get("stopped"):
        return value
    # One read-only check, including /proc start ticks, to avoid PID reuse false positives.
    code = """import json, pathlib, sys
result = {}
for item in json.loads(sys.argv[1]):
    try:
        fields = pathlib.Path('/proc/' + str(int(item['pid'])) + '/stat').read_text(encoding='utf-8').rsplit(')', 1)[1].split()
        result[item['name']] = fields[0] != 'Z' and str(item.get('start_ticks', fields[19])) == fields[19]
    except (OSError, KeyError, ValueError):
        result[item.get('name', 'unknown')] = False
print(json.dumps(result))
"""
    args = ["python3", "-c", code, json.dumps(processes)]
    if os.name == "nt":
        args = ["wsl.exe", "-d", "Ubuntu-22.04", "--exec", *args]
    try:
        completed = subprocess.run(
            args,
            capture_output=True,
            timeout=5,
            check=False,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        if completed.returncode == 0:
            value["pid_status"] = json.loads(completed.stdout.decode("utf-8"))
        else:
            value["pid_check"] = "unavailable"
    except (OSError, subprocess.TimeoutExpired, ValueError):
        value["pid_check"] = "unavailable"
    return value


# 优先解析显式运行目录，否则从 manager.json 读取 run_dir；文件或格式错误向调用方传播。
def resolve_run_dir(root, run_dir=None, manager=None):
    if run_dir:
        return _windows_path(run_dir)
    manager_path = Path(manager or root / "simulation/runtime/manager.json")
    value = json.loads(manager_path.read_text(encoding="utf-8"))
    return _windows_path(value["run_dir"])


# 把统计快照格式化为命令行文本，缺少样本时显示占位符，保留错误指标和有效率的区别。
def report_text(report):
    lines = [
        f"run: {report['folder']}",
        f"scope: {report['scope']}",
    ]
    for vehicle in report["vehicles"]:
        validity = vehicle["validity"]
        ratio = lambda key, validity=validity: (
            "--" if validity[key] is None else f"{validity[key] * 100:.0f}%"
        )
        error = (
            "--"
            if vehicle["rmse_m"] is None
            else f"RMSE={vehicle['rmse_m']:.3f}m max={vehicle['max_error_m']:.3f}m"
        )
        lines.append(
            f"vehicle {vehicle['vehicle']}: {vehicle['status']}, {error}, valid={ratio('valid')}, structural={ratio('structural')}, peer={ratio('peer')}, trajectory={vehicle['trajectory']}"
        )
    pairs = (
        ", ".join(
            f"{p['vehicles'][0]}-{p['vehicles'][1]}={p['distance_error_m']:.3f}m valid={p['valid']}"
            for p in report["pairs"]
        )
        or "--"
    )
    drift = report["common_translation"]
    lines.append(f"pairwise_distance_error: {pairs}")
    lines.append(
        "common_translation_drift: "
        + (
            "--"
            if drift is None
            else f"{drift['magnitude_m']:.3f}m valid={drift['valid']}"
        )
    )
    if report["backlog"]:
        lines.append("log_tail: pending backlog")
    return "\n".join(lines)


# 解析已有运行目录和输出选项，选择有界尾部轮询或全量流式统计，输出文本或 JSON。
def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Read-only report for an existing Gazebo/PX4 run"
    )
    parser.add_argument(
        "--run-dir", help="Existing run directory; defaults to manager.json"
    )
    parser.add_argument("--manager", help="Manager JSON path")
    parser.add_argument(
        "--json", action="store_true", dest="as_json", help="Print JSON report"
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="Stream from log starts; GUI remains bounded",
    )
    parser.add_argument(
        "--include-trails",
        action="store_true",
        help="Include bounded visual trails in JSON",
    )
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[1]
    folder = resolve_run_dir(root, args.run_dir, args.manager)
    metrics = LocalizationMetrics(
        folder, full=args.full, manager=manager_context(root, folder, args.manager)
    )
    report = (
        metrics.read_full(include_trails=args.include_trails)
        if args.full
        else metrics.poll(include_trails=args.include_trails)
    )
    print(
        json.dumps(report, ensure_ascii=False, indent=2)
        if args.as_json
        else report_text(report)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
