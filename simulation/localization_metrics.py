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


class JsonlTail:
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
            # A malformed huge unterminated record cannot grow memory indefinitely.
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


def pose(row):
    try:
        vehicle = row["vehicle"]
        stamp = float(row["sim_time_us"])
        xy = (float(row["x_m"]), float(row["y_m"]))
        if (
            type(vehicle) is int
            and 0 <= vehicle < 4
            and stamp >= 0
            and all(math.isfinite(v) for v in (stamp, *xy))
        ):
            return vehicle, stamp, xy
    except (KeyError, ValueError, TypeError, OverflowError):
        pass
    return None


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


def home_positions(spawns):
    homes = {}
    if not isinstance(spawns, (list, tuple)):
        return homes
    for spawn in spawns:
        try:
            i = spawn["id"]
            point = tuple(float(spawn[k]) for k in ("x", "y", "z"))
            if type(i) is int and 0 <= i < 4 and all(math.isfinite(v) for v in point):
                homes[i] = point
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
    return homes


class LocalizationMetrics:
    tolerance_us = 100000
    stale_seconds = 3.0

    def __init__(
        self, folder, full=False, manager=None, homes=None, team_status_path=None
    ):
        self.folder = Path(folder)
        self.full = full
        self.manager = manager or {}
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
            JsonlTail(self.folder / f"companion_{i}.jsonl") for i in range(4)
        ]
        if self.full:
            for tail in self.tails:
                tail.full = True
        self.clear()

    def clear(self):
        self.truth = [deque(maxlen=2048) for _ in range(4)]
        self.estimates = [deque(maxlen=2048) for _ in range(4)]
        self.trails = {
            kind: [deque(maxlen=600) for _ in range(4)]
            for kind in ("truth", "estimate")
        }
        self.received = {kind: [None] * 4 for kind in ("truth", "estimate")}
        self.counts = [
            {"samples": 0, "valid": 0, "structural": 0, "peer": 0} for _ in range(4)
        ]
        self.fusion = [
            {"tof_accepted": 0, "tof_rejected": 0, "uwb_accepted": 0, "uwb_rejected": 0}
            for _ in range(4)
        ]
        self.distance = [0.0] * 4
        self.origin = [None] * 4
        self.excursion = [0.0] * 4
        self.matched = [None] * 4
        self.match_history = [deque(maxlen=2048) for _ in range(4)]
        self.error_stats = [{"n": 0, "sum_sq": 0.0, "maximum": 0.0} for _ in range(4)]
        self.last_stamp = {kind: [None] * 4 for kind in ("truth", "estimate")}
        self.ended = [False] * 4
        self.landing_events = [None] * 4
        self.landings = [None] * 4
        self.started = time.time()

    def set_homes(self, spawns):
        homes = home_positions(spawns)
        if homes != self.homes:
            self.homes = homes
            self.landings = [None] * 4

    def poll(self, now=None, include_trails=True):
        now = time.monotonic() if now is None else now
        batches = [tail.read() for tail in self.tails]
        if any(tail.reset for tail in self.tails):
            self.clear()
        self._consume(batches, now)
        self._match()
        return self.snapshot(now, include_trails=include_trails)

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
                    if sensor in ("tof", "uwb") and row.get("vehicle") == index - 1:
                        suffix = (
                            "accepted" if row.get("accepted") is True else "rejected"
                        )
                        self.fusion[index - 1][sensor + "_" + suffix] += 1
                    continue
                expected = "truth_evaluation_only" if index == 0 else "estimate"
                if kind != expected or (index and row.get("source") != "ESTIMATE"):
                    continue
                parsed = pose(row)
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
                trail.append(xy)

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

    def _match(self, final=False):
        for i in range(4):
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
        for i in range(4):
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
        if len(aligned_matches) == 4:
            stamps = [
                s
                for m in aligned_matches.values()
                for s in (m["stamp"], m["truth_stamp"])
            ]
            if max(stamps) - min(stamps) <= self.tolerance_us:
                delta = [
                    sum(m["delta"][axis] for m in aligned_matches.values()) / 4
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

    def _file_age(self, vehicle_index, now):
        path = self.tails[vehicle_index].path
        try:
            return max(0.0, time.time() - path.stat().st_mtime)
        except OSError:
            return None


def _windows_path(value):
    value = str(value)
    if os.name == "nt" and value.startswith("/mnt/") and len(value) > 6:
        return Path(value[5].upper() + ":" + value[6:])
    return Path(value)


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


def resolve_run_dir(root, run_dir=None, manager=None):
    if run_dir:
        return _windows_path(run_dir)
    manager_path = Path(manager or root / "simulation/runtime/manager.json")
    value = json.loads(manager_path.read_text(encoding="utf-8"))
    return _windows_path(value["run_dir"])


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


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Read-only report for an existing Gazebo/4PX4 run"
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
