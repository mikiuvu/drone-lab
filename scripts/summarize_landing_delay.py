"""Compare PX4 landing gates with independent physical motion after a recovery failure."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

def latest(data, time_s):
    index = int(np.searchsorted(data["timestamp"], round(time_s * 1e6), side="right")) - 1
    if index < 0:
        raise ValueError(f"No native sample at or before {time_s:.3f} s")
    return index


def blocked_seconds(data, key, start_s, end_s, blocked_value):
    if end_s <= start_s:
        raise ValueError("Recovery window must have positive duration")
    times = data["timestamp"].astype(np.float64) / 1e6
    inner = times[(times > start_s) & (times < end_s)]
    boundaries = np.concatenate(([start_s], inner, [end_s]))
    indices = np.searchsorted(times, boundaries[:-1], side="right") - 1
    if np.any(indices < 0):
        raise ValueError("Landing detector does not cover the recovery window")
    blocked = data[key][indices] == blocked_value
    return float(np.sum(np.diff(boundaries)[blocked]))


def summarize(run, vehicle, paths):
    from summarize_recovery_ulog import load_segments, rows

    companion = rows(run / f"companion_{vehicle}.jsonl")
    failure = next((row for row in companion if row.get("kind") == "unrecoverable"), None)
    if failure is None:
        raise ValueError("Vehicle has no unrecoverable event")
    start_s = failure["sim_time_us"] / 1e6
    data, _, metadata, _ = load_segments(paths)
    land = data["vehicle_land_detected"]
    local = data["vehicle_local_position"]
    ev = data["vehicle_visual_odometry"]
    land_indices = np.flatnonzero((land["timestamp"] >= failure["sim_time_us"]) & (land["landed"] != 0))
    if not len(land_indices):
        raise ValueError("Vehicle has no native landing confirmation after failure")
    end_s = int(land["timestamp"][land_indices[0]]) / 1e6
    land_time = land["timestamp"].astype(np.float64) / 1e6
    gate_gaps = np.diff(land_time)[(land_time[1:] >= start_s) & (land_time[:-1] <= end_s)]

    # 位置一致的发送/接收记录约束时钟差；接收时间差同时包含链路延迟。
    ev_time = ev["timestamp"].astype(np.float64) / 1e6
    ev_position = np.stack([ev[f"position[{axis}]"] for axis in range(3)], axis=1)
    estimates = [row for row in companion if row.get("kind") == "estimate" and row.get("xy_valid")
                 and start_s - 12 <= row["sim_time_us"] / 1e6 < start_s]
    anchors = []
    previous_s = -1.0
    for row in estimates:
        stamp_s = row["sim_time_us"] / 1e6
        if stamp_s - previous_s < 0.8:
            continue
        candidates = np.flatnonzero(np.abs(ev_time - stamp_s) < 0.25)
        if not len(candidates):
            continue
        expected = np.array([row["y_m"], row["x_m"], -row["z_m"]])
        errors = np.linalg.norm(ev_position[candidates] - expected, axis=1)
        best = int(np.argmin(errors))
        if errors[best] > 1e-5:
            continue
        index = candidates[best]
        anchors.append({"companion_time_s": stamp_s, "native_receive_time_s": float(ev_time[index]),
                        "position_error_m": float(errors[best]), "receive_offset_ms": float((ev_time[index] - stamp_s) * 1000)})
        previous_s = stamp_s
    if len(anchors) < 3:
        raise ValueError("Fewer than three matching odometry records; physical/native timing is unverified")

    truth_path = run / "truth_evaluation_only.jsonl"
    truth = [row for row in rows(truth_path) if row.get("vehicle") == vehicle]
    truth_time = np.array([row["sim_time_us"] / 1e6 for row in truth])
    if len(truth_time) < 2 or np.any(np.diff(truth_time) <= 0):
        raise ValueError("Physical trace must contain increasing timestamps")
    position = np.array([[row[key] for key in ("x_m", "y_m", "z_m")] for row in truth])
    requested = sorted(set([start_s, end_s] + list(np.arange(np.ceil(start_s), end_s, 5))))
    samples = []
    for time_s in requested:
        ti = int(np.searchsorted(truth_time, time_s, side="right")) - 1
        if ti < 0:
            raise ValueError("Physical trace starts after the sample window")
        before = int(np.searchsorted(truth_time, truth_time[ti] - 0.2))
        if ti == before or time_s - truth_time[ti] > 0.1 or np.any(np.diff(truth_time[before:ti + 1]) > 0.1):
            raise ValueError("Physical trace does not cover the sample window")
        velocity = (position[ti] - position[before]) / (truth_time[ti] - truth_time[before])
        li, pi = latest(land, time_s), latest(local, time_s)
        samples.append({"time_s": float(time_s), "physical_sample_time_s": float(truth_time[ti]),
            "physical_position_m": position[ti].tolist(), "physical_velocity_window_s": float(truth_time[ti] - truth_time[before]),
            "physical_xy_speed_mps": float(np.linalg.norm(velocity[:2])), "physical_vz_mps": float(velocity[2]),
            "physical_ground_contact": truth[ti]["landed"],
            "native_position_time_s": int(local["timestamp"][pi]) / 1e6,
            "native_xy_speed_mps": float(np.hypot(local["vx"][pi], local["vy"][pi])),
            "native_vz_mps": float(local["vz"][pi]), "native_eph_m": float(local["eph"][pi]),
            "native_land_time_s": int(land["timestamp"][li]) / 1e6,
            "native_gates": {key: bool(land[key][li]) for key in ("landed", "horizontal_movement", "vertical_movement",
                "rotational_movement", "has_low_throttle", "in_descend", "close_to_ground_or_skipped_check")}})
    select = (ev["timestamp"] >= failure["sim_time_us"]) & (ev["timestamp"] <= end_s * 1e6)
    velocity = np.stack([ev[f"velocity[{axis}]"][select] for axis in range(3)], axis=1)
    return {"run": str(run), "vehicle": vehicle, "failure_time_s": start_s, "native_landing_time_s": end_s,
        "failure_to_landing_s": end_s - start_s,
        "native_gate_log_gap_max_s": float(np.max(gate_gaps)) if len(gate_gaps) else None,
        "clock_matching": {"anchors": anchors, "physical_time_shift_s": 0,
            "receive_offset_median_ms": float(np.median([row["receive_offset_ms"] for row in anchors])),
            "receive_offset_range_ms": [min(row["receive_offset_ms"] for row in anchors), max(row["receive_offset_ms"] for row in anchors)]},
        "blocked_duration_s": {label: blocked_seconds(land, key, start_s, end_s, value) for label, key, value in
            (("horizontal_movement", "horizontal_movement", 1), ("vertical_movement", "vertical_movement", 1),
             ("rotational_movement", "rotational_movement", 1), ("low_throttle_missing", "has_low_throttle", 0),
             ("descent_not_requested", "in_descend", 0), ("ground_proximity_missing", "close_to_ground_or_skipped_check", 0))},
        "external_velocity": {"samples": int(np.count_nonzero(select)),
            "disabled_samples": int(np.count_nonzero(~np.all(np.isfinite(velocity), axis=1)))},
        "samples": samples, "ulog_segments": metadata,
        "physical_trace_sha256": hashlib.sha256(truth_path.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--vehicle", type=int, required=True)
    parser.add_argument("--ulog", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.run, args.vehicle, args.ulog)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: result[key] for key in ("vehicle", "failure_to_landing_s", "blocked_duration_s", "external_velocity")}))


if __name__ == "__main__":
    main()
