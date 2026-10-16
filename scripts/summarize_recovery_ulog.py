"""Summarize recovery commands, PX4 landing feedback and external odometry from ULog."""

import argparse
import hashlib
import json
import os
from pathlib import Path

import numpy as np
from pyulog import ULog


def rows(path):
    if not path.exists():
        return []
    result = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                result.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return result


def vertical_velocity_up(frame, velocity, quaternion=None, pose_frame=None):
    """将 ULog 外部速度转为向上分量；局部 NED/FRD 坐标的 Z 均向下。"""
    velocity = np.asarray(velocity, dtype=float)
    if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
        return None
    if frame in (1, 2):
        return -float(velocity[2])
    if frame != 3 or pose_frame not in (1, 2) or quaternion is None:
        return None
    quaternion = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(quaternion)
    if quaternion.shape != (4,) or not np.isfinite(norm) or norm < 1e-12:
        return None
    w, x, y, z = quaternion / norm
    down = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])
    return -float(down @ velocity)


def velocity_consistency(ev, local, begin_us, end_us, max_age_us=50000):
    """按外部量测采样时间匹配不晚于它的原生速度，不跨越陈旧数据。"""
    result = {'native_begin_s': begin_us / 1e6, 'native_end_s': end_us / 1e6,
              'max_native_age_ms': max_age_us / 1000, 'paired_samples': 0,
              'invalid_or_unsupported_samples': 0, 'missing_native_samples': 0,
              'invalid_native_samples': 0}
    if not ev or not local:
        result['available'] = False
        return result
    result['available'] = True
    differences = []
    for index in np.flatnonzero((ev['timestamp_sample'] >= begin_us) & (ev['timestamp_sample'] <= end_us)):
        external = vertical_velocity_up(int(ev['velocity_frame'][index]),
            [ev[f'velocity[{axis}]'][index] for axis in range(3)],
            [ev[f'q[{axis}]'][index] for axis in range(4)], int(ev['pose_frame'][index]))
        if external is None:
            result['invalid_or_unsupported_samples'] += 1
            continue
        stamp = int(ev['timestamp_sample'][index])
        previous = int(np.searchsorted(local['timestamp'], stamp, side='right')) - 1
        if previous < 0 or stamp - int(local['timestamp'][previous]) > max_age_us:
            result['missing_native_samples'] += 1
            continue
        if not np.isfinite(local['vz'][previous]) or ('v_z_valid' in local and not local['v_z_valid'][previous]):
            result['invalid_native_samples'] += 1
            continue
        differences.append(external + float(local['vz'][previous]))
    result['paired_samples'] = len(differences)
    if differences:
        absolute = np.abs(differences)
        result.update(absolute_difference_p50_mps=float(np.quantile(absolute, .5)),
                      absolute_difference_p95_mps=float(np.quantile(absolute, .95)),
                      absolute_difference_max_mps=float(np.max(absolute)),
                      samples_over_1_mps=int(np.count_nonzero(absolute > 1.0)))
    return result


def load_segments(paths):
    topics = ["sensor_combined", "vehicle_imu_status", "vehicle_land_detected",
              "vehicle_status", "vehicle_visual_odometry", "vehicle_local_position"]
    segments = sorted([(path, ULog(str(path), message_name_filter_list=topics)) for path in paths],
                      key=lambda segment: segment[1].start_timestamp)
    for previous, current in zip(segments, segments[1:]):
        if current[1].start_timestamp <= previous[1].last_timestamp:
            raise ValueError("ULog segments overlap or use different boot clocks")
    grouped = {}
    imu_gaps = []
    for path, ulog in segments:
        for topic in ulog.data_list:
            if topic.multi_id != 0:
                continue
            grouped.setdefault(topic.name, []).append(topic.data)
            if topic.name == "sensor_combined" and len(topic.data["timestamp"]) > 1:
                imu_gaps.append(np.diff(topic.data["timestamp"].astype(np.int64)) / 1000)
    data = {}
    for name, chunks in grouped.items():
        keys = set(chunks[0])
        if any(set(chunk) != keys for chunk in chunks):
            raise ValueError(f"ULog topic schema changed between segments: {name}")
        data[name] = {key: np.concatenate([chunk[key] for chunk in chunks]) for key in keys}
    metadata = [{"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                 "start_time_s": ulog.start_timestamp / 1e6, "end_time_s": ulog.last_timestamp / 1e6}
                for path, ulog in segments]
    return data, segments, metadata, np.concatenate(imu_gaps) if imu_gaps else np.array([])


def summarize(run, ulog_root):
    config = json.loads((run / "map.json").read_text(encoding="utf-8"))
    team = rows(run / "team_mission.jsonl")
    phases = []
    for row in team:
        if not phases or row["phase"] != phases[-1]["phase"]:
            phases.append({"time_s": row["sim_time_us"] / 1e6, "phase": row["phase"]})
    result = {"run": str(run), "phase_transitions": phases, "vehicles": []}
    recovery_phases = {"RETURN_ALL", "ABORT_RETURN", "LAND_ALL", "COMPLETE", "ABORTED"}
    recovery_us = next((row["sim_time_us"] for row in team if row["phase"] in recovery_phases), None)
    result["recovery_start_time_s"] = recovery_us / 1e6 if recovery_us is not None else None
    landing_us = next((row["sim_time_us"] for row in team if row["phase"] == "LAND_ALL"), None)
    for spawn in config["spawns"]:
        vehicle = spawn["id"]
        log = rows(run / f"companion_{vehicle}.jsonl")
        failures = [row for row in log if row.get("kind") == "unrecoverable"]
        failure_us = failures[0]["sim_time_us"] if failures else 0
        after = [row for row in log if row.get("sim_time_us", 0) >= failure_us]
        requests = [row for row in after if row.get("kind") == "command_request"]
        stops = [row for row in log if row.get("kind") == "stop"]
        controls = [row for row in after if row.get("kind") == "control_intention"]
        forbidden = [row for row in requests if row.get("command") in ("ARM", "OFFBOARD")] if failures else []
        paths = sorted((ulog_root / f"px4_{vehicle}" / "log").glob("**/*.ulg"))
        if not paths:
            raise ValueError(f"No ULog for vehicle {vehicle}")
        data, segments, metadata, imu_gaps = load_segments(paths)
        status = data.get("vehicle_status", {})
        land = data.get("vehicle_land_detected", {})
        first_armed_us = None
        if status:
            indices = np.flatnonzero(status["arming_state"] == 2)
            if len(indices):
                first_armed_us = int(status["timestamp"][indices[0]])
        first_airborne_us = None
        if land and first_armed_us is not None:
            indices = np.flatnonzero((land["timestamp"] >= first_armed_us) & (land["landed"] == 0))
            if len(indices):
                first_airborne_us = int(land["timestamp"][indices[0]])
        first_land = None
        if land and (failures or first_airborne_us is not None):
            indices = np.flatnonzero((land["timestamp"] >= (failure_us or first_airborne_us)) & (land["landed"] != 0))
            if len(indices):
                first_land = int(land["timestamp"][indices[0]]) / 1e6
        disarm = None
        if status and first_armed_us is not None:
            indices = np.flatnonzero((status["timestamp"] >= max(failure_us, first_armed_us)) & (status["arming_state"] == 1))
            if len(indices):
                disarm = int(status["timestamp"][indices[0]]) / 1e6
        row = {"vehicle": vehicle, "failure_time_s": failure_us / 1e6 if failures else None,
               "land_requests": sum(request.get("command") == "LAND" for request in requests),
               "disarm_requests": sum(request.get("command") == "DISARM" for request in requests),
               "auto_land_confirmed": any(control.get("auto_land_active") for control in controls),
               "arm_or_offboard_after_failure": forbidden,
               "arm_or_offboard_after_recovery": [row for row in log if row.get("kind") == "command_request"
                   and row.get("command") in ("ARM", "OFFBOARD") and recovery_us is not None
                   and row.get("sim_time_us", 0) >= recovery_us],
               "arm_or_offboard_after_landing": [row for row in log if row.get("kind") == "command_request"
                   and row.get("command") in ("ARM", "OFFBOARD") and landing_us is not None
                   and row.get("sim_time_us", 0) >= landing_us],
               "armed_at_stop": stops[-1]["armed_at_stop"] if stops else None,
               "native_landed": bool(land["landed"][-1]) if land else None,
               "native_arming_state": int(status["arming_state"][-1]) if status else None,
               "first_armed_time_s": first_armed_us / 1e6 if first_armed_us is not None else None,
               "first_airborne_time_s": first_airborne_us / 1e6 if first_airborne_us is not None else None,
               "landing_time_s": first_land, "disarm_time_s": disarm, "external_odometry": None,
               "ulog_path": metadata[-1]["path"], "ulog_sha256": metadata[-1]["sha256"],
               "ulog_segments": metadata,
               "ulog_dropouts": sum(len(ulog.dropouts) for path, ulog in segments),
               "stale_message_count": sum("STALE" in message.message for path, ulog in segments for message in ulog.logged_messages)}
        imu = data.get("sensor_combined", {})
        if imu and len(imu_gaps):
            row["combined_imu"] = {"samples": len(imu["timestamp"]),
                                    "gap_p99_ms": float(np.quantile(imu_gaps, .99)), "gap_max_ms": float(np.max(imu_gaps))}
        imu_status = data.get("vehicle_imu_status", {})
        if imu_status:
            row["sensor_rates_hz"] = {key: float(imu_status[key][-1]) for key in ("accel_rate_hz", "gyro_rate_hz")}
        ev = data.get("vehicle_visual_odometry", {})
        if ev:
            select = ev["timestamp"] >= failure_us
            position = np.stack([ev[f"position[{axis}]"][select] for axis in (0, 1)], axis=1)
            velocity = np.stack([ev[f"velocity[{axis}]"][select] for axis in (0, 1, 2)], axis=1)
            finite_position = np.all(np.isfinite(position), axis=1)
            row["external_odometry"] = {
                "window_start_s": failure_us / 1e6,
                "samples": int(np.count_nonzero(select)),
                "max_xy_norm_m": float(np.max(np.linalg.norm(position[finite_position], axis=1))) if np.any(finite_position) else None,
                "velocity_samples_disabled": int(np.count_nonzero(~np.all(np.isfinite(velocity), axis=1)))}
        alignment = next((item for item in log if item.get('kind') == 'imu_clock_alignment'), None)
        row['post_contact_velocity'] = []
        if alignment:
            offset = alignment['sim_time_us'] - alignment['imu_boot_time_us']
            for contact in (item for item in log if item.get('kind') == 'impact'):
                begin = int(contact['sim_time_us']) - offset
                consistency = velocity_consistency(ev, data.get('vehicle_local_position', {}), begin, begin + 4000000)
                consistency.update(contact_time_s=contact['sim_time_us'] / 1e6,
                                   observation_minus_native_us=offset)
                row['post_contact_velocity'].append(consistency)
        result["vehicles"].append(row)
    result["all_stopped"] = all(row["armed_at_stop"] is False and row["native_landed"] is True
                                and row["native_arming_state"] == 1 for row in result["vehicles"])
    result["terminal"] = phases[-1]["phase"] if phases else None
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--ulog-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-stopped", action="store_true")
    args = parser.parse_args()
    base = Path(r"\\wsl.localhost\Ubuntu-22.04\home\devcontainers\drone-sim\runtime") if os.name == "nt" else Path("/home/devcontainers/drone-sim/runtime")
    result = summarize(args.run, args.ulog_root or base / args.run.name)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"terminal": result["terminal"], "all_stopped": result["all_stopped"],
                      "vehicles": [{key: row[key] for key in ("vehicle", "failure_time_s", "landing_time_s", "disarm_time_s",
                          "armed_at_stop", "native_landed", "external_odometry")} for row in result["vehicles"]]}, ensure_ascii=False))
    if args.require_stopped and (not result["all_stopped"] or result["terminal"] not in ("COMPLETE", "ABORTED")
                                or any(row["arm_or_offboard_after_failure"] or row["arm_or_offboard_after_landing"]
                                       or row["arm_or_offboard_after_recovery"]
                                       for row in result["vehicles"])):
        raise SystemExit("Recovery lacks terminal, landing or disarm confirmation, or contains ARM/OFFBOARD retries")


if __name__ == "__main__":
    main()
