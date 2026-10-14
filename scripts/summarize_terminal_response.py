"""Compare terminal pose estimates, velocity references and evaluator trajectories."""

import argparse
import bisect
import json
import math
from pathlib import Path

from summarize_contact_guidance import interpolate, rows


def series(records, field):
    selected = [row for row in records if field in row]
    return [row["sim_time_us"] for row in selected], [row[field] for row in selected]


def previous(records, stamps, stamp, max_age_us=100000):
    index = bisect.bisect_right(stamps, stamp) - 1
    if index < 0 or stamp - stamps[index] > max_age_us:
        return None
    return records[index]


def summarize(run):
    config = json.loads((run / "map.json").read_text(encoding="utf-8"))
    truth = rows(run / "truth_evaluation_only.jsonl")
    attempts = []
    for spawn in config["spawns"]:
        vehicle = spawn["id"]
        log = rows(run / f"companion_{vehicle}.jsonl")
        estimates = [row for row in log if row.get("kind") == "estimate"]
        controls = [row for row in log if row.get("kind") == "control_intention"]
        control_stamps = [row["sim_time_us"] for row in controls]
        positions = [row for row in truth if row["vehicle"] == vehicle]
        measured = {key: series(positions, key) for key in ("x_m", "y_m", "yaw_rad")}
        estimated = {key: series(estimates, key) for key in
                     ("x_m", "y_m", "vy_mps", "yaw_rad")}
        estimate_stamps = [row["sim_time_us"] for row in estimates]
        commits = [row for row in log if row.get("kind") == "dive_aim" and row.get("committed")]
        for commit in commits:
            stamp = commit["sim_time_us"]
            true_y = interpolate(*measured["y_m"], stamp)
            estimate_y = commit.get("self_y_m")
            if estimate_y is None:
                estimate_y = interpolate(*estimated["y_m"], stamp)
            item = {"vehicle": vehicle, "commit_time_s": stamp / 1e6,
                    "aim_y_m": commit["aim_y_m"], "source": commit["aim_source"],
                    "commit_y_bias_mm": (estimate_y - true_y) * 1000
                        if estimate_y is not None and true_y is not None else None,
                    "samples": []}
            for offset in range(-300000, 600001, 100000):
                sample_us = stamp + offset
                sample = {"offset_s": offset / 1e6}
                for key in ("x_m", "y_m", "yaw_rad"):
                    sample[f"truth_{key}"] = interpolate(*measured[key], sample_us)
                    sample[f"estimate_{key}"] = interpolate(*estimated[key], sample_us)
                sample["estimate_vy_mps"] = interpolate(*estimated["vy_mps"], sample_us)
                # Central differences use a 40 ms window, only within recorded truth spans.
                before = interpolate(*measured["y_m"], sample_us - 20000)
                after = interpolate(*measured["y_m"], sample_us + 20000)
                sample["truth_vy_mps"] = (after - before) / .04 if before is not None and after is not None else None
                control = previous(controls, control_stamps, sample_us)
                estimate = previous(estimates, estimate_stamps, sample_us)
                if estimate:
                    sample["localization_mode"] = estimate.get("localization_mode")
                    sample["xy_valid"] = estimate.get("xy_valid")
                    sample["flow_valid"] = estimate.get("flow_valid")
                if control:
                    sample["control_mode"] = control["mode"]
                    yaw = sample["estimate_yaw_rad"]
                    if yaw is not None:
                        sample["command_world_vy_mps"] = (math.sin(yaw) * control["vx_flu_mps"]
                            + math.cos(yaw) * control["vy_flu_mps"])
                item["samples"].append(sample)
            attempts.append(item)
    return {"run": str(run), "truth_velocity_window_ms": 40, "attempts": attempts}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.run)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"commits": len(result["attempts"]),
        "commit_y_bias_mm": [row["commit_y_bias_mm"] for row in result["attempts"]]}))


if __name__ == "__main__":
    main()
