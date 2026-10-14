"""Compare committed guidance and pressure-plane crossings with evaluator truth."""

import argparse
import bisect
import json
from pathlib import Path


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
    return sorted(result, key=lambda row: row.get("sim_time_us", 0))


def interpolate(stamps, values, stamp, max_gap_us=200000):
    index = bisect.bisect_left(stamps, stamp)
    if index < len(stamps) and stamps[index] == stamp:
        return values[index]
    if index == 0 or index == len(stamps):
        return None
    gap = stamps[index] - stamps[index - 1]
    if gap <= 0 or gap > max_gap_us:
        return None
    fraction = (stamp - stamps[index - 1]) / gap
    return values[index - 1] + fraction * (values[index] - values[index - 1])


def summarize(run):
    config = json.loads((run / "map.json").read_text(encoding="utf-8"))
    mission = config["team_mission"]
    anchor = mission["target"]
    # The aircraft center is evaluated 50 mm before the armor center plane.
    plane_x = anchor["x"] - .05
    armor = rows(run / "armor_truth.jsonl")
    stamps = [row["sim_time_us"] for row in armor]
    values = [row["y_m"] + anchor["y"] for row in armor]
    truth = rows(run / "truth_evaluation_only.jsonl")
    pressure = [row for row in rows(run / "truth_evaluation_only_contacts.jsonl")
                if row.get("kind") == "armor_pressure"]
    team = rows(run / "team_mission.jsonl")
    final = team[-1] if team else {}
    attempts = []
    vehicles = []
    alignment = []
    retries = []
    for spawn in config["spawns"]:
        vehicle = spawn["id"]
        log = rows(run / f"companion_{vehicle}.jsonl")
        positions = [row for row in truth if row["vehicle"] == vehicle]
        commits = [row for row in log if row.get("kind") == "dive_aim" and row.get("committed")]
        hits = [row for row in pressure if row["vehicle"] == vehicle]
        stops = [row for row in log if row.get("kind") == "stop"]
        checks = [row for row in log if row.get("kind") == "dive_alignment"]
        retries.append({"vehicle": vehicle,
                        "misses": sum(row.get("kind") == "dive_miss" for row in log),
                        "weak_contacts": sum(row.get("kind") == "weak_impact" for row in log),
                        "entry_reacquisitions": [row for row in log if row.get("kind") == "dive_retry_entry"],
                        "commit_gate_blocks": [row for row in log if row.get("kind") == "dive_commit_gate"]})
        if checks:
            cfg = mission["dive_attack"]
            alignment.append({"vehicle": vehicle, "checks": len(checks),
                "ready_checks": sum(row["ready"] for row in checks),
                "commits": sum(row["committed"] for row in checks),
                "lateral_close_checks": sum(abs(row["error_y_m"]) <= cfg.get("entry_alignment_tolerance_m", .06) for row in checks),
                "height_close_checks": sum(abs(row["error_z_m"]) <= cfg.get("entry_height_tolerance_m", .12) for row in checks),
                "relative_speed_close_checks": sum(abs(row["vy_mps"] - row["target_vy_mps"]) <= cfg.get("entry_alignment_speed_tolerance_mps", .2) for row in checks)})
        vehicles.append({"vehicle": vehicle, "qualified_contacts": sum(bool(row.get("qualified")) for row in hits),
                         "pressure_events": len(hits), "armed_at_stop": stops[-1].get("armed_at_stop") if stops else None,
                         "final_position": positions[-1] if positions else None})
        for number, commit in enumerate(commits):
            start = commit["sim_time_us"]
            end = min(start + int(mission["dive_attack"].get("dive_timeout_s", 1.2) * 1e6),
                      commits[number + 1]["sim_time_us"] if number + 1 < len(commits) else float("inf"))
            corrections = [row for row in log if row.get("kind") == "dive_aim_correction" and start < row["sim_time_us"] <= end]
            item = {"vehicle": vehicle, "commit_time_s": start / 1e6, "source": commit["aim_source"],
                    "estimated_arrival_s": commit["lead_s"], "initial_aim_y_m": commit["aim_y_m"],
                    "corrections": len(corrections), "crossing": None}
            predicted_y = interpolate(stamps, values, start + commit["lead_s"] * 1e6)
            item["forecast_error_mm"] = 1000 * (commit["aim_y_m"] - predicted_y) if predicted_y is not None else None
            if "self_y_m" in commit:
                item["commit_state"] = {key: commit.get(key) for key in
                    ("entry_alignment", "self_x_m", "self_y_m", "self_z_m", "self_vx_mps", "self_vy_mps", "track_y_m", "track_vy_mps", "track_valid", "track_measured", "measurement_time_us", "commit_eligible")}
                exposure = commit.get("measurement_time_us")
                item["commit_observation_age_ms"] = (start - exposure) / 1000 if exposure is not None else None
            if commit.get("track_valid") and commit["aim_source"].startswith("linear"):
                direct = commit["track_y_m"] + commit["track_vy_mps"] * commit["lead_s"]
                width = mission["dive_attack"].get("aim_lateral_limit_m", .35)
                direct = max(anchor["y"] - width, min(anchor["y"] + width, direct))
                item["unconstrained_tracker_aim_y_m"] = direct
                item["unconstrained_tracker_forecast_error_mm"] = (direct - predicted_y) * 1000 if predicted_y is not None else None
            for previous, current in zip(positions, positions[1:]):
                if current["sim_time_us"] < start or previous["sim_time_us"] > end:
                    continue
                if previous["x_m"] < plane_x <= current["x_m"]:
                    fraction = (plane_x - previous["x_m"]) / (current["x_m"] - previous["x_m"])
                    stamp = previous["sim_time_us"] + fraction * (current["sim_time_us"] - previous["sim_time_us"])
                    if not start <= stamp <= end or current["sim_time_us"] - previous["sim_time_us"] > 200000:
                        continue
                    y = previous["y_m"] + fraction * (current["y_m"] - previous["y_m"])
                    z = previous["z_m"] + fraction * (current["z_m"] - previous["z_m"])
                    target_y = interpolate(stamps, values, stamp)
                    fresh = [row for row in corrections if row["sim_time_us"] <= stamp]
                    aim_y = fresh[-1]["aim_y_m"] if fresh else commit["aim_y_m"]
                    item["crossing"] = {"time_s": stamp / 1e6, "actual_arrival_s": (stamp - start) / 1e6,
                                        "y_error_mm": (y - target_y) * 1000 if target_y is not None else None,
                                        "z_error_mm": (z - anchor["z"]) * 1000,
                                        "last_aim_y_m": aim_y,
                                        "aim_error_mm": (aim_y - target_y) * 1000 if target_y is not None else None}
                    break
            attempts.append(item)
    return {"run": str(run), "plane_x_m": plane_x,
            "terminal": {key: final.get(key) for key in ("phase", "sim_time_us", "completed", "skipped", "reason")},
            "vehicles": vehicles, "attempts": attempts, "alignment": alignment, "retries": retries}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = summarize(args.run)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"terminal": result["terminal"], "contacts": {row["vehicle"]: row["qualified_contacts"] for row in result["vehicles"]},
                      "attempts": result["attempts"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
