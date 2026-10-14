"""Verify return-route handoffs against the coordinator's received vehicle states."""

import argparse
import json
import math
from pathlib import Path

from summarize_contact_guidance import rows


def active_commands(row):
    return {command["vehicle"]: command for command in row["commands"]
            if not command["skipped"] and not command["degraded"]}


def summarize(run):
    config = json.loads((run / "map.json").read_text(encoding="utf-8"))
    tolerance = config["team_mission"].get("return_tolerance_m", .1)
    log = rows(run / "team_mission.jsonl")
    events = []
    for before, after in zip(log, log[1:]):
        if before["phase"] not in ("RETURN_ALL", "ABORT_RETURN"):
            continue
        if after["phase"] not in (before["phase"], "LAND_ALL"):
            continue
        previous = active_commands(before)
        following = active_commands(after)
        vehicles = sorted(previous.keys() & following.keys())
        if not any(any(previous[vehicle][key] != following[vehicle][key]
                       for key in ("target_x_m", "target_y_m", "target_z_m")) for vehicle in vehicles):
            continue
        states = {state["vehicle"]: state for state in after["vehicles"]}
        checks = []
        for vehicle in vehicles:
            state = states[vehicle]
            required = ("state_time_us", "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps", "armed")
            missing = [key for key in required if key not in state]
            check = {"vehicle": vehicle, "missing_fields": missing, "passed": False}
            if not missing:
                target = previous[vehicle]
                xy = math.hypot(state["x_m"] - target["target_x_m"], state["y_m"] - target["target_y_m"])
                height = abs(state["z_m"] - target["target_z_m"])
                speed = math.sqrt(sum(state[key] ** 2 for key in ("vx_mps", "vy_mps", "vz_mps")))
                age = after["sim_time_us"] - state["state_time_us"]
                valid = state["seen"] and state["fresh"] and state["ready"] and state["xy_valid"] and state["z_valid"] and state["armed"]
                check.update({"state_time_us": state["state_time_us"], "state_age_ms": age / 1000,
                              "xy_error_m": xy, "height_error_m": height, "speed_mps": speed,
                              "passed": bool(valid and age >= 0 and xy <= tolerance + 1e-9
                                             and height <= .08 + 1e-9 and speed <= .3 + 1e-9)})
            checks.append(check)
        events.append({"sim_time_us": after["sim_time_us"], "from": before["phase"],
                       "to": after["phase"], "checks": checks})
    return {"run": str(run), "xy_tolerance_m": tolerance, "height_tolerance_m": .08,
            "speed_tolerance_mps": .3, "events": events,
            "all_settled": bool(events) and all(event["checks"] and all(check["passed"] for check in event["checks"])
                                               for event in events)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-settled", action="store_true")
    args = parser.parse_args()
    result = summarize(args.run)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"events": len(result["events"]), "all_settled": result["all_settled"]}))
    if args.require_settled and not result["all_settled"]:
        raise SystemExit("Return transitions lack received-state settling confirmation")


if __name__ == "__main__":
    main()
