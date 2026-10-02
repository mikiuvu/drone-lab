"""Summarize flight time, physical contacts and home offsets from a SITL run."""

import argparse
import json
import math
from pathlib import Path


def rows(path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def summarize(folder):
    config = json.loads((folder / "map.json").read_text(encoding="utf-8"))
    mission = config["team_mission"]
    phases = []
    previous = None
    for row in rows(folder / "team_mission.jsonl"):
        key = (row["phase"], row["active_vehicle"])
        if key != previous:
            phases.append(
                {
                    "time_s": row["sim_time_us"] / 1e6,
                    "phase": key[0],
                    "active_vehicle": key[1],
                }
            )
            previous = key
    takeoff_phase = next(
        (r["time_s"] for r in phases if r["phase"] == "TAKEOFF"), math.inf
    )
    landing_phase = next(
        (r["time_s"] for r in phases if r["phase"] == "LAND_ALL"), math.inf
    )
    completion = next((r["time_s"] for r in phases if r["phase"] == "COMPLETE"), None)
    homes = {r["id"]: r for r in config["spawns"]}
    lift, land, final = {}, {}, {}
    for row in rows(folder / "truth_evaluation_only.jsonl"):
        i, time_s = row["vehicle"], row["sim_time_us"] / 1e6
        final[i] = row
        if (
            time_s > takeoff_phase
            and row["z_m"]
            > mission["home_ground_z_m"] + mission["body_clearance_m"] + 0.03
        ):
            lift.setdefault(i, time_s)
        if time_s > landing_phase and row.get("landed"):
            land.setdefault(i, time_s)
    contacts = {i: [] for i in homes}
    for row in rows(folder / "truth_evaluation_only_contacts.jsonl"):
        if row.get("enemy_base_contact"):
            contacts[row["vehicle"]].append(row["sim_time_us"] / 1e6)
    vehicles = []
    for i, home in homes.items():
        impacts, stopped = [], None
        for row in rows(folder / f"companion_{i}.jsonl"):
            if row["kind"] == "impact":
                impacts.append(row["sim_time_us"] / 1e6)
            if row["kind"] == "stop":
                stopped = row["armed_at_stop"]
        point = final.get(i)
        vehicles.append(
            {
                "id": i,
                "takeoff_s": lift.get(i),
                "landed_s": land.get(i),
                "impact_s": impacts,
                "physical_contact_confirmed": bool(impacts)
                and all(
                    any(abs(t - other) < 0.3 for other in contacts[i]) for t in impacts
                ),
                "home_error_m": math.hypot(
                    point["x_m"] - home["x"], point["y_m"] - home["y"]
                )
                if point
                else None,
                "armed_at_stop": stopped,
            }
        )
    start = min(lift.values()) if lift else None
    return {
        "run": folder.name,
        "vehicle_count": len(homes),
        "phases": phases,
        "physical_flight_s": max(land.values()) - start
        if len(land) == len(homes) and start is not None
        else None,
        "confirmed_mission_s": completion - start
        if completion is not None and start is not None
        else None,
        "complete": completion is not None,
        "vehicles": vehicles,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = summarize(args.run_dir)
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    print(text)
