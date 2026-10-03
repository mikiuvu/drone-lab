# @file trial_report.py
# @brief 汇总 SITL 运行的阶段、实际飞行时间、接触证据和末帧返航偏移。

"""Summarize flight time, physical contacts and home offsets from a SITL run."""

import argparse
import json
import math
from pathlib import Path


# 逐行解析 UTF-8 JSONL 并跳过 JSON 语法损坏行；文件读取异常继续向调用方传播。
def rows(path):
    """@brief 按行读取 UTF-8 JSONL，并跳过语法损坏的记录。

    @param path JSONL 文件路径。
    @return 解码后的 JSON 对象迭代器。
    """
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


# @param folder 包含地图、任务、真值、接触和每机伴随日志的运行目录。
# @return 阶段时间、各机起落/撞击、接触确认和末帧 XY 返航偏移组成的字典。
# @note 起飞取 TAKEOFF 后超过离地阈值的首帧，落地取 LAND_ALL 后 landed 首帧；时间单位秒。
# 每次估计撞击须在 0.3 s 内找到目标物理接触，全部匹配且存在撞击才确认；所有飞机落地后才计算总飞行时长。
def summarize(folder):
    """@brief 从一组运行日志生成飞行阶段、接触和返航误差报告。

    @param folder 运行目录，包含 map.json 和各类 JSONL 日志。
    @return 报告字典；时间单位 s，位置和误差单位 m。
    """
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
